using System;
using System.Buffers;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Threading.Tasks;
using Microsoft.Extensions.ObjectPool;
using Robust.Shared.GameObjects;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Dynamics.Contacts;
using Robust.Shared.Physics.Dynamics.Joints;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Systems;

/*
 * These comments scabbed directly from Box2D and the licence applies to them.
 */

// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.
Here are the algorithms:
Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.
Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.
Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.
Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.
Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.
Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.
Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.
Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.
Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.
Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.
This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.
Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.
Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance
The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.
The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

public abstract partial class SharedPhysicsSystem
{
    /*
     * Handles island generation and constraints solver code.
     */
    private const int MaxIslands = 256;

    private readonly ObjectPool<List<Entity<PhysicsComponent, TransformComponent>>> _islandBodyPool =
        new DefaultObjectPool<List<Entity<PhysicsComponent, TransformComponent>>>(new ListPolicy<Entity<PhysicsComponent, TransformComponent>>(), MaxIslands);

    private readonly ObjectPool<List<Contact>> _islandContactPool =
        new DefaultObjectPool<List<Contact>>(new ListPolicy<Contact>(), MaxIslands);

    /// <summary>
    /// Due to joint relays we need to track the dummy joint and the original joint.
    /// </summary>
    private readonly ObjectPool<List<(Joint Original, Joint Joint)>> _islandJointPool =
        new DefaultObjectPool<List<(Joint Original, Joint Joint)>>(new ListPolicy<(Joint Original, Joint Joint)>(), MaxIslands);

    // The per-island broken-joint list was the one un-pooled list in the island data, allocated
    // fresh per island per tick. Pool it like the other three; ListPolicy clears it on return.
    private readonly ObjectPool<List<(Joint Joint, float Error)>> _islandBrokenJointPool =
        new DefaultObjectPool<List<(Joint Joint, float Error)>>(new ListPolicy<(Joint Joint, float Error)>(), MaxIslands);

    // Reused across steps to avoid per-tick allocation. Solve runs once per step, sequentially,
    // so a single reused buffer is safe and keeps the grown backing array instead of re-allocating it.
    private readonly List<IslandData> _islands = new();
    private readonly List<(Joint Original, Joint Joint)> _islandJoints = new();
    private readonly ParallelOptions _solveParallelOptions = new();

    // Reused per step to avoid the per-tick islands.ToArray(). Only grows; the entries past the
    // current island count are stale and never read (every loop is bounded by the live island count).
    private IslandData[] _actualIslands = Array.Empty<IslandData>();

    internal record struct IslandData(
        int Index,
        bool LoneIsland,
        List<Entity<PhysicsComponent, TransformComponent>> Bodies,
        List<Contact> Contacts,
        List<(Joint Original, Joint Joint)> Joints,
        List<(Joint Joint, float Error)> BrokenJoints)
    {
        /// <summary>
        /// MapUid for the island.
        /// </summary>
        public EntityUid MapUid;

        /// <summary>
        /// Island index to be used for bodies to identify which island they're in.
        /// </summary>
        public readonly int Index = Index;

        /// <summary>
        /// Are we the special island that has all contact-less bodies in it?
        /// This is treated separately for sleep purposes.
        /// </summary>
        public readonly bool LoneIsland = LoneIsland;

        /// <summary>
        /// Offset in the data arrays
        /// </summary>
        public int Offset = 0;

        public readonly List<Entity<PhysicsComponent, TransformComponent>> Bodies = Bodies;
        public readonly List<Contact> Contacts = Contacts;
        public readonly List<(Joint Original, Joint Joint)> Joints = Joints;
        public bool PositionSolved = false;
        public readonly List<(Joint Joint, float Error)> BrokenJoints = BrokenJoints;
    }

    // Caching for island generation.
    private readonly HashSet<Entity<PhysicsComponent, TransformComponent>> _islandSet = new(64);
    private readonly Stack<Entity<PhysicsComponent, TransformComponent>> _bodyStack = new(64);
    private readonly List<Entity<PhysicsComponent, TransformComponent>> _awakeBodyList = new(256);

    // Config
    private bool _warmStarting;
    private float _maxLinearCorrection;
    private float _maxAngularCorrection;
    private int _velocityIterations;
    private int _positionIterations;
    private float _maxLinearVelocity;
    private float _maxAngularVelocity;
    private float _maxTranslationPerTick;
    private float _maxRotationPerTick;
    private int _tickRate;
    private bool _sleepAllowed;
    protected float AngularToleranceSqr;
    protected float LinearToleranceSqr;
    protected float TimeToSleep;
    private float _velocityThreshold;
    private float _baumgarte;

    private const int VelocityConstraintsPerThread = 16;
    private const int PositionConstraintsPerThread = 16;

    // Hyperion: box2d-v3-style graph coloring for the internally-parallel (big) islands. Contacts in a color
    // share no *dynamic* body, so a color's contacts can be solved in parallel without racing on a body's
    // velocity slot (the old code raced over arbitrary contact batches). Static/kinematic bodies are exempt
    // (their velocity writes are no-ops). Keeps the existing 2.x impulse math, so it stays compatible with the
    // pinned stock client. Buffers reused per-step: big islands solve serially, so one buffer set is race-free.
    // Hyperion: raised 12 -> 24 (A5). The bitmask is a ulong (64 bits of headroom to spare); more colors
    // shrink the always-serial overflow range on high-contact big islands. Shifts which contacts land in
    // overflow vs. get a color, so solve order changes slightly -> sub-epsilon float drift only.
    private const int SolverGraphColorCount = 24;
    private readonly int[] _colorStarts = new int[SolverGraphColorCount + 2]; // [c]..[c+1] = color c range; [24]=overflow start; [25]=total
    private int _colorCount;
    private int[] _contactColorOf = Array.Empty<int>();
    private ulong[] _bodyColorBits = Array.Empty<ulong>();
    private ContactVelocityConstraint[] _vcSorted = Array.Empty<ContactVelocityConstraint>();
    private ContactPositionConstraint[] _pcSorted = Array.Empty<ContactPositionConstraint>();

    #region Setup

    private void InitializeIsland()
    {
        Subs.CVar(_cfg, CVars.NetTickrate, SetTickRate, true);
        Subs.CVar(_cfg, CVars.WarmStarting, SetWarmStarting, true);
        Subs.CVar(_cfg, CVars.MaxLinearCorrection, SetMaxLinearCorrection, true);
        Subs.CVar(_cfg, CVars.MaxAngularCorrection, SetMaxAngularCorrection, true);
        Subs.CVar(_cfg, CVars.VelocityIterations, SetVelocityIterations, true);
        Subs.CVar(_cfg, CVars.PositionIterations, SetPositionIterations, true);
        Subs.CVar(_cfg, CVars.MaxLinVelocity, SetMaxLinearVelocity, true);
        Subs.CVar(_cfg, CVars.MaxAngVelocity, SetMaxAngularVelocity, true);
        Subs.CVar(_cfg, CVars.SleepAllowed, SetSleepAllowed, true);
        Subs.CVar(_cfg, CVars.AngularSleepTolerance, SetAngularToleranceSqr, true);
        Subs.CVar(_cfg, CVars.LinearSleepTolerance, SetLinearToleranceSqr, true);
        Subs.CVar(_cfg, CVars.TimeToSleep, SetTimeToSleep, true);
        Subs.CVar(_cfg, CVars.VelocityThreshold, SetVelocityThreshold, true);
        Subs.CVar(_cfg, CVars.Baumgarte, SetBaumgarte, true);
    }

    private void SetWarmStarting(bool value) => _warmStarting = value;
    private void SetMaxLinearCorrection(float value) => _maxLinearCorrection = value;
    private void SetMaxAngularCorrection(float value) => _maxAngularCorrection = value;
    private void SetVelocityIterations(int value) => _velocityIterations = value;
    private void SetPositionIterations(int value) => _positionIterations = value;
    private void SetMaxLinearVelocity(float value)
    {
        _maxLinearVelocity = value;
        UpdateMaxTranslation();
    }

    private void SetMaxAngularVelocity(float value)
    {
        _maxAngularVelocity = value;
        UpdateMaxRotation();
    }

    private void SetTickRate(int value)
    {
        _tickRate = value;
        UpdateMaxTranslation();
        UpdateMaxRotation();
    }

    private void SetSleepAllowed(bool value) => _sleepAllowed = value;
    private void SetAngularToleranceSqr(float value) => AngularToleranceSqr = value;
    private void SetLinearToleranceSqr(float value) => LinearToleranceSqr = value;
    private void SetTimeToSleep(float value) => TimeToSleep = value;
    private void SetVelocityThreshold(float value) => _velocityThreshold = value;
    private void SetBaumgarte(float value) => _baumgarte = value;

    private void UpdateMaxTranslation()
    {
        _maxTranslationPerTick = _maxLinearVelocity / _tickRate;
    }

    private void UpdateMaxRotation()
    {
        _maxRotationPerTick = (MathF.Tau * _maxAngularVelocity) / _tickRate;
    }

    #endregion

    /// <summary>
    ///     Where the magic happens.
    /// </summary>
    public void Step(float frameTime, bool prediction)
    {
        var invDt = frameTime > 0.0f ? 1.0f / frameTime : 0.0f;
        var dtRatio = _invDt0 * frameTime;

        // Integrate velocities, solve velocity constraints, and do integration.
        Solve(frameTime, dtRatio, invDt, prediction);

        // TODO: SolveTOI

        // Box2d recommends clearing (if you are) during fixed updates rather than variable if you are using it
        if (_autoClearForces)
            ClearForces();

        _invDt0 = invDt;
    }

    private void ClearForces()
    {
        foreach (var ent in AwakeBodies)
        {
            var uid = ent.Owner;
            var body = ent.Comp1;

            if (body.Force != Vector2.Zero)
            {
                body.Force = Vector2.Zero;
                DirtyField(uid, body, nameof(PhysicsComponent.Force));
            }

            if (body.Torque != 0f)
            {
                body.Torque = 0f;
                DirtyField(uid, body, nameof(PhysicsComponent.Torque));
            }
        }
    }

    private void Solve(float frameTime, float dtRatio, float invDt, bool prediction)
    {
        // Build and simulated islands from awake bodies.
        _bodyStack.EnsureCapacity(AwakeBodies.Count);
        _islandSet.EnsureCapacity(AwakeBodies.Count);
        _awakeBodyList.AddRange(AwakeBodies);

        var islandIndex = 0;
        var loneIsland = new IslandData(
            islandIndex++,
            true,
            _islandBodyPool.Get(),
            _islandContactPool.Get(),
            _islandJointPool.Get(),
            _islandBrokenJointPool.Get());

        var islands = _islands;
        islands.Clear();
        var islandJoints = _islandJoints;
        islandJoints.Clear();

        // Build the relevant islands / graphs for all bodies.
        foreach (var ent in _awakeBodyList)
        {
            var xform = ent.Comp2;
            var seed = ent.Comp1;

            // I tried not running prediction for non-contacted entities but unfortunately it looked like shit
            // when contact broke so if you want to try that then GOOD LUCK.
            if (seed.Island) continue;

            var seedUid = seed.Owner;
            var mapUid = xform.MapUid;

            // TODO: Handle this on client.
            if (mapUid == null)
            {
                continue;
            }

            if (!EntityManager.MetaQuery.TryGetComponent(seedUid, out var metadata))
            {
                Log.Error($"Found deleted entity {ToPrettyString(seedUid)} on map!");
                RemoveSleepBody(ent);
                continue;
            }

            if ((metadata.EntityPaused && !seed.IgnorePaused) ||
                (prediction && !seed.Predict) ||
                !seed.CanCollide ||
                seed.BodyType == BodyType.Static)
            {
                continue;
            }

            // Start of a new island
            var bodies = _islandBodyPool.Get();
            var contacts = _islandContactPool.Get();
            var joints = _islandJointPool.Get();
            _bodyStack.Push(ent);

            seed.Island = true;

            while (_bodyStack.TryPop(out var bodyEnt))
            {
                var bodyUid = bodyEnt.Owner;
                var body = bodyEnt.Comp1;

                bodies.Add(bodyEnt);

                _islandSet.Add(bodyEnt);

                // Static bodies don't propagate islands
                if (body.BodyType == BodyType.Static) continue;

                // As static bodies can never be awake (unlike Farseer) we'll set this after the check.
                SetAwake(bodyUid, body, true, updateSleepTime: false);

                var node = body.Contacts.First;

                while (node != null)
                {
                    var contact = node.Value;
                    node = node.Next;

                    // Has this contact already been added to an island / is it pre-init?
                    if ((contact.Flags & (ContactFlags.Island | ContactFlags.PreInit)) != 0x0) continue;

                    // Is this contact solid and touching?
                    if (!contact.Enabled || !contact.IsTouching) continue;

                    // Skip sensors.
                    if (contact.FixtureA?.Hard != true || contact.FixtureB?.Hard != true) continue;

                    contacts.Add(contact);
                    contact.Flags |= ContactFlags.Island;
                    var other = contact.OtherBody(bodyUid);

                    // Was the other body already added to this island?
                    if (other.Island) continue;

                    var otherEnt = contact.OtherEnt(bodyUid);
                    var otherXform = contact.OtherTransform(bodyUid);
                    // TODO: Store this transform on the component directly.
                    _bodyStack.Push(new Entity<PhysicsComponent, TransformComponent>(otherEnt, other, otherXform));
                    other.Island = true;
                }

                // Handle joints
                if (RelayTargetQuery.TryGetComponent(bodyUid, out var relayComp))
                {
                    foreach (var relay in relayComp.Relayed)
                    {
                        if (!JointQuery.TryGetComponent(relay, out var jointComp))
                            continue;

                        foreach (var joint in jointComp.GetJoints.Values)
                        {
                            if (joint.IslandFlag)
                                continue;

                            var uidA = joint.BodyAUid;
                            var uidB = joint.BodyBUid;
                            DebugTools.AssertNotEqual(uidA, uidB);

                            if (JointQuery.TryGetComponent(uidA, out var jointCompA) &&
                                jointCompA.Relay != null)
                            {
                                DebugTools.AssertNotEqual(uidB, jointCompA.Relay.Value);
                                uidA = jointCompA.Relay.Value;
                            }

                            if (JointQuery.TryGetComponent(uidB, out var jointCompB) &&
                                jointCompB.Relay != null)
                            {
                                DebugTools.AssertNotEqual(uidA, jointCompB.Relay.Value);
                                uidB = jointCompB.Relay.Value;
                            }

                            // Only clone when a relay actually remapped an endpoint. Without a remap the clone
                            // is an identical copy pointing at the same bodies, so we can solve the original
                            // directly; ReturnIsland skips the no-op CopyTo when original == joint.
                            var copy = uidA == joint.BodyAUid && uidB == joint.BodyBUid
                                ? joint
                                : joint.Clone(uidA, uidB);
                            islandJoints.Add((joint, copy));
                            joint.IslandFlag = true;
                        }
                    }
                }

                if (JointQuery.TryGetComponent(bodyUid, out var jointComponent) &&
                    jointComponent.Relay == null)
                {
                    foreach (var joint in jointComponent.Joints.Values)
                    {
                        if (joint.IslandFlag)
                            continue;

                        var uidA = joint.BodyAUid;
                        var uidB = joint.BodyBUid;

                        if (JointQuery.TryGetComponent(uidA, out var jointCompA) &&
                            jointCompA.Relay != null)
                        {
                            uidA = jointCompA.Relay.Value;
                        }

                        if (JointQuery.TryGetComponent(uidB, out var jointCompB) &&
                            jointCompB.Relay != null)
                        {
                            uidB = jointCompB.Relay.Value;
                        }

                        // See the relay-target path above: skip the clone when no endpoint was remapped.
                        var copy = uidA == joint.BodyAUid && uidB == joint.BodyBUid
                            ? joint
                            : joint.Clone(uidA, uidB);
                        islandJoints.Add((joint, copy));
                        joint.IslandFlag = true;
                    }
                }

                foreach (var (original, joint) in islandJoints)
                {
                    // TODO: Same here store physicscomp + transform on the joint, the savings are worth it.
                    var bodyA = PhysicsQuery.GetComponent(joint.BodyAUid);
                    var bodyB = PhysicsQuery.GetComponent(joint.BodyBUid);

                    if (!bodyA.CanCollide || !bodyB.CanCollide)
                        continue;

                    joints.Add((original, joint));

                    if (!bodyA.Island)
                    {
                        _bodyStack.Push(new Entity<PhysicsComponent, TransformComponent>(joint.BodyAUid, bodyA, XformQuery.GetComponent(joint.BodyAUid)));
                        bodyA.Island = true;
                    }

                    if (!bodyB.Island)
                    {
                        _bodyStack.Push(new Entity<PhysicsComponent, TransformComponent>(joint.BodyBUid, bodyB, XformQuery.GetComponent(joint.BodyBUid)));
                        bodyB.Island = true;
                    }
                }

                islandJoints.Clear();
            }

            int idx;

            // Bodies not touching anything, hence we can just add it to the lone island.
            if (contacts.Count == 0 && joints.Count == 0)
            {
                DebugTools.Assert(bodies.Count == 1 && bodies[0].Comp1.BodyType != BodyType.Static);
                loneIsland.MapUid = mapUid.Value;
                loneIsland.Bodies.Add(bodies[0]);
                idx = loneIsland.Index;
            }
            else
            {
                var data = new IslandData(islandIndex++, false, bodies, contacts, joints, _islandBrokenJointPool.Get())
                {
                    MapUid = mapUid.Value
                };
                islands.Add(data);
                idx = data.Index;
            }

            // Allow static bodies to be re-used in other islands
            for (var i = 0; i < bodies.Count; i++)
            {
                var body = bodies[i].Comp1;

                // Static bodies can participate in other islands
                if (body.BodyType == BodyType.Static)
                {
                    body.Island = false;
                }

                body.IslandIndex[idx] = i;
            }
        }

        // If we didn't use lone island just return it.
        if (loneIsland.Bodies.Count > 0)
        {
            islands.Add(loneIsland);
        }
        else
        {
            ReturnIsland(loneIsland);
        }

        SolveIslands(islands, frameTime, dtRatio, invDt, prediction);

        foreach (var island in islands)
        {
            ReturnIsland(island);
        }

        Cleanup(frameTime);
    }

    private void ReturnIsland(in IslandData island)
    {
        foreach (var bodyEnt in island.Bodies)
        {
            var body = bodyEnt.Comp1;
            DebugTools.Assert(body.IslandIndex.ContainsKey(island.Index));
            body.IslandIndex.Remove(island.Index);
        }

        _islandBodyPool.Return(island.Bodies);
        _islandContactPool.Return(island.Contacts);

        foreach (var (original, joint) in island.Joints)
        {
            // Do we need to copy data back to the original?
            if (original != joint)
            {
                joint.CopyTo(original);
            }

            original.IslandFlag = false;
        }

        _islandJointPool.Return(island.Joints);
        _islandBrokenJointPool.Return(island.BrokenJoints);
    }

    protected virtual void Cleanup(float frameTime)
    {
        foreach (var bodyEnt in _islandSet)
        {
            var body = bodyEnt.Comp1;

            if (!body.Island || body.Deleted)
            {
                continue;
            }

            body.Island = false;
            DebugTools.Assert(body.BodyType != BodyType.Static);

            // So Box2D would update broadphase here buutttt we'll just wait until MoveEvent queue is used.
        }

        _islandSet.Clear();
        _islandSet.Clear();
        _awakeBodyList.Clear();
    }

    private void SolveIslands(List<IslandData> islands, float frameTime, float dtRatio, float invDt, bool prediction)
    {
        var iBegin = 0;
        var data = new SolverData(
            frameTime,
            dtRatio,
            invDt,
            _warmStarting,
            _maxLinearCorrection,
            _maxAngularCorrection,
            _velocityIterations,
            _positionIterations,
            _maxLinearVelocity,
            _maxAngularVelocity,
            _maxTranslationPerTick,
            _maxRotationPerTick,
            _sleepAllowed,
            AngularToleranceSqr,
            LinearToleranceSqr,
            TimeToSleep,
            _velocityThreshold,
            _baumgarte
        );

        // We'll sort islands from internally parallel (due to lots of contacts) to running all the islands in parallel
        islands.Sort(static (x, y) => InternalParallel(y).CompareTo(InternalParallel(x)));

        var totalBodies = 0;
        var islandCount = islands.Count;
        if (_actualIslands.Length < islandCount)
            Array.Resize(ref _actualIslands, islandCount);
        islands.CopyTo(_actualIslands);
        var actualIslands = _actualIslands;

        for (var i = 0; i < islands.Count; i++)
        {
            ref var island = ref actualIslands[i];
            island.Offset = totalBodies;
            UpdateLerpData(island.Bodies);

#if DEBUG
            RaiseLocalEvent(new IslandSolveMessage(island.Bodies));
#endif

            totalBodies += island.Bodies.Count;
        }

        // Actual solver here; cache the data for later.
        var solvedPositions = ArrayPool<Vector2>.Shared.Rent(totalBodies);
        var solvedAngles = ArrayPool<float>.Shared.Rent(totalBodies);
        var linearVelocities = ArrayPool<Vector2>.Shared.Rent(totalBodies);
        var angularVelocities = ArrayPool<float>.Shared.Rent(totalBodies);
        var sleepStatus = ArrayPool<bool>.Shared.Rent(totalBodies);
        // Cleanup any potentially stale data first.
        for (var i = 0; i < totalBodies; i++)
        {
            sleepStatus[i] = false;
        }

        var options = _solveParallelOptions;
        options.MaxDegreeOfParallelism = _parallel.ParallelProcessCount;

        while (iBegin < islandCount)
        {
            ref var island = ref actualIslands[iBegin];

            if (!InternalParallel(island))
                break;

            SolveIsland(ref island, in data, options, prediction, solvedPositions, solvedAngles, linearVelocities, angularVelocities, sleepStatus);
            iBegin++;
        }

        Parallel.For(iBegin, islandCount, options, i =>
        {
            ref var island = ref actualIslands[i];
            SolveIsland(ref island, in data, null, prediction, solvedPositions, solvedAngles, linearVelocities, angularVelocities, sleepStatus);
        });

        // Update data sequentially
        // Hyperion: WriteBack bucket (A2) = the post-solve write-back only (UpdateBodies + SleepBodies);
        // matches the existing ConstraintSolve Stopwatch/AddTicks idiom used in SolveIsland below.
        var writeBackStart = Stopwatch.GetTimestamp();
        for (var i = 0; i < islandCount; i++)
        {
            ref readonly var island = ref actualIslands[i];

            UpdateBodies(in island, solvedPositions, solvedAngles, linearVelocities, angularVelocities);
            SleepBodies(in island, sleepStatus);
        }
        PhaseProfiler.AddTicks(PhysicsPhase.WriteBack, Stopwatch.GetTimestamp() - writeBackStart);

        // Cleanup
        ArrayPool<Vector2>.Shared.Return(solvedPositions);
        ArrayPool<float>.Shared.Return(solvedAngles);
        ArrayPool<Vector2>.Shared.Return(linearVelocities);
        ArrayPool<float>.Shared.Return(angularVelocities);
        ArrayPool<bool>.Shared.Return(sleepStatus);
    }

    /// <summary>
    /// If this is the first time a body has been updated this tick update its position for lerping.
    /// Due to substepping we have to check it every time.
    /// </summary>
    protected virtual void UpdateLerpData(List<Entity<PhysicsComponent, TransformComponent>> bodies)
    {

    }

    /// <summary>
    /// Can we run the island in parallel internally, otherwise solve it in parallel with the rest.
    /// </summary>
    /// <param name="island"></param>
    /// <returns></returns>
    private static bool InternalParallel(IslandData island)
    {
        // Should lone island most times as well.
        return island.Bodies.Count > 128 || island.Contacts.Count > 128 || island.Joints.Count > 128;
    }

    /// <summary>
    ///     Go through all the bodies in this island and solve.
    /// </summary>
    // Hyperion: greedily color the island's contacts (box2d-v3 style) and reorder the velocity/position
    // constraint arrays so each color occupies a contiguous range. Two contacts get the same color only if they
    // share no *dynamic* body, so a color's contacts can be solved in parallel without racing on a body's
    // velocity slot. Contacts that don't fit in SolverGraphColorCount colors land in the overflow range (solved
    // serially). Result is published in _colorStarts / _colorCount. Only called for internally-parallel islands,
    // which are solved serially, so the reused buffers are race-free.
    private void ColorIsland(in IslandData island, ContactVelocityConstraint[] vc, ContactPositionConstraint[] pc, int bodyCount)
    {
        var contactCount = island.Contacts.Count;

        if (_contactColorOf.Length < contactCount)
            Array.Resize(ref _contactColorOf, contactCount);
        if (_vcSorted.Length < contactCount)
            Array.Resize(ref _vcSorted, contactCount);
        if (_pcSorted.Length < contactCount)
            Array.Resize(ref _pcSorted, contactCount);
        if (_bodyColorBits.Length < bodyCount)
            Array.Resize(ref _bodyColorBits, bodyCount);

        Array.Clear(_bodyColorBits, 0, bodyCount);
        var bodyBits = _bodyColorBits;
        var colorOf = _contactColorOf;

        const ulong colorMask = (1ul << SolverGraphColorCount) - 1;
        Span<int> counts = stackalloc int[SolverGraphColorCount + 1];
        counts.Clear();
        var maxColor = 0;

        for (var i = 0; i < contactCount; i++)
        {
            ref var c = ref vc[i];

            // A body only constrains the coloring if it's dynamic (static/kinematic velocity writes are no-ops).
            ulong used = 0;
            if (c.InvMassA > 0f)
                used |= bodyBits[c.IndexA];
            if (c.InvMassB > 0f)
                used |= bodyBits[c.IndexB];

            var avail = ~used & colorMask;
            int color;
            if (avail == 0)
            {
                color = SolverGraphColorCount; // overflow
            }
            else
            {
                color = BitOperations.TrailingZeroCount(avail);
                var bit = 1ul << color;
                if (c.InvMassA > 0f)
                    bodyBits[c.IndexA] |= bit;
                if (c.InvMassB > 0f)
                    bodyBits[c.IndexB] |= bit;
                if (color + 1 > maxColor)
                    maxColor = color + 1;
            }

            colorOf[i] = color;
            counts[color]++;
        }

        // Prefix-sum the counts into per-color start offsets.
        var starts = _colorStarts;
        var acc = 0;
        for (var col = 0; col <= SolverGraphColorCount; col++)
        {
            starts[col] = acc;
            acc += counts[col];
        }
        starts[SolverGraphColorCount + 1] = acc; // == contactCount

        // Counting-sort the constraints into color order. vc and pc are reordered by the same permutation, so
        // pc[k] stays the position constraint for the same contact as vc[k], and the coloring is valid for both.
        Span<int> cursor = stackalloc int[SolverGraphColorCount + 1];
        for (var col = 0; col <= SolverGraphColorCount; col++)
            cursor[col] = starts[col];

        for (var i = 0; i < contactCount; i++)
        {
            var pos = cursor[colorOf[i]]++;
            _vcSorted[pos] = vc[i];
            _pcSorted[pos] = pc[i];
        }

        Array.Copy(_vcSorted, vc, contactCount);
        Array.Copy(_pcSorted, pc, contactCount);

        _colorCount = maxColor;
    }

    private void SolveIsland(
        ref IslandData island,
        in SolverData data,
        ParallelOptions? options,
        bool prediction,
        Vector2[] solvedPositions,
        float[] solvedAngles,
        Vector2[] linearVelocities,
        float[] angularVelocities,
        bool[] sleepStatus)
    {
        var bodyCount = island.Bodies.Count;
        var positions = ArrayPool<Vector2>.Shared.Rent(bodyCount);
        var angles = ArrayPool<float>.Shared.Rent(bodyCount);
        var offset = island.Offset;
        var gravity = Gravity;
        var bodies = island.Bodies;

        // Hyperion (A3): prep loop (gravity/force/damping + world->local snapshot) writes only to its own
        // index (positions[i]/angles[i]/linearVelocities[i+offset]/angularVelocities[i+offset]), so it's
        // race-free to run in parallel the same way FinalisePositions already is. Gated on options != null,
        // the same gate ColorIsland uses below: that's only true on the internally-parallel big-island path.
        // Small islands get inter-island parallelism instead and must stay serial here to avoid nesting
        // Parallel.For inside an already-parallel-dispatched island solve.
        if (options != null)
        {
            _parallel.ProcessNow(new PrepareBodiesJob
            {
                System = this,
                Data = data,
                Offset = offset,
                Gravity = gravity,
                Bodies = bodies,
                Positions = positions,
                Angles = angles,
                LinearVelocities = linearVelocities,
                AngularVelocities = angularVelocities,
            }, bodyCount);
        }
        else
        {
            PrepareBodies(0, bodyCount, offset, gravity, in data, bodies, positions, angles, linearVelocities, angularVelocities);
        }

        var contactCount = island.Contacts.Count;
        var velocityConstraints = ArrayPool<ContactVelocityConstraint>.Shared.Rent(contactCount);
        var positionConstraints = ArrayPool<ContactPositionConstraint>.Shared.Rent(contactCount);

        // Pass the data into the solver
        ResetSolver(in data, in island, options, velocityConstraints, positionConstraints);

        InitializeVelocityConstraints(in data, in island, options, velocityConstraints, positionConstraints, positions, angles, linearVelocities, angularVelocities);

        // Hyperion: color + reorder constraints for the internally-parallel path (big islands only; small ones
        // get inter-island parallelism and solve serially internally). Must run after the constraints are built.
        if (options != null)
            ColorIsland(in island, velocityConstraints, positionConstraints, bodyCount);

        if (data.WarmStarting)
        {
            WarmStart(in data, in island, velocityConstraints, linearVelocities, angularVelocities);
        }

        var jointCount = island.Joints.Count;

        if (jointCount > 0)
        {
            for (var i = 0; i < island.Joints.Count; i++)
            {
                var joint = island.Joints[i].Joint;
                if (!joint.Enabled) continue;

                var bodyA = PhysicsQuery.GetComponent(joint.BodyAUid);
                var bodyB = PhysicsQuery.GetComponent(joint.BodyBUid);
                joint.InitVelocityConstraints(in data, in island, bodyA, bodyB, positions, angles, linearVelocities, angularVelocities);
            }
        }

        // Velocity solver
        // Hyperion: ConstraintSolve bucket = the velocity + position constraint iterations only
        // (the prep/warm-start and the integrate step between them are not counted here).
        var solveStart = Stopwatch.GetTimestamp();
        for (var i = 0; i < data.VelocityIterations; i++)
        {
            for (var j = 0; j < jointCount; ++j)
            {
                var joint = island.Joints[j].Joint;

                if (!joint.Enabled)
                    continue;

                joint.SolveVelocityConstraints(in data, in island, linearVelocities, angularVelocities);

                var error = joint.Validate(data.InvDt);

                if (error > 0.0f)
                    island.BrokenJoints.Add((island.Joints[j].Original, error));
            }

            SolveVelocityConstraints(in island, options, _colorStarts, _colorCount, velocityConstraints, linearVelocities, angularVelocities);
        }
        PhaseProfiler.AddTicks(PhysicsPhase.ConstraintSolve, Stopwatch.GetTimestamp() - solveStart);

        // Store for warm starting.
        StoreImpulses(in island, options, velocityConstraints);

        var maxVel = data.MaxTranslation / data.FrameTime;
        var maxVelSq = maxVel * maxVel;
        var maxAngVel = data.MaxRotation / data.FrameTime;
        var maxAngVelSq = maxAngVel * maxAngVel;

        // Integrate positions
        // Hyperion: Integrate bucket (A2) = the position-integration loop only. This runs per-island, possibly
        // concurrently with other islands under the outer Parallel.For in SolveIslands, so AddTicks' Interlocked.Add
        // keeps this race-free the same way ConstraintSolve already is.
        // A3: each index only touches its own linearVelocities[offset+i]/angularVelocities[offset+i]/
        // positions[i]/angles[i] slot, so (like PrepareBodies) it's race-free to fan out on the internally-
        // parallel big-island path. Same options != null gate as everywhere else in this method.
        var integrateStart = Stopwatch.GetTimestamp();
        if (options != null)
        {
            _parallel.ProcessNow(new IntegratePositionsJob
            {
                System = this,
                Offset = offset,
                FrameTime = data.FrameTime,
                MaxVel = maxVel,
                MaxVelSq = maxVelSq,
                MaxAngVel = maxAngVel,
                MaxAngVelSq = maxAngVelSq,
                Positions = positions,
                Angles = angles,
                LinearVelocities = linearVelocities,
                AngularVelocities = angularVelocities,
            }, bodyCount);
        }
        else
        {
            IntegratePositions(0, bodyCount, offset, data.FrameTime, maxVel, maxVelSq, maxAngVel, maxAngVelSq, positions, angles, linearVelocities, angularVelocities);
        }
        PhaseProfiler.AddTicks(PhysicsPhase.Integrate, Stopwatch.GetTimestamp() - integrateStart);

        island.PositionSolved = false;

        var posStart = Stopwatch.GetTimestamp();
        for (var i = 0; i < data.PositionIterations; i++)
        {
            var contactsOkay = SolvePositionConstraints(in data, in island, options, _colorStarts, _colorCount, positionConstraints, positions, angles);
            var jointsOkay = true;

            for (var j = 0; j < island.Joints.Count; ++j)
            {
                var joint = island.Joints[j].Joint;

                if (!joint.Enabled)
                    continue;

                var jointOkay = joint.SolvePositionConstraints(in data, positions, angles);

                jointsOkay = jointsOkay && jointOkay;
            }

            if (contactsOkay && jointsOkay)
            {
                island.PositionSolved = true;
                break;
            }
        }
        PhaseProfiler.AddTicks(PhysicsPhase.ConstraintSolve, Stopwatch.GetTimestamp() - posStart);

        // Transform the solved positions back into local terms
        // This means we can run the entire solver in parallel and not have to worry about stale world positions later
        // E.g. if a parent had its position updated then our worldposition is invalid
        // We can safely do this in parallel, and (A7) do: each index writes only its own
        // solvedPositions[offset+i]/solvedAngles[offset+i] slot, so batches never race.
        // (bodies == island.Bodies, already captured above for the PrepareBodies pass.)

        if (options != null)
        {
            // Hyperion (A7): was a static local `ProcessParallelInternal` wrapping a `Parallel.For` with a
            // captured lambda, allocating a closure every big-island tick. Converted to the engine's own
            // IParallelRobustJob record-struct pattern (mirrors ManifoldsJob in Contacts.cs / BroadphaseContactJob
            // in SharedBroadphaseSystem.cs): a value-type job dispatched via `_parallel.ProcessNow`, which batches
            // internally at BatchSize just like the old FinaliseBodies=32 chunking did, but through the engine's
            // own ThreadPool-based job queue instead of TPL Parallel.For, and with no per-tick delegate allocation.
            _parallel.ProcessNow(new FinalisePositionsJob
            {
                System = this,
                Offset = offset,
                Bodies = bodies,
                Positions = positions,
                Angles = angles,
                SolvedPositions = solvedPositions,
                SolvedAngles = solvedAngles,
            }, bodyCount);
        }
        else
        {
            FinalisePositions(0, bodyCount, offset, bodies, positions, angles, solvedPositions, solvedAngles);
        }

        // Check sleep status for all of the bodies
        // Writing sleep timer is safe but updating awake or not is not safe.

        // We have a special island for no-contact no-joint bodies and just run this custom sleeping behaviour
        // for it while still keeping the benefits of a big island.
        if (island.LoneIsland)
        {
            if (!prediction && data.SleepAllowed)
            {
                for (var i = 0; i < bodyCount; i++)
                {
                    var body = island.Bodies[i].Comp1;

                    if (body.BodyType == BodyType.Static) continue;

                    if (!body.SleepingAllowed ||
                        body.AngularVelocity * body.AngularVelocity > data.AngTolSqr ||
                        Vector2.Dot(body.LinearVelocity, body.LinearVelocity) > data.LinTolSqr)
                    {
                        SetSleepTime(body, 0f);
                    }
                    else
                    {
                        SetSleepTime(body, body.SleepTime + data.FrameTime);
                    }

                    if (body.SleepTime >= data.TimeToSleep && island.PositionSolved)
                    {
                        sleepStatus[offset + i] = true;
                    }
                }
            }
        }
        else
        {
            // Sleep bodies if needed. Prediction won't accumulate sleep-time for bodies.
            if (!prediction && data.SleepAllowed)
            {
                var minSleepTime = float.MaxValue;

                for (var i = 0; i < bodyCount; i++)
                {
                    var body = island.Bodies[i].Comp1;

                    if (body.BodyType == BodyType.Static) continue;

                    if (!body.SleepingAllowed ||
                        body.AngularVelocity * body.AngularVelocity > data.AngTolSqr ||
                        Vector2.Dot(body.LinearVelocity, body.LinearVelocity) > data.LinTolSqr)
                    {
                        SetSleepTime(body, 0f);
                        minSleepTime = 0.0f;
                    }
                    else
                    {
                        SetSleepTime(body, body.SleepTime + data.FrameTime);
                        minSleepTime = MathF.Min(minSleepTime, body.SleepTime);
                    }
                }

                if (minSleepTime >= data.TimeToSleep && island.PositionSolved)
                {
                    for (var i = 0; i < island.Bodies.Count; i++)
                    {
                        sleepStatus[offset + i] = true;
                    }
                }
            }
        }

        // Cleanup
        ArrayPool<Vector2>.Shared.Return(positions);
        ArrayPool<float>.Shared.Return(angles);
        ArrayPool<ContactVelocityConstraint>.Shared.Return(velocityConstraints);
        ArrayPool<ContactPositionConstraint>.Shared.Return(positionConstraints);
    }

    /// <summary>
    /// Snapshots world position/rotation into local (positions/angles), and applies gravity/force/damping into
    /// linearVelocities/angularVelocities, for bodies [start, end) of the island. Each index writes only its
    /// own slot, so this is safe to call over disjoint ranges concurrently (see PrepareBodiesJob).
    /// </summary>
    private void PrepareBodies(
        int start,
        int end,
        int offset,
        Vector2 gravity,
        in SolverData data,
        List<Entity<PhysicsComponent, TransformComponent>> bodies,
        Vector2[] positions,
        float[] angles,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        for (var i = start; i < end; i++)
        {
            var bodyEnt = bodies[i];
            var body = bodyEnt.Comp1;
            var xform = bodyEnt.Comp2;
            var (worldPos, worldRot) =
                _transform.GetWorldPositionRotation(xform);

            var transform = new Transform(worldPos, worldRot);
            var position = Physics.Transform.Mul(transform, body.LocalCenter);
            // DebugTools.Assert(!float.IsNaN(position.X) && !float.IsNaN(position.Y));
            var angle = transform.Quaternion2D.Angle;

            // var bodyTransform = body.GetTransform();
            // DebugTools.Assert(bodyTransform.Position.EqualsApprox(position) && MathHelper.CloseTo(angle, bodyTransform.Quaternion2D.Angle));

            var linearVelocity = body.LinearVelocity;
            var angularVelocity = body.AngularVelocity;

            // if the body cannot move, nothing to do here
            if (body.BodyType == BodyType.Dynamic)
            {
                if (body.IgnoreGravity)
                {
                    linearVelocity += body.Force * data.FrameTime * body.InvMass;
                }
                else
                {
                    linearVelocity += (gravity + body.Force * body.InvMass) * data.FrameTime;
                }

                angularVelocity += body.InvI * body.Torque * data.FrameTime;

                linearVelocity *= Math.Clamp(1.0f - data.FrameTime * body.LinearDamping, 0.0f, 1.0f);
                angularVelocity *= Math.Clamp(1.0f - data.FrameTime * body.AngularDamping, 0.0f, 1.0f);
            }

            positions[i] = position;
            angles[i] = angle;
            linearVelocities[i + offset] = linearVelocity;
            angularVelocities[i + offset] = angularVelocity;
        }
    }

    // Hyperion (A3): job for the big-island body-prep pass. Range-based (like FinalisePositionsJob) so each
    // batch calls PrepareBodies once over its whole slice instead of dispatching per-body.
    // Implements IParallelRobustJob (not just IParallelRangeRobustJob) since IParallelManager.ProcessNow only
    // dispatches the former; Execute(int) is a one-index-range fallback that ExecuteRange (the actual batched
    // path used by ProcessNow) overrides, mirroring BroadphaseContactJob / ManifoldsJob.
    private record struct PrepareBodiesJob : IParallelRobustJob
    {
        public int BatchSize => 32;

        public SharedPhysicsSystem System;
        public SolverData Data;
        public int Offset;
        public Vector2 Gravity;
        public List<Entity<PhysicsComponent, TransformComponent>> Bodies;
        public Vector2[] Positions;
        public float[] Angles;
        public Vector2[] LinearVelocities;
        public float[] AngularVelocities;

        public void Execute(int index)
        {
            ExecuteRange(index, index + 1);
        }

        public void ExecuteRange(int startIndex, int endIndex)
        {
            System.PrepareBodies(startIndex, endIndex, Offset, Gravity, in Data, Bodies, Positions, Angles, LinearVelocities, AngularVelocities);
        }
    }

    /// <summary>
    /// Clamps linear/angular velocity to the max-per-tick limits and integrates positions/angles for bodies
    /// [start, end) of the island. Each index writes only its own slot, so this is safe to call over disjoint
    /// ranges concurrently (see IntegratePositionsJob).
    /// </summary>
    private void IntegratePositions(
        int start,
        int end,
        int offset,
        float frameTime,
        float maxVel,
        float maxVelSq,
        float maxAngVel,
        float maxAngVelSq,
        Vector2[] positions,
        float[] angles,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        for (var i = start; i < end; i++)
        {
            var linearVelocity = linearVelocities[offset + i];
            var angularVelocity = angularVelocities[offset + i];

            var velSqr = linearVelocity.LengthSquared();
            if (velSqr > maxVelSq)
            {
                linearVelocity *= maxVel / MathF.Sqrt(velSqr);
                linearVelocities[offset + i] = linearVelocity;
            }

            if (angularVelocity * angularVelocity > maxAngVelSq)
            {
                angularVelocity *= maxAngVel / MathF.Abs(angularVelocity);
                angularVelocities[offset + i] = angularVelocity;
            }

            // Integrate
            positions[i] += linearVelocity * frameTime;
            angles[i] += angularVelocity * frameTime;
        }
    }

    // Hyperion (A3): job for the big-island position-integrate pass. Range-based like PrepareBodiesJob /
    // FinalisePositionsJob.
    // Implements IParallelRobustJob (see PrepareBodiesJob comment above) for the same reason: ProcessNow
    // requires it, ExecuteRange stays the real batched worker.
    private record struct IntegratePositionsJob : IParallelRobustJob
    {
        public int BatchSize => 32;

        public SharedPhysicsSystem System;
        public int Offset;
        public float FrameTime;
        public float MaxVel;
        public float MaxVelSq;
        public float MaxAngVel;
        public float MaxAngVelSq;
        public Vector2[] Positions;
        public float[] Angles;
        public Vector2[] LinearVelocities;
        public float[] AngularVelocities;

        public void Execute(int index)
        {
            ExecuteRange(index, index + 1);
        }

        public void ExecuteRange(int startIndex, int endIndex)
        {
            System.IntegratePositions(startIndex, endIndex, Offset, FrameTime, MaxVel, MaxVelSq, MaxAngVel, MaxAngVelSq, Positions, Angles, LinearVelocities, AngularVelocities);
        }
    }

    private void FinalisePositions(int start, int end, int offset, List<Entity<PhysicsComponent, TransformComponent>> bodies, Vector2[] positions, float[] angles, Vector2[] solvedPositions, float[] solvedAngles)
    {
        for (var i = start; i < end; i++)
        {
            var ent = bodies[i];
            var body = ent.Comp1;

            if (body.BodyType == BodyType.Static)
                continue;

            var xform = ent.Comp2;
            // TODO: Root cause out why this TryComp is necessary.
            // See https://github.com/space-wizards/RobustToolbox/pull/6135
            if (!TryComp(xform.ParentUid, out TransformComponent? transform))
                continue;

            var (_, parentRot, parentInvMatrix) = _transform.GetWorldPositionRotationInvMatrix(transform);
            var worldRot = (float) (parentRot + xform._localRotation);

            var angle = angles[i];

            var q = new Quaternion2D(angle);
            var adjustedPosition = positions[i] - Physics.Transform.Mul(q, body.LocalCenter);

            var solvedPosition = Vector2.Transform(adjustedPosition, parentInvMatrix);
            solvedPositions[offset + i] = solvedPosition - xform.LocalPosition;
            solvedAngles[offset + i] = angle - worldRot;
        }
    }

    // Hyperion (A7): job for the big-island position finalise pass. Implements IParallelRobustJob because
    // IParallelManager.ProcessNow only accepts that interface (not the range-only IParallelRangeRobustJob);
    // ExecuteRange is overridden so each batch still calls FinalisePositions once over its whole [start, end)
    // slice, matching the old FinaliseBodies=32 chunking exactly. Each index range writes only to its own
    // slots in solvedPositions/solvedAngles, so batches never race.
    private record struct FinalisePositionsJob : IParallelRobustJob
    {
        public int BatchSize => 32;

        public SharedPhysicsSystem System;
        public int Offset;
        public List<Entity<PhysicsComponent, TransformComponent>> Bodies;
        public Vector2[] Positions;
        public float[] Angles;
        public Vector2[] SolvedPositions;
        public float[] SolvedAngles;

        public void Execute(int index)
        {
            ExecuteRange(index, index + 1);
        }

        public void ExecuteRange(int startIndex, int endIndex)
        {
            System.FinalisePositions(startIndex, endIndex, Offset, Bodies, Positions, Angles, SolvedPositions, SolvedAngles);
        }
    }

    /// <summary>
    /// Updates the positions, rotations, and velocities of all of the solved bodies.
    /// Run sequentially to avoid threading issues.
    /// </summary>
    private void UpdateBodies(
        in IslandData island,
        Vector2[] positions,
        float[] angles,
        Vector2[] linearVelocities,
        float[] angularVelocities)
    {
        foreach (var (joint, error) in island.BrokenJoints)
        {
            var ev = new JointBreakEvent(joint, MathF.Sqrt(error));
            RaiseLocalEvent(joint.BodyAUid, ref ev);
            RaiseLocalEvent(joint.BodyBUid, ref ev);
            RaiseLocalEvent(ref ev);
            joint.Dirty();
        }

        var offset = island.Offset;

        for (var i = 0; i < island.Bodies.Count; i++)
        {
            var bodyEnt = island.Bodies[i];
            var body = bodyEnt.Comp1;

            // So technically we don't /need/ to skip static bodies here but it saves us having to check for deferred updates so we'll do it anyway.
            // Plus calcing worldpos can be costly so we skip that too which is nice.
            if (body.BodyType == BodyType.Static) continue;

            var uid = bodyEnt.Owner;
            var position = positions[offset + i];
            var angle = angles[offset + i];
            var xform = bodyEnt.Comp2;

            var linVelocity = linearVelocities[offset + i];
            var physicsDirtied = false;

            if (!float.IsNaN(linVelocity.X) && !float.IsNaN(linVelocity.Y))
            {
                physicsDirtied |= SetLinearVelocity(uid, linVelocity, false, body: body);
            }

            var angVelocity = angularVelocities[offset + i];

            if (!float.IsNaN(angVelocity))
            {
                physicsDirtied |= SetAngularVelocity(uid, angVelocity, false, body: body);
            }

            // Temporary NaN guards until PVS is fixed.
            // May reparent object and change body's velocity.
            if (!float.IsNaN(position.X) && !float.IsNaN(position.Y))
            {
                _transform.SetLocalPositionRotation(uid,
                    xform.LocalPosition + position,
                    xform.LocalRotation + angle,
                    xform);
            }

            if (physicsDirtied)
            {
                Dirty(uid, body);
            }
        }
    }

    private void SleepBodies(in IslandData island, bool[] sleepStatus)
    {
        var offset = island.Offset;

        for (var i = 0; i < island.Bodies.Count; i++)
        {
            var sleep = sleepStatus[offset + i];

            if (!sleep)
                continue;

            var body = island.Bodies[i];

            SetAwake(body.Owner, body, false);
        }
    }
}
