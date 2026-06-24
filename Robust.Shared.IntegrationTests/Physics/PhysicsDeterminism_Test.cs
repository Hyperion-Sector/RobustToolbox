using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Shared;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Systems;

namespace Robust.UnitTesting.Shared.Physics;

/// <summary>
/// Golden-hash determinism gate for the physics solver. Runs a fixed box-stack scenario
/// SINGLE-THREADED (thread.parallel_count = 1, since the current parallel solve is racy by design)
/// and hashes the settled body states with FNV-1a over the raw float bits.
///
/// Any BEHAVIOR-PRESERVING change to the physics step (the Phase 0 alloc reuse, the Phase 2
/// persistent-island rewrite, the broadphase work) MUST keep <see cref="GoldenHash"/> identical.
/// A changed hash means the simulation result changed: review whether that was intended, and only
/// then record a new golden. Note: the hash is over float bits, so the golden is machine/JIT
/// specific; treat it as a same-machine before/after regression gate, not a cross-platform constant.
/// </summary>
[TestFixture]
internal sealed class PhysicsDeterminism_Test : RobustIntegrationTest
{
    private const int Ticks = 150;

    // Recorded on fork/master (Federation 277.2.0 + Phase 0 solver-alloc reuse), single-threaded.
    private const ulong GoldenHash = 1043003047946187357UL;

    [Test]
    public async Task DeterministicStack()
    {
        var server = StartServer(new ServerIntegrationOptions { Pool = false });
        await server.WaitIdleAsync();

        await server.WaitPost(() => server.CfgMan.SetCVar(CVars.ThreadParallelCount, 1));

        var entMan = server.EntMan;
        var mapSys = server.System<SharedMapSystem>();
        var physics = server.System<SharedPhysicsSystem>();
        var fixtures = server.System<FixtureSystem>();

        await server.WaitPost(() =>
        {
            mapSys.CreateMap(out var mapId);
            physics.SetGravity(new Vector2(0f, -9.8f));

            // Static ground.
            var groundUid = entMan.Spawn(null, new MapCoordinates(0f, 0f, mapId));
            var ground = entMan.AddComponent<PhysicsComponent>(groundUid);
            var edge = new EdgeShape(new Vector2(-20f, 0f), new Vector2(20f, 0f));
            fixtures.CreateFixture(groundUid, "fix1", new Fixture(edge, 2, 2, true), body: ground);
            physics.WakeBody(groundUid, body: ground);

            // A handful of dynamic box stacks: contacts + islands + settling exercise the solver.
            var box = new PolygonShape();
            box.SetAsBox(0.5f, 0.5f);

            for (var col = 0; col < 4; col++)
            {
                for (var row = 0; row < 6; row++)
                {
                    var uid = entMan.Spawn(null, new MapCoordinates(col * 3f - 4.5f, 1f + row * 1.01f, mapId));
                    var body = entMan.AddComponent<PhysicsComponent>(uid);
                    physics.SetBodyType(uid, BodyType.Dynamic, body: body);
                    fixtures.CreateFixture(uid, "fix1", new Fixture(box, 2, 2, true, 5f), body: body);
                    physics.WakeBody(uid, body: body);
                }
            }
        });

        await server.WaitRunTicks(Ticks);

        var hash = 0UL;
        await server.WaitPost(() => hash = HashPhysicsState(entMan));

        TestContext.WriteLine($"Physics determinism state-hash = {hash}UL");
        Assert.That(hash, Is.EqualTo(GoldenHash),
            "Physics simulation result changed. If intentional, record the printed hash as the new golden.");
    }

    private static ulong HashPhysicsState(IEntityManager entMan)
    {
        var states = new List<(EntityUid Uid, Vector2 Pos, float Rot, Vector2 Lin, float Ang)>();
        var query = entMan.EntityQueryEnumerator<PhysicsComponent, TransformComponent>();
        while (query.MoveNext(out var uid, out var phys, out var xform))
        {
            states.Add((uid, xform.LocalPosition, (float) xform.LocalRotation.Theta, phys.LinearVelocity, phys.AngularVelocity));
        }

        states.Sort(static (a, b) => a.Uid.CompareTo(b.Uid));

        var h = 14695981039346656037UL; // FNV-1a 64-bit offset basis.
        foreach (var s in states)
        {
            h = Fold(h, s.Pos.X);
            h = Fold(h, s.Pos.Y);
            h = Fold(h, s.Rot);
            h = Fold(h, s.Lin.X);
            h = Fold(h, s.Lin.Y);
            h = Fold(h, s.Ang);
        }

        return h;
    }

    private static ulong Fold(ulong h, float value)
    {
        var bits = (uint) BitConverter.SingleToInt32Bits(value);
        h ^= bits;
        h *= 1099511628211UL; // FNV-1a 64-bit prime.
        return h;
    }
}
