using System;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Systems;
using Robust.UnitTesting.Server;

namespace Robust.Shared.IntegrationTests.Physics;

[TestFixture]
public sealed class PhysicsPhaseAttribution_Test
{
    [Test]
    public void DenseStackPopulatesConstraintSolveMoveChurnAndNarrowphase()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entMan = sim.Resolve<IEntityManager>();
        var physics = entMan.System<SharedPhysicsSystem>();
        var fixtures = entMan.System<FixtureSystem>();
        entMan.System<SharedMapSystem>().CreateMap(out var mapId);
        physics.SetGravity(new Vector2(0f, -9.8f));

        // Ground edge so the falling boxes collide and pile rather than fall forever.
        var ground = entMan.SpawnEntity(null, new MapCoordinates(0, 0, mapId));
        var groundBody = entMan.AddComponent<PhysicsComponent>(ground);
        fixtures.CreateFixture(ground, "fix1", new Fixture(new EdgeShape(new Vector2(-50, 0), new Vector2(50, 0)), 2, 2, true), body: groundBody);
        physics.WakeBody(ground, body: groundBody);

        // A column of dynamic boxes that fall, collide, and settle => contacts + movement.
        var box = new PolygonShape();
        box.SetAsBox(0.5f, 0.5f);
        for (var i = 0; i < 30; i++)
        {
            var b = entMan.SpawnEntity(null, new MapCoordinates(new Vector2(0, 1 + i * 1.1f), mapId));
            var body = entMan.AddComponent<PhysicsComponent>(b);
            physics.SetBodyType(b, BodyType.Dynamic, body: body);
            fixtures.CreateFixture(b, "fix1", new Fixture(box, 2, 2, true, 5f), body: body);
            physics.WakeBody(b, body: body);
        }

        // Track the peak per-bucket time across the run; the stack may sleep before the last tick.
        double maxSolve = 0, maxChurn = 0, maxNarrow = 0;
        for (var t = 0; t < 60; t++)
        {
            entMan.TickUpdate(1f / 60f, false);
            var s = physics.PhaseProfiler.LastTickSeconds;
            maxSolve = Math.Max(maxSolve, s[(int) PhysicsPhase.ConstraintSolve]);
            maxChurn = Math.Max(maxChurn, s[(int) PhysicsPhase.MoveChurn]);
            maxNarrow = Math.Max(maxNarrow, s[(int) PhysicsPhase.Narrowphase]);
        }

        Assert.That(maxSolve, Is.GreaterThan(0), "constraint-solve should register while the stack collides/settles");
        Assert.That(maxChurn, Is.GreaterThan(0), "move-churn should register while boxes move");
        Assert.That(maxNarrow, Is.GreaterThan(0), "narrowphase should register with active contacts");
    }
}
