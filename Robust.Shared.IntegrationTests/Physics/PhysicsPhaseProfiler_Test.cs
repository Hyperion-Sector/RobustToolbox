// RobustToolbox/Robust.Shared.IntegrationTests/Physics/PhysicsPhaseProfiler_Test.cs
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Shared.Physics;

namespace Robust.Shared.IntegrationTests.Physics;

[TestFixture]
public sealed class PhysicsPhaseProfiler_Test
{
    [Test]
    public void AccumulatesAndSnapshotsPerPhase()
    {
        var p = new PhysicsPhaseProfiler();
        p.BeginTick();
        p.AddTicks(PhysicsPhase.ConstraintSolve, 1000);
        p.AddTicks(PhysicsPhase.ConstraintSolve, 500);
        p.AddTicks(PhysicsPhase.MoveChurn, 250);
        p.EndTick();

        var freq = (double) System.Diagnostics.Stopwatch.Frequency;
        Assert.That(p.LastTickSeconds[(int) PhysicsPhase.ConstraintSolve], Is.EqualTo(1500 / freq).Within(1e-12));
        Assert.That(p.LastTickSeconds[(int) PhysicsPhase.MoveChurn], Is.EqualTo(250 / freq).Within(1e-12));
        Assert.That(p.LastTickSeconds[(int) PhysicsPhase.Prestep], Is.EqualTo(0));
    }

    [Test]
    public void AddTicksIsThreadSafe()
    {
        var p = new PhysicsPhaseProfiler();
        p.BeginTick();
        Parallel.For(0, 10_000, _ => p.AddTicks(PhysicsPhase.ConstraintSolve, 1));
        p.EndTick();
        Assert.That(p.LastTickSeconds[(int) PhysicsPhase.ConstraintSolve],
            Is.EqualTo(10_000 / (double) System.Diagnostics.Stopwatch.Frequency).Within(1e-12));
    }
}
