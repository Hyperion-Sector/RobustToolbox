using BenchmarkDotNet.Attributes;
using Robust.Shared.Analyzers;
using Robust.UnitTesting.Server;

namespace Robust.Benchmarks.Physics;

[Virtual, MediumRunJob, MemoryDiagnoser]
public class PhysicsBenchmark
{
    // TODO: Rain
    // Large pyramid
    // Joint Grid
    // Spinner
    // Washer

    #region Many Pyramids

    private ISimulation _manyPyramidSim = default!;

    [GlobalSetup(Target = nameof(ManyPyramids))]
    public void PyramidSetup()
    {
        _manyPyramidSim = RobustServerSimulation.NewSimulation().InitializeInstance();
        PhysicsScenarios.BuildManyPyramids(_manyPyramidSim);
    }

    [Benchmark]
    public void ManyPyramids() => PhysicsScenarios.Step(_manyPyramidSim, PhysicsScenarios.DefaultTicks);

    #endregion

    #region Smash

    private ISimulation _smashSim = default!;

    [GlobalSetup(Target = nameof(Smash))]
    public void SmashSetup()
    {
        _smashSim = RobustServerSimulation.NewSimulation().InitializeInstance();
        PhysicsScenarios.BuildSmash(_smashSim);
    }

    [Benchmark]
    public void Smash() => PhysicsScenarios.Step(_smashSim, PhysicsScenarios.DefaultTicks);

    #endregion

    #region Tumbler

    private ISimulation _tumblerSim = default!;

    [GlobalSetup(Target = nameof(Tumbler))]
    public void TumblerSetup()
    {
        _tumblerSim = RobustServerSimulation.NewSimulation().InitializeInstance();
        PhysicsScenarios.BuildTumbler(_tumblerSim);
    }

    [Benchmark]
    public void Tumbler() => PhysicsScenarios.Step(_tumblerSim, PhysicsScenarios.DefaultTicks);

    #endregion
}
