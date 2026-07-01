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

    #region Scatter

    private ISimulation _scatterSim = default!;

    [GlobalSetup(Target = nameof(Scatter))]
    public void ScatterSetup()
    {
        _scatterSim = RobustServerSimulation.NewSimulation().InitializeInstance();
        PhysicsScenarios.BuildScatter(_scatterSim);
    }

    [Benchmark]
    public void Scatter() => PhysicsScenarios.Step(_scatterSim, PhysicsScenarios.DefaultTicks);

    #endregion

    #region MultiMap

    private ISimulation _multiMapSim = default!;

    [GlobalSetup(Target = nameof(MultiMap))]
    public void MultiMapSetup()
    {
        _multiMapSim = RobustServerSimulation.NewSimulation().InitializeInstance();
        PhysicsScenarios.BuildMultiMap(_multiMapSim);
    }

    [Benchmark]
    public void MultiMap() => PhysicsScenarios.Step(_multiMapSim, PhysicsScenarios.DefaultTicks);

    #endregion

    #region BigPyramid

    private ISimulation _bigPyramidSim = default!;

    [GlobalSetup(Target = nameof(BigPyramid))]
    public void BigPyramidSetup()
    {
        _bigPyramidSim = RobustServerSimulation.NewSimulation().InitializeInstance();
        PhysicsScenarios.BuildBigPyramid(_bigPyramidSim);
    }

    [Benchmark]
    public void BigPyramid() => PhysicsScenarios.Step(_bigPyramidSim, PhysicsScenarios.BigPyramidTicks);

    #endregion
}
