using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace Robust.Shared.Physics;

/// <summary>
/// Buckets the physics step into the cost paths an optimization would target. See the physics test suite
/// design: buckets are defined by target, not call order, so each candidate maps to exactly one.
/// </summary>
public enum PhysicsPhase
{
    Prestep,
    ContactFind,
    GridQuery,
    Narrowphase,
    IslandBuild,
    ConstraintPrep,
    ConstraintSolve,
    Integrate,
    MoveChurn,
    WriteBack,
}

/// <summary>
/// Per-tick accumulator of physics phase timings, in Stopwatch ticks. Slots written from parallel code
/// (ConstraintSolve, MoveChurn) accumulate atomically; main-thread seams use the same path (uncontended).
/// </summary>
public sealed class PhysicsPhaseProfiler
{
    private const int Count = (int) PhysicsPhase.WriteBack + 1;

    private readonly long[] _ticks = new long[Count];
    private readonly double[] _lastSeconds = new double[Count];

    public IReadOnlyList<double> LastTickSeconds => _lastSeconds;

    public void BeginTick()
    {
        Array.Clear(_ticks, 0, Count);
    }

    public void AddTicks(PhysicsPhase phase, long stopwatchTicks)
    {
        Interlocked.Add(ref _ticks[(int) phase], stopwatchTicks);
    }

    public Scope Measure(PhysicsPhase phase) => new(this, phase);

    public void EndTick()
    {
        var freq = (double) Stopwatch.Frequency;
        for (var i = 0; i < Count; i++)
            _lastSeconds[i] = _ticks[i] / freq;
    }

    public readonly struct Scope : IDisposable
    {
        private readonly PhysicsPhaseProfiler _profiler;
        private readonly PhysicsPhase _phase;
        private readonly long _start;

        public Scope(PhysicsPhaseProfiler profiler, PhysicsPhase phase)
        {
            _profiler = profiler;
            _phase = phase;
            _start = Stopwatch.GetTimestamp();
        }

        public void Dispose()
        {
            _profiler.AddTicks(_phase, Stopwatch.GetTimestamp() - _start);
        }
    }
}
