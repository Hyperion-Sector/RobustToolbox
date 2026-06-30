using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Robust.Shared.GameObjects;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Systems;
using Robust.UnitTesting.Server;

namespace Robust.Benchmarks.Physics;

/// <summary>
/// Gate-1 decomposition harness: runs each <see cref="PhysicsScenarios"/> load and reports the
/// per-phase bucket profile read from <see cref="SharedPhysicsSystem.PhaseProfiler"/>. This is the
/// "which bucket moved" half of the synthetic gate (BenchmarkDotNet owns "is the total delta real").
/// Prints markdown to stdout only — never writes into the repo tree.
/// </summary>
public static class PhysicsBucketReport
{
    private const int Warmups = 1;
    private const int Passes = 3;

    public static void Run(string[] args)
    {
        var tickOverride = ParseTicks(args);
        var phases = Enum.GetValues<PhysicsPhase>();

        // scenario -> bucket -> mean seconds over a run
        var grid = new Dictionary<string, double[]>();

        foreach (var scenario in PhysicsScenarios.All)
        {
            var ticks = tickOverride ?? scenario.Ticks;
            var totals = new double[phases.Length];

            for (var pass = 0; pass < Warmups + Passes; pass++)
            {
                var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
                scenario.Build(sim);
                var entManager = sim.Resolve<IEntityManager>();
                var physics = entManager.System<SharedPhysicsSystem>();

                var passTotals = new double[phases.Length];
                for (var t = 0; t < ticks; t++)
                {
                    entManager.TickUpdate(PhysicsScenarios.FrameTime, false);
                    var s = physics.PhaseProfiler.LastTickSeconds;
                    for (var p = 0; p < phases.Length; p++)
                        passTotals[p] += s[p];
                }

                if (pass < Warmups)
                    continue; // discard JIT warmup

                for (var p = 0; p < phases.Length; p++)
                    totals[p] += passTotals[p];
            }

            for (var p = 0; p < phases.Length; p++)
                totals[p] /= Passes;

            grid[scenario.Name] = totals;
            Console.Error.WriteLine($"[bucket-report] {scenario.Name}: {ticks} ticks x {Passes} passes done");
        }

        Console.WriteLine(Format(grid, phases, tickOverride));
    }

    private static int? ParseTicks(string[] args)
    {
        for (var i = 0; i < args.Length - 1; i++)
        {
            if (args[i] == "--ticks" && int.TryParse(args[i + 1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var n) && n > 0)
                return n;
        }
        return null;
    }

    private static string Format(Dictionary<string, double[]> grid, PhysicsPhase[] phases, int? tickOverride)
    {
        var sb = new StringBuilder();
        sb.AppendLine($"# Physics bucket report (mean of {Passes} passes, ticks={(tickOverride?.ToString(CultureInfo.InvariantCulture) ?? "per-scenario")})");
        sb.AppendLine();

        // Absolute (ms per run).
        sb.Append("| Scenario |");
        foreach (var ph in phases) sb.Append($" {ph} (ms) |");
        sb.AppendLine();
        sb.Append("|---|");
        foreach (var _ in phases) sb.Append("---|");
        sb.AppendLine();
        foreach (var (name, totals) in grid)
        {
            sb.Append($"| {name} |");
            foreach (var ph in phases)
                sb.Append($" {(totals[(int) ph] * 1000.0).ToString("F2", CultureInfo.InvariantCulture)} |");
            sb.AppendLine();
        }

        sb.AppendLine();

        // Share (% of summed bucket time).
        sb.Append("| Scenario |");
        foreach (var ph in phases) sb.Append($" {ph} % |");
        sb.AppendLine();
        sb.Append("|---|");
        foreach (var _ in phases) sb.Append("---|");
        sb.AppendLine();
        foreach (var (name, totals) in grid)
        {
            var sum = 0.0;
            foreach (var v in totals) sum += v;
            sb.Append($"| {name} |");
            foreach (var ph in phases)
            {
                var pct = sum > 0 ? totals[(int) ph] / sum * 100.0 : 0.0;
                sb.Append($" {pct.ToString("F1", CultureInfo.InvariantCulture)} |");
            }
            sb.AppendLine();
        }

        return sb.ToString();
    }
}
