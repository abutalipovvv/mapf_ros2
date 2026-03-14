from __future__ import annotations

import argparse
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - fallback for source-tree execution
    get_package_share_directory = None

from .modes import discrete_cbs, manual_modes
from .result_logger import append_result
from .scenario_loader import load_scenarios


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_scenarios_file() -> str:
    if get_package_share_directory is not None:
        return str(
            Path(get_package_share_directory("fleet_experiments"))
            / "config"
            / "thesis_scenarios.yaml"
        )
    return str(_repo_root() / "config" / "thesis_scenarios.yaml")


def _default_landmarks_file() -> str:
    if get_package_share_directory is not None:
        return str(
            Path(get_package_share_directory("hybrid_fleet_manager"))
            / "config"
            / "landmarks.yaml"
        )
    return str(_repo_root().parents[0] / "hybrid_fleet_manager" / "config" / "landmarks.yaml")


def _default_results_root() -> Path:
    workspace_candidate = Path.cwd() / "fleet_experiments" / "results"
    if workspace_candidate.parent.exists():
        return workspace_candidate
    return Path.cwd() / "output" / "fleet_experiments"


def main() -> None:
    parser = argparse.ArgumentParser(description="Run thesis experiment scenarios.")
    parser.add_argument("--mode", required=True, choices=["discrete_cbs", "nav2_only", "hybrid"])
    parser.add_argument("--scenario", required=True, help="Scenario name from thesis_scenarios.yaml")
    parser.add_argument("--runs", type=int, default=1, help="Number of repetitions")
    parser.add_argument("--scenario-file", default=_default_scenarios_file())
    parser.add_argument("--landmarks-file", default=_default_landmarks_file())
    parser.add_argument("--output-dir", default=str(_default_results_root()))
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="For discrete_cbs, launch external visualize.py after planning.",
    )
    parser.add_argument("--status", default="completed", help="Manual mode result status")
    parser.add_argument("--execution-time-sec", type=float, default=0.0)
    parser.add_argument("--success-rate", type=float, default=1.0)
    parser.add_argument("--blocked-tasks", type=int, default=0)
    parser.add_argument("--requeue-count", type=int, default=0)
    parser.add_argument("--notes", default="")
    args = parser.parse_args()

    scenarios = load_scenarios(args.scenario_file)
    if args.scenario not in scenarios:
        available = ", ".join(sorted(scenarios.keys()))
        raise ValueError(f"Unknown scenario '{args.scenario}'. Available: {available}")

    scenario = scenarios[args.scenario]
    results_root = Path(args.output_dir)

    for run_index in range(1, max(1, args.runs) + 1):
        if args.mode == "discrete_cbs":
            result = discrete_cbs.run(
                mode=args.mode,
                scenario=scenario,
                landmarks_file=args.landmarks_file,
                run_index=run_index,
                results_root=str(results_root),
                visualize=args.visualize,
            )
        else:
            result = manual_modes.run(
                mode=args.mode,
                scenario=scenario,
                landmarks_file=args.landmarks_file,
                run_index=run_index,
                result_status=args.status,
                execution_time_sec=args.execution_time_sec,
                success_rate=args.success_rate,
                blocked_tasks=args.blocked_tasks,
                requeue_count=args.requeue_count,
                notes=args.notes,
            )

        artifact_paths = append_result(results_root, result)
        print(
            f"[{result.mode}] scenario={result.scenario} run={run_index} "
            f"status={result.status} csv={artifact_paths['csv']} json={artifact_paths['json']}"
        )
        commands = result.artifacts.get("commands")
        if isinstance(commands, list) and commands:
            print("Suggested commands:")
            for command in commands:
                print(f"  - {command}")
        visualize_command = result.artifacts.get("visualize_command")
        if isinstance(visualize_command, str) and visualize_command:
            print(f"Visualization command: {visualize_command}")


if __name__ == "__main__":
    main()
