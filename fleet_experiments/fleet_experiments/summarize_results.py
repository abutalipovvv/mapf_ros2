from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
from typing import Dict, List

from .result_logger import load_raw_rows, write_summary


NUMERIC_FIELDS = [
    "robot_count",
    "makespan",
    "sum_of_costs",
    "planning_time_sec",
    "execution_time_sec",
    "success_rate",
    "completed_tasks",
    "failed_tasks",
    "blocked_tasks",
    "requeue_count",
    "conflicts_resolved",
    "high_level_nodes",
]


def _default_results_root() -> Path:
    workspace_candidate = Path.cwd() / "fleet_experiments" / "results"
    if workspace_candidate.parent.exists():
        return workspace_candidate
    return Path.cwd() / "output" / "fleet_experiments"


def _mean(values: List[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / len(values)


def main() -> None:
    parser = argparse.ArgumentParser(description="Aggregate raw experiment logs into a summary CSV.")
    parser.add_argument("--results-dir", default=str(_default_results_root()))
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    raw_csv = results_dir / "raw" / "experiments.csv"
    rows = load_raw_rows(raw_csv)
    if not rows:
        print(f"No raw results found at {raw_csv}")
        return

    grouped: Dict[tuple[str, str], List[Dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[(row["mode"], row["scenario"])].append(row)

    summary_rows = []
    for (mode, scenario), group_rows in sorted(grouped.items()):
        summary_row: Dict[str, object] = {
            "mode": mode,
            "scenario": scenario,
            "runs": len(group_rows),
        }
        for field in NUMERIC_FIELDS:
            numeric_values: List[float] = []
            for row in group_rows:
                raw_value = row.get(field, "")
                if raw_value == "":
                    continue
                try:
                    numeric_values.append(float(raw_value))
                except ValueError:
                    continue
            summary_row[f"avg_{field}"] = round(_mean(numeric_values), 6)

        summary_row["statuses"] = ",".join(row["status"] for row in group_rows)
        summary_rows.append(summary_row)

    summary_path = results_dir / "summary" / "summary.csv"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    write_summary(summary_path, summary_rows)
    print(f"Summary written to {summary_path}")


if __name__ == "__main__":
    main()
