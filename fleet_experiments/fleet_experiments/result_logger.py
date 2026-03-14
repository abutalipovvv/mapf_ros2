from __future__ import annotations

import csv
import json
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List

from .models import ExperimentResult


RAW_CSV_FIELDS: List[str] = [
    "timestamp_utc",
    "mode",
    "scenario",
    "run_index",
    "status",
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
    "notes",
]


def ensure_results_dirs(results_root: Path) -> Dict[str, Path]:
    raw_dir = results_root / "raw"
    summary_dir = results_root / "summary"
    plots_dir = results_root / "plots"
    raw_dir.mkdir(parents=True, exist_ok=True)
    summary_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)
    return {
        "root": results_root,
        "raw": raw_dir,
        "summary": summary_dir,
        "plots": plots_dir,
    }


def append_result(results_root: Path, result: ExperimentResult) -> Dict[str, Path]:
    dirs = ensure_results_dirs(results_root)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    json_path = dirs["raw"] / f"{timestamp}_{result.mode}_{result.scenario}_run{result.run_index:02d}.json"

    payload = asdict(result)
    payload["timestamp_utc"] = timestamp
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=True)

    csv_path = dirs["raw"] / "experiments.csv"
    row = {field: "" for field in RAW_CSV_FIELDS}
    row["timestamp_utc"] = timestamp
    row["mode"] = result.mode
    row["scenario"] = result.scenario
    row["run_index"] = result.run_index
    row["status"] = result.status

    for key, value in result.metrics.items():
        if key in row:
            row[key] = value

    with csv_path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=RAW_CSV_FIELDS)
        if csv_path.stat().st_size == 0:
            writer.writeheader()
        writer.writerow(row)

    return {"json": json_path, "csv": csv_path}


def load_raw_rows(csv_path: Path) -> List[Dict[str, str]]:
    if not csv_path.exists():
        return []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def write_summary(summary_path: Path, rows: Iterable[Dict[str, object]]) -> None:
    rows = list(rows)
    if not rows:
        return

    fieldnames = list(rows[0].keys())
    with summary_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
