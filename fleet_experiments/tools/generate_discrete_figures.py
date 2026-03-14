from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Iterable

import matplotlib
matplotlib.use("Agg")
from matplotlib import animation
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT / "fleet_experiments") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "fleet_experiments"))
if str(REPO_ROOT / "hybrid_fleet_manager") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "hybrid_fleet_manager"))

from fleet_experiments.modes import discrete_cbs
from fleet_experiments.scenario_loader import load_scenarios


def _default_scenario_file() -> Path:
    return REPO_ROOT / "fleet_experiments" / "config" / "dissertation_figure_scenarios.yaml"


def _default_landmarks_file() -> Path:
    return REPO_ROOT / "hybrid_fleet_manager" / "config" / "landmarks.yaml"


def _default_output_dir() -> Path:
    return REPO_ROOT / "output" / "discrete_cbs_figures"


def _load_visualize_module():
    import importlib.util

    path = REPO_ROOT / "multi_agent_path_planning" / "centralized" / "visualize.py"
    spec = importlib.util.spec_from_file_location("mapf_visualize", path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _save_gif_and_frames(input_yaml: Path, output_yaml: Path, out_dir: Path, scenario_name: str) -> None:
    visualize = _load_visualize_module()

    with input_yaml.open("r", encoding="utf-8") as f:
        map_payload = yaml.safe_load(f)
    with output_yaml.open("r", encoding="utf-8") as f:
        schedule_payload = yaml.safe_load(f)

    anim = visualize.Animation(map_payload, schedule_payload)
    gif_path = out_dir / f"{scenario_name}.gif"
    anim.anim.save(
        str(gif_path),
        writer=animation.PillowWriter(fps=4),
        dpi=180,
    )

    total_frames = max(1, int(anim.T + 1) * 10)
    frame_indices = [0, max(0, total_frames // 2), max(0, total_frames - 1)]
    anim.init_func()
    for idx, frame_idx in enumerate(frame_indices, 1):
        anim.animate_func(frame_idx)
        frame_path = out_dir / f"{scenario_name}_frame_{idx}.png"
        anim.fig.savefig(str(frame_path), dpi=200, bbox_inches="tight")


def _iter_scenarios(requested: list[str], available: dict) -> Iterable[str]:
    if requested:
        for name in requested:
            if name not in available:
                available_names = ", ".join(sorted(available.keys()))
                raise ValueError(f"Unknown scenario '{name}'. Available: {available_names}")
            yield name
        return
    for name in available.keys():
        yield name


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate discrete CBS GIFs and frame images.")
    parser.add_argument("--scenario-file", default=str(_default_scenario_file()))
    parser.add_argument("--landmarks-file", default=str(_default_landmarks_file()))
    parser.add_argument("--output-dir", default=str(_default_output_dir()))
    parser.add_argument("--scenario", action="append", help="Scenario name to generate. Can be repeated.")
    args = parser.parse_args()

    scenarios = load_scenarios(args.scenario_file)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    results_root = REPO_ROOT / "fleet_experiments" / "results"

    for scenario_name in _iter_scenarios(args.scenario or [], scenarios):
        scenario = scenarios[scenario_name]
        result = discrete_cbs.run(
            mode="discrete_cbs",
            scenario=scenario,
            landmarks_file=args.landmarks_file,
            run_index=1,
            results_root=str(results_root),
            visualize=False,
        )
        if result.status != "completed":
            print(f"[skip] {scenario_name}: no discrete CBS solution")
            continue

        input_yaml = Path(result.artifacts["input_yaml"])
        output_yaml = Path(result.artifacts["output_yaml"])
        _save_gif_and_frames(input_yaml, output_yaml, out_dir, scenario_name)
        print(f"[ok] {scenario_name} -> {out_dir / (scenario_name + '.gif')}")


if __name__ == "__main__":
    main()
