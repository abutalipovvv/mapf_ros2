from __future__ import annotations

from pathlib import Path
from typing import Dict, Any

import yaml


def load_landmarks(yaml_path: str) -> Dict[str, Dict[str, Any]]:
    path = Path(yaml_path)
    if not path.exists():
        raise FileNotFoundError(f"Landmarks file not found: {yaml_path}")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("Landmarks YAML must contain a top-level dictionary")

    for name, value in data.items():
        if not isinstance(value, dict):
            raise ValueError(f"Landmark '{name}' must be a dictionary")

        if "x" not in value or "y" not in value:
            raise ValueError(f"Landmark '{name}' must contain x and y")

        if "yaw" not in value:
            value["yaw"] = 0.0

    return data