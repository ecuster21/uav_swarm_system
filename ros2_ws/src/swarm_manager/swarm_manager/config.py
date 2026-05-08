from pathlib import Path
from typing import Any

import yaml


def normalize_namespace(namespace: str) -> str:
    return namespace.strip().strip("/")


def load_yaml(path: str) -> dict[str, Any]:
    config_path = Path(path).expanduser()
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML file must contain a mapping: {config_path}")
    return data
