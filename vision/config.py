"""Loading config.yaml."""

from pathlib import Path

import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config.yaml"


def load_config(path=None):
    """Load the YAML config. Relative file paths inside it are resolved against the config's folder."""
    path = Path(path or DEFAULT_CONFIG).resolve()
    with open(path) as f:
        cfg = yaml.safe_load(f)
    cfg["_dir"] = str(path.parent)
    return cfg


def resolve_path(cfg, p):
    """Turn a path from the config into an absolute path (relative to the config file)."""
    if p is None:
        return None
    p = Path(p)
    return p if p.is_absolute() else Path(cfg.get("_dir", ".")) / p
