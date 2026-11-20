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
    _overlay_head_poses(cfg)
    return cfg


def _overlay_head_poses(cfg):
    """An ACTIVE calib/robot<id>_head_pose.yaml (written by the dashboard calibration checklist) is the
    source of truth for that robot's head geometry and overrides the copy embedded in config.yaml."""
    for robot in cfg.get("robots") or []:
        f = Path(cfg["_dir"]) / "calib" / f"robot{robot.get('id')}_head_pose.yaml"
        if not f.exists():
            continue
        with open(f) as fh:
            data = yaml.safe_load(fh) or {}
        if str(data.get("status", "")).startswith("active") and data.get("head_pose"):
            robot["head_pose"] = data["head_pose"]


def resolve_path(cfg, p):
    """Turn a path from the config into an absolute path (relative to the config file)."""
    if p is None:
        return None
    p = Path(p)
    return p if p.is_absolute() else Path(cfg.get("_dir", ".")) / p
