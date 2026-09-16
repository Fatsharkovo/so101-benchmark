"""SO-101 benchmark. Simulator and UI dependencies are imported lazily."""

from importlib.util import find_spec

__version__ = "0.1.0"

# LeRobot discovers distributions with the lerobot_env_ prefix.
if find_spec("lerobot") is not None:
    from .integration import SO101EnvConfig as SO101EnvConfig
