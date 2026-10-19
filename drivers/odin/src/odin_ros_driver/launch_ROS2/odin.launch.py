"""Short native ROS 2 launch alias for the standalone Odin driver."""

from pathlib import Path
import runpy


def generate_launch_description():
    """Reuse the full launch entry and its configuration arguments."""
    launch = Path(__file__).with_name("odin1_ros2.launch.py")
    return runpy.run_path(str(launch))["generate_launch_description"]()
