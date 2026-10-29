"""Install the independent optional FPGA planner without changing the main ROS workspace."""
from glob import glob
from setuptools import find_packages, setup

# Ament package identity, retained from the original user-provided planner.
package_name = "fpga_uav_planner_ros2"

setup(
    name=package_name,
    version="27.3.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=False,
    maintainer="FPGA-UAV ROS2 adaptation team",
    maintainer_email="maintainer@example.invalid",
    description="ROS 2 Jazzy adapter for the UAV FPGA planner with native ABI 3301",
    license="GPLv3",
    entry_points={
        "console_scripts": [
            "fpga_flight_backend = fpga_uav_planner_ros2.flight_backend_node:main",
            "fpga_kino_planner = fpga_uav_planner_ros2.fpga_kino_planner_node:main",
            "fpga_astar_compare = fpga_uav_planner_ros2.fpga_astar_compare:main",
            "publish_demo_goal = fpga_uav_planner_ros2.publish_demo_goal:main",
        ],
    },
)
