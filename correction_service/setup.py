"""Install correction executables; configuration stays in the source project."""

from setuptools import find_packages, setup

# ROS package name used by the installation layout.
PACKAGE_NAME = "correction_service"


setup(
    name=PACKAGE_NAME,
    version="2.0.0",
    packages=find_packages(exclude=("test", "tests")),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xld",
    maintainer_email="xld@todo.todo",
    description="Independent multi-Tag AprilTag-to-Odin window calibration service.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "correction_node = correction_service.node:main",
            "uvc_camera_node = correction_service.uvc_camera_node:main",
            "correction_panel = correction_service.correction_panel:main",
        ],
    },
)
