"""ament_python 安装入口，递归安装相机配置档案且不导入 Qt 调试面板。"""

from pathlib import Path

from setuptools import find_packages, setup

PACKAGE_NAME = "correction_service"


def config_data_files() -> list[tuple[str, list[str]]]:
    """按目录递归收集活动配置与命名相机档案，便于后续增加新型号。"""
    entries: list[tuple[str, list[str]]] = []
    for directory in sorted(
        path for path in Path("config").rglob("*") if path.is_dir()
    ):
        files = sorted(str(path) for path in directory.iterdir() if path.is_file())
        if files:
            entries.append((f"share/{PACKAGE_NAME}/{directory}", files))
    root_files = sorted(str(path) for path in Path("config").iterdir() if path.is_file())
    if root_files:
        entries.insert(0, (f"share/{PACKAGE_NAME}/config", root_files))
    return entries


setup(
    name=PACKAGE_NAME,
    version="2.0.0",
    packages=find_packages(exclude=("test", "tests")),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
    ]
    + config_data_files(),
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
