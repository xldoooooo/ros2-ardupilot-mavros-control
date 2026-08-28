"""ament_python 安装入口，机载节点不导入 Qt 调试面板。"""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "correction_service"

setup(
    name=PACKAGE_NAME,
    version="1.0.0",
    packages=find_packages(exclude=("test", "tests")),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xld",
    maintainer_email="xld@todo.todo",
    description="Independent AprilTag-to-Odin planar correction service.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "correction_node = correction_service.node:main",
            "correction_panel = correction_service.correction_panel:main",
        ],
    },
)
