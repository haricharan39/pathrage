import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ugv_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hari Charan",
    maintainer_email="you@example.com",
    description="Perception (Tier 1 + Tier 2) and visual localization glue for the UGV visual nav project.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "depth_obstacle_node = ugv_vision.depth_obstacle_node:main",
            "segmentation_node = ugv_vision.segmentation_node:main",
            "costmap_fusion_node = ugv_vision.costmap_fusion_node:main",
            "localization_node = ugv_vision.localization_node:main",
        ],
    },
)
