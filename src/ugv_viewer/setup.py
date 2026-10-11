import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ugv_viewer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*.*")),
        (os.path.join("share", package_name, "web", "vendor"), glob("web/vendor/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Hari Charan",
    maintainer_email="you@example.com",
    description="Browser-based 3D point-cloud map viewer for the VisionNav UGV.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "viewer_node = ugv_viewer.viewer_node:main",
            "viewer_demo = ugv_viewer.demo:main",
        ],
    },
)
