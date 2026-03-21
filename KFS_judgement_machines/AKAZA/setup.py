from setuptools import setup
import os
from glob import glob

package_name = "kfs_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={"": "."},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "resource"), glob("resource/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@test.com",
    description="KFS package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # matcher_node.py 内の main 関数を呼び出す
            "matcher_node = kfs_pkg.matcher_node:main",
            "udp_node = kfs_pkg.node:main",
        ],
    },
)
