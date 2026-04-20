from setuptools import find_packages, setup


package_name = "kfs_cube_fusion"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/kfs_cube_fusion.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Fusion node for KFS recognition, cube gating, and depth-based center distance measurement",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "kfs_cube_fusion_node = kfs_cube_fusion.kfs_cube_fusion_node:main",
        ],
    },
)
