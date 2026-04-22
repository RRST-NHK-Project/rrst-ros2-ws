from setuptools import find_packages, setup

package_name = "kfs_cube_identifier"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/kfs_cube_identifier.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="KFS cube face identifier using color filtering, homography, and AKAZE matching",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "kfs_cube_identifier_node = kfs_cube_identifier.kfs_cube_identifier_node:main",
            "kfs_debug_image_viewer = kfs_cube_identifier.debug_image_viewer_node:main",
        ],
    },
)
