from setuptools import find_packages, setup

package_name = "ld19_eight_direction_distance"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/ld19_eight_direction_distance.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@example.com",
    description="Subscribe LD19 LaserScan and print 8-direction distances.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ld19_eight_direction_node = ld19_eight_direction_distance.ld19_eight_direction_node:main",
        ],
    },
)
