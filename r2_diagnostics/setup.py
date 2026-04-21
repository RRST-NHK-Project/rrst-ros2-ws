from setuptools import find_packages, setup

package_name = "r2_diagnostics"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/diagnostics.launch.py"]),
        ("share/" + package_name + "/config", ["config/diagnostics_config.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Development Team",
    maintainer_email="dev@example.com",
    description="R2 robot self-diagnostic package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "diagnostics_node = r2_diagnostics.diagnostics_node:main",
            "run_diagnostics = r2_diagnostics.cli:main",
        ],
    },
)
