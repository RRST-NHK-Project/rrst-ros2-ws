from setuptools import find_packages, setup

package_name = "webcam_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/calibration",
            ["calibration/webcam_calibration.yaml"],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/webcam_publisher.launch.py",
                "launch/webcam_common.launch.py",
                "launch/webcam_publisher_calibrated.launch.py",
                "launch/webcam_calibrated_with_viewer.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="tashikou1682@gmail.com",
    description="Webカメラ映像をROS 2トピックにPublishするパッケージ",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "webcam_publisher = webcam_publisher.webcam_publisher_node:main",
        ],
    },
)
