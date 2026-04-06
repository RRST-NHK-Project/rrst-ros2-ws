from setuptools import find_packages, setup

package_name = "aruco_tracker"

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
                "launch/aruco_tracker.launch.py",
                "launch/aruco_webcam_tracker.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="koki2022@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "aruco_pose_publisher = aruco_tracker.aruco_pose_publisher:main",
            "aruco_viewer = aruco_tracker.aruco_viewer:main",
            "aruco_webcam_detector = aruco_tracker.aruco_webcam_detector:main",
        ],
    },
)
