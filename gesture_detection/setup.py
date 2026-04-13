from setuptools import find_packages, setup

package_name = "gesture_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/gesture_detection.launch.py"]),
        (
            "share/" + package_name + "/launch",
            ["launch/hand_state_detection.launch.py"],
        ),
        ("share/" + package_name + "/models", ["models/gesture_recognizer.task"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="tashikou1682@gmail.com",
    description="MediaPipeでジェスチャー検知を行い結果をPublishするパッケージ",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "gesture_detector = gesture_detection.gesture_detector_node:main",
            "hand_state_detector = gesture_detection.hand_state_detector_node:main",
            "gesture_viewer = gesture_detection.gesture_viewer_node:main",
        ],
    },
)
