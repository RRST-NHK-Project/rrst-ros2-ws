from setuptools import find_packages, setup

package_name = "webcam_calibration"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/webcam_calibration.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="tashikou1682@gmail.com",
    description=(
        "WebカメラをOpenCVでキャリブレーションし、"
        "カメラパラメータYAMLを保存するパッケージ"
    ),
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "webcam_calibrator = " "webcam_calibration.webcam_calibrator_node:main",
        ],
    },
)
