from setuptools import find_packages, setup

package_name = 'ros2can'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros2can.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='tashikou1682@gmail.com',
    description=(
        'xiao_esp32_s3_smd_serial_bridge (CANバス複数マイコン対応) 専用のデバッグ GUI。'
        'CANノードを選択してアクチュエータへの指令値を直接送信し、センサ値を表示する。'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2can = ros2can.main:main',
        ],
    },
)
