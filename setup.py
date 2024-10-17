from setuptools import find_packages, setup

package_name = 'f7_udp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'listener = f7_udp.PS4_listener:main',
        '4_omni = f7_udp.W4_Omni_Driver:main',
        'setoshio = f7_udp.yolov8_setoshio_pub:main',
        'setoshio_gui = f7_udp.yolo_setoshio_gui:main',
        'cr24_main = f7_udp.cr24_main:main',
        'cr24_test = f7_udp.cr24_test:main',
        'cr24_gui = f7_udp.cr24_gui:main',
        'cr24_main_manual = f7_udp.cr24_manual:main',
        'cr24_main_manual2 = f7_udp.cr24_manual2:main',
        'cr24_pos = f7_udp.cr24_pos:main',
        'cr24_wlcam = f7_udp.cr24_yolo_wireless:main',
        'cr24_main_apk = f7_udp.cr24_main_unity:main',
        'cr24_manual2_apk = f7_udp.cr24_manual2_unity:main',
        'nr25_omni = f7_udp.NHK2025_Omni_Driver:main',
        'nr25_omni_imu = f7_udp.NHK2025_Omni_Attitude_Control:main',
        'imu_to_odom = f7_udp.imu_to_odom:main'
        ],
    },
)
