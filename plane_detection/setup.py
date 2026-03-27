from setuptools import find_packages, setup

package_name = 'plane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/plane_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='tashikou1682@gmail.com',
    description='深度カメラから平面を検知し位置（座標・姿勢・距離）をPublishするパッケージ',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plane_detector = plane_detection.plane_detector_node:main',
            'plane_viewer = plane_detection.plane_viewer_node:main',
        ],
    },
)
