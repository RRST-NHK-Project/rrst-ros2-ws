from setuptools import find_packages, setup

package_name = 'cube_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cube_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='tashikou1682@gmail.com',
    description='深度カメラから立方体を検知し位置（座標・姿勢・距離）をPublishするパッケージ',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cube_detector = cube_detection.cube_detector_node:main',
            'cube_viewer = cube_detection.cube_viewer_node:main',
        ],
    },
)
