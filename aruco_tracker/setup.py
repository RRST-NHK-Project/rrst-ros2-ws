from setuptools import find_packages, setup

package_name = 'aruco_tracker'

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
    maintainer_email='koki2022@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'aruco_test = aruco_tracker.aruco_test:main',
        'aruco_generator = aruco_tracker.aruco_generator:main',
        'camera = aruco_tracker.camera:main',
        'camera_calibration = aruco_tracker.camera_calibration:main',
        'aruco_viewer = aruco_tracker.aruco_viewer:main'
        ],
    },
)
