from setuptools import setup
import os
from glob import glob

package_name = 'kfs_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # すべて絶対パス（/から始まるパス）を使わず、相対パスで記述します
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), 
            glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@test.com',
    description='KFS package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'matcher_node = kfs_pkg.matcher_node:main',
            'udp_node = kfs_pkg.node:main',
        ],
    },
)