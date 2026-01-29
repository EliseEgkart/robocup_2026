from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files 전체 설치
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch', 'nav2_bringup'),
            glob('launch/nav2_bringup/*.py')),

        # params 폴더 전체 설치
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),

        # map 폴더 전체 설치
        (os.path.join('share', package_name, 'map'),
            glob('map/*')),

        # rviz 폴더 전체 설치
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonshot',
    maintainer_email='ky942400@gmail.com',
    description='Navigation package for AMR',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigator_node = robot_navigator.navigator:main'
        ],
    },
)
