from setuptools import find_packages, setup

package_name = 'serial_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/serial_comm.launch.py']),
        ('share/' + package_name, ['launch/teleop_launch.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonshot',
    maintainer_email='ky942400@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_publisher_node = serial_comm.imu_publisher:main',
            'base_main_node = serial_comm.base_main:main',
            'test_md_teleop_node = serial_comm.test_md_teleop:main',
        ],
    },
)
