from setuptools import find_packages, setup

package_name = 'ros2_project_sc22hw'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/map', ['map/map.pgm', 'map/map.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='ROS2 Project for RGB Box Detection and Navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = ros2_project_sc22hw.robot_controller:main',
        ],
    },
)
