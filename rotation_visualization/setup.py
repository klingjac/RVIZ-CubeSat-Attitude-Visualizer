from setuptools import setup

package_name = 'rotation_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/VirtualSatv8.stl']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'tf2_ros', 'tf2_geometry_msgs'],
    zip_safe=True,
    maintainer='Jacob Klingler',
    maintainer_email='your.email@example.com',
    description='Example Python package for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualization_node = rotation_visualization.visualization_node:main'
        ],
    },
)

