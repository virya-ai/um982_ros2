from setuptools import setup
package_name = 'um982_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Athar Ahmed',
    maintainer_email='athar.a@virya.ai',
    description='ROS 2 package for the UM982 GNSS receiver with RTK correction support - originally developed for the CUAV C-RTK2 HP, but compatible with any device using the UM982 chip.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'um982_node = um982_ros2.um982_node:main',
            'um982_node_ros2_standard = um982_ros2.um982_node_ros2_standard:main',
            'um982_node_updated = um982_ros2.um982_node_updated:main',
            'um982_node_updated_with_enu_heading = um982_ros2.um982_node_updated_with_enu_heading:main',
            'um982_path = um982_ros2.um982_path:main',
        ],
    },
)