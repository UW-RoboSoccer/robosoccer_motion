from setuptools import setup

package_name = 'motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'std_msgs'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='UWRS',
    maintainer_email='uwrobosoccer@gmail.com',
    description='Motion planner node for RoboSoccer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planner_node = motion.motion_planner_node:main',
        ],
    },
)
