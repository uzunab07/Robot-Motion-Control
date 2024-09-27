from setuptools import setup

package_name = 'turtlebot3_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root-dev',
    maintainer_email='root-dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot3_simulation_launch = turtlebot3_simulation.odom_compute:main',
            'drive_robot = turtlebot3_simulation.drive_robot:main'
        ],
    },
)
