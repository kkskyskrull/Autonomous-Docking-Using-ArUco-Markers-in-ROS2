from setuptools import setup

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/robot_description/urdf', ['robot_description/urdf/diffbot.urdf']),
        ('share/' + package_name + '/robot_description/rviz', ['robot_description/rviz/diffbot.rviz']),
        ('share/' + package_name + '/robot_description/launch', ['robot_description/launch/display_diffbot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Robot description package with URDF and RViz',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
