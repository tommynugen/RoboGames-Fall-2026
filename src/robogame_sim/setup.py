from glob import glob

from setuptools import find_packages, setup

package_name = 'robogame_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tommynugen',
    maintainer_email='thomasnugen@gmail.com',
    description='Gazebo arena and simulated ESP32 robots for Robogame',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sim_firmware = robogame_sim.sim_firmware:main',
            'robot_sdf = robogame_sim.robot_sdf:main',
            'chase_node = robogame_sim.chase_node:main',
        ],
    },
)
