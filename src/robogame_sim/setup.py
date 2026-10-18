from glob import glob

from setuptools import find_packages, setup

package_name = 'robogame_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['arena_editor.html', 'arena_draw.js',
                                 'robot_console.html']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/layouts', glob('layouts/*.json')),
        ('share/' + package_name + '/models/robogame_arena_cad',
            glob('models/robogame_arena_cad/model.*')),
        ('share/' + package_name + '/models/robogame_arena_cad/meshes',
            glob('models/robogame_arena_cad/meshes/*.stl')),
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
            'racing_teleop = robogame_sim.racing_teleop:main',
            'chase_node = robogame_sim.chase_node:main',
            'arena_editor = robogame_sim.arena_editor:main',
            'fpv_toggle = robogame_sim.fpv_toggle:main',
            'controller_robots = robogame_sim.controller_robots:main',
            'active_arena = robogame_sim.active_arena:main',
            'robot_console = robogame_sim.robot_console:main',
        ],
    },
)
