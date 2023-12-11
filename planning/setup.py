import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_dir={'': 'scripts'},
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.osm'))),
    ],
    zip_safe=True,
    maintainer='shaohang',
    maintainer_email='shaohang@todo.todo',
    description='The planning package',
    license='TODO',
    entry_points={
        'console_scripts': [
            "planning_manager_node = planning.planning_manager_node:main",
        ],
    },
)
