from setuptools import setup

package_name = 'carla_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=['carla_bridge', 'carla_agent'],
    package_dir={'': 'scripts'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaohang',
    maintainer_email='shaohang@todo.todo',
    description='The carla_bridge package',
    license='TODO',
    entry_points={
        'console_scripts': [
        ],
    },
)
