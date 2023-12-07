from setuptools import setup

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=['global_planner', 'local_planner', 'planning_utils', 'motion_predictor'],
    package_dir={'': 'scripts'},
    install_requires=['setuptools'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    zip_safe=True,
    maintainer='shaohang',
    maintainer_email='shaohang@todo.todo',
    description='The planning package',
    license='TODO',
    entry_points={
        'console_scripts': [
        ],
    },
)
