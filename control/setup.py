from setuptools import setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=['wpt_tracker'],
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
    description='The control package',
    license='TODO',
    entry_points={
        'console_scripts': [
        ],
    },
)
