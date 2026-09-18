import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cobot3_navigation'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        # Map files
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),

        # Nav2 parameter files
        (
            os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')
        ),

        # RViz config files
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='rokey',
    maintainer_email='todo@todo.com',

    description='Navigation package for cobot3',
    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        ],
    },
)