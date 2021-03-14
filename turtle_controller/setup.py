from setuptools import setup
import os
from glob import glob
package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wojtek',
    maintainer_email='wojtek.gierulski@gmail.com',
    description='Apache License 2.0',
    license='lab 1',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'controller = turtle_controller.controller:main',
        ],
    },
)
