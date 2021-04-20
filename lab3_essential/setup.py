import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

from setuptools import setup

package_name = 'lab3_essential'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('lab3_essential/dh.txt')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krzysiek',
    maintainer_email='krzys.byniek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kdl_fk = lab3_essential.kdl_fk:main',
            'no_kdl = lab3_essential.no_kdl:main'
        ],
    },
)
