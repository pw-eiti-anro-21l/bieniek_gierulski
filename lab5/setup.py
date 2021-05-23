import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

from setuptools import setup

package_name = 'lab5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('lab5/dh.txt')),
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
            'interpolator_point = lab5.interpolator_point:main',
            'oint = lab5.oint:main',
            'inv_kin = lab5.inv_kin:main',
            'interpolator = lab5.interpolator:main',
            'jint = lab5.jint:main',
            'marker_array_drawer = lab5.marker_array_drawer:main',
            'no_kdl = lab5.no_kdl:main'

        ],
    },
)
