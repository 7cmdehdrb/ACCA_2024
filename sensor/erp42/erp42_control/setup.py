import os
from setuptools import setup
from glob import glob

package_name = 'erp42_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/stanley.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='gjs',
    maintainer_email='junseonggg2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_opener = erp42_control.path_opener:main',
            'controller_2024 = erp42_control.controller_2024:main',
            'pid_tunning = erp42_control.pid_tunning:main',
            'controller_autoware = erp42_control.controller_autoware:main',
            'encoder_imu = erp42_control.encoder_imu:main',
        ],
    },
)
