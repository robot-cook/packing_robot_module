import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'packing_robot_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cook',
    maintainer_email='cook@euler-robotics.com',
    description='REQ_JOB/SET_PACK_POSE TCP server bridging clients to doosan-robot2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'packing_robot_server = packing_robot_module.server_node:main',
        ],
    },
)
