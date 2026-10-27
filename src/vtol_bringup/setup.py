import os
from glob import glob

from setuptools import setup

package_name = 'vtol_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uav',
    maintainer_email='uav@todo.todo',
    description='VTOL Multi-Agent Simulation Bringup',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'multi_vtol_commander = vtol_bringup.multi_vtol_commander:main',
        ],
    },
)
