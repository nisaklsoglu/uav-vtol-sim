import os
from glob import glob

from setuptools import setup

package_name = 'vtol_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/config', [
            'config/sitl_no_prearm.parm',
            'config/suas_mission.json',
            'config/fastdds_udp_loopback.xml',
        ]),
        ('share/' + package_name + '/config/cases', [
            'config/cases/case1_navigation.json',
            'config/cases/case2_obstacles_separation.json',
            'config/cases/case3_target_drop.json',
        ]),
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
