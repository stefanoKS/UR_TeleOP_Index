from setuptools import setup
from glob import glob

name = 'ur_vr_sim_teleop'
setup(
    name=name, version='0.2.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/config', glob('config/*.yaml')),
        ('share/' + name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Local teleoperation project', maintainer_email='maintainer@example.com',
    description='Clutched six-axis control for a mock UR10e', license='MIT',
    entry_points={'console_scripts': ['teleop = ur_vr_sim_teleop.node:main']},
)
