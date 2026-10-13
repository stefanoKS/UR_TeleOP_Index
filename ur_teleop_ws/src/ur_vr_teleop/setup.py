from setuptools import setup
from glob import glob

setup(
    name='ur_vr_teleop', version='0.1.0', packages=['ur_vr_teleop'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ur_vr_teleop']),
        ('share/ur_vr_teleop', ['package.xml', 'README.md']),
        ('share/ur_vr_teleop/launch', glob('launch/*.launch.py')),
        ('share/ur_vr_teleop/rviz', glob('rviz/*.rviz')),
    ],
    package_data={'ur_vr_teleop': ['setup_migration.json', 'setup_source_compatibility.json']},
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Local teleoperation project', maintainer_email='maintainer@example.com',
    description='Guarded UR10e teleop commissioning and local GUI', license='MIT',
    entry_points={'console_scripts': ['preview = ur_vr_teleop.preview:main',
        'commissioning = ur_vr_teleop.commissioning:main', 'gui = ur_vr_teleop.gui:main',
        'desktop = ur_vr_teleop.desktop:main',
        'engineering_record = ur_vr_teleop.engineering:main']},
)
