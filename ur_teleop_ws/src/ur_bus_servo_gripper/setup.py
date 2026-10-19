from setuptools import setup
setup(name='ur_bus_servo_gripper', version='0.1.0', packages=['ur_bus_servo_gripper'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/ur_bus_servo_gripper']),
                  ('share/ur_bus_servo_gripper', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, license='MIT',
      maintainer='Local teleoperation project', maintainer_email='maintainer@example.com',
      description='Independent supervised ST3215 gripper',
      entry_points={'console_scripts': ['gripper = ur_bus_servo_gripper.gripper_node:main']})
