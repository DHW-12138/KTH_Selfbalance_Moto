from glob import glob
from setuptools import find_packages, setup
setup(
    name='iim42652_gyro', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/iim42652_gyro']),
                ('share/iim42652_gyro', ['package.xml']),
                ('share/iim42652_gyro/launch', glob('launch/*.launch.py')),
                ('share/iim42652_gyro/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Maintainer', maintainer_email='maintainer@example.com',
    description='IIM-42652 SPI gyroscope publisher', license='Apache-2.0',
    entry_points={'console_scripts': ['imu_node = iim42652_gyro.node:main',
                                      'calibrate_gyro = iim42652_gyro.calibrate:main']},
)
