from setuptools import find_packages, setup

setup(
    name='motorcycle_control', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages', ['resource/motorcycle_control']),
                ('share/motorcycle_control', ['package.xml', 'README.md'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Maintainer', maintainer_email='maintainer@example.com',
    description='ROS 2 motorcycle speed master for Waveshare USB-CAN-A',
    license='Apache-2.0',
    entry_points={'console_scripts': ['master_node = motorcycle_control.node:main']},
)
