from setuptools import find_packages, setup

package_name = 'hardware_test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sujalu',
    maintainer_email='sujaldhote82@gmail.com',
    description='Hardware test package for Raspberry Pi + ROS2 integration',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Pi side node (servo subscriber)
            'servo_subscriber = hardware_test_pkg.servo_subscriber:main',
            # Laptop side node (controller / publisher)
            'servo_controller = hardware_test_pkg.servo_controller:main',
        ],
    },
)
