from setuptools import find_packages, setup

package_name = 'imu_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'geometry_msgs',
        'adafruit-circuitpython-bno08x',
        'adafruit-circuitpython-busdevice',
        'Adafruit-Blinka',
    ],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='luis.anchundia27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_publisher.imu_publisher_node:main'
        ],
    },
)
