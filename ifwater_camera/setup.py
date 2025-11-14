from setuptools import find_packages, setup

package_name = 'ifwater_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy','opencv-python'],
    zip_safe=True,
    maintainer='rdc_jetson',
    maintainer_email='rdc_jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = ifwater_camera.camera_publisher:main',
        ],
    },
)
