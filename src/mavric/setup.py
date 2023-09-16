from setuptools import setup

package_name = 'mavric'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mavric',
    maintainer_email='natestorms416@gmail.com',
    description='Ros2 Package for controlling MAVRIC:SCARAB',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu = py_pybsub.BNO055_IMU:main'
        ],
    },
)
