from setuptools import setup

package_name = 'rpi_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = rpi_test.servo:main',
            'servo2 = rpi_test.servo2:main',
            'imu = rpi_test.imu:main',
            'gnss = rpi_test.gnss:main',
            'motor = rpi_test.motor:main'
        ],
    },
)
