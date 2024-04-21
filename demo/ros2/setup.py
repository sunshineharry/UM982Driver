from setuptools import find_packages, setup

package_name = 'um982_driver'

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
    maintainer='hankunjiang',
    maintainer_email='hankunjiang@outlook.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um982_serial_driver_node = um982_driver_ros2.um982_serial_driver_node:main'
        ],
    },
)
