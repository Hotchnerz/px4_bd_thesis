from setuptools import setup
import os
from glob import glob

package_name = 'flight_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radam',
    maintainer_email='radam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard = flight_test.offboard_predep:main',
            'test_offboard = flight_test.offboard_test:main',
            'old_offboard = flight_test.old_offboard:main',
            'offboard_perch = flight_test.offboard_perch:main',
            'control = flight_test.control:main'
        ],
    },
)
