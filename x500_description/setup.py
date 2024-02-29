from setuptools import setup
import os
from glob import glob

package_name = 'x500_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
  	('share/' + package_name+'/urdf/', glob('urdf/*')),
  	('share/' + package_name+'/rviz/', glob('rviz/*')),
  	('share/' + package_name+'/meshes/collision/', glob('meshes/collision/*')),
  	('share/' + package_name+'/meshes/visual/', glob('meshes/visual/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='TODO:',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tf_broadcaster = x500_description.aruco_tf_broadcast:main',
            'map_tf_broadcaster = x500_description.map_tf_broadcast:main'
        ],
    },
)
