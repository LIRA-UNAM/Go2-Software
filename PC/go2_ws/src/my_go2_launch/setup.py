from setuptools import setup
import os
from glob import glob

package_name = 'my_go2_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # asegúrate que coincide con la carpeta que contiene __init__.py
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_to_laserscan_qos = my_go2_launch.pointcloud_to_laserscan_qos:main',
	    'img_publisher = my_go2_launch.gstreamer_image_publisher:main',
        'speak = my_go2_launch.speak:main',
        'odom_publisher = my_go2_launch.odom_publisher:main',
	'laser_scan = my_go2_launch.laserscan_pass_through:main', 
        ],
    },
)
