from setuptools import setup
import os
from glob import glob

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # TODO: si usas layout con 'src/', ver nota abajo
    data_files=[
        # Índice de recursos de ament
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Archivos de launch
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Archivos de configuración
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joel David Gago Garcia',      
    maintainer_email='jdgago9811@gmail.com',  
    description='Vision 2D (ROS 2) con OpenCV/cv_bridge.',
    license='MIT',                  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # nombre-ejecutable = paquete.modulo:funcion_main,
	    'webcam_pub = vision_pkg.webcam_publisher:main',
            'image_sub = vision_pkg.image_subscriber:main',
            'dino_server = vision_pkg.grounding_dino_server:main',
	    'yolo_node = vision_pkg.yolo_node:main',
	    'dino_client = vision_pkg.dino_client:main'
        ],
    },
)
