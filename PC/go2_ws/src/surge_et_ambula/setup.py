from setuptools import setup
from glob import glob
import os

package_name = 'surge_et_ambula'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # tu código Python dentro de surge_et_ambula/
    data_files=[
        # Necesario para que ament index encuentre el paquete
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml en share/<paquete>
        ('share/' + package_name, ['package.xml']),

        # Launch files (dentro del paquete de código)
        ('share/' + package_name + '/launch',
         glob('surge_et_ambula/launch/*.launch.py')),

        # --- Recursos que quieres compartir/instalar ---
        # Carpeta con mapas (ajusta el nombre si usas 'maps' en vez de 'lab')
        ('share/' + package_name + '/maps',
         glob('maps/lab/*')),

        # (OPCIONAL) otras carpetas de recursos si existen:
        # ('share/' + package_name + '/urdf',   glob('urdf/*')),
        # ('share/' + package_name + '/rviz',   glob('rviz/*')),
        # ('share/' + package_name + '/config', glob('config/*')),
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
            # 'nodo=surge_et_ambula.modulo:main',
        ],
    },
)
