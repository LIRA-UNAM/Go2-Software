import os
from glob import glob
from setuptools import setup

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	# Instalar launch y urdf
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        ('share/' + package_name + '/urdf', ['urdf/go2_description.urdf']),
        ('share/' + package_name + '/dae', [
            'dae/base.dae',
            'dae/foot.dae',
            'dae/hip.dae',
            'dae/calf_mirror.dae',
            'dae/thigh_mirror.dae',
        ]),
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
	    'joint_state_relay = robot_description.joint_state_relay:main',
        ],
    },
)
