from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vtol_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # LAUNCH dosyalarını sisteme tanıt (uav, gcs, simulation)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # CONFIG (YAML) dosyalarını sisteme tanıt
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kadir',
    maintainer_email='kadir@todo.todo',
    description='VTOL Kamikaze Drone Bringup Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # Bringup paketi genellikle script içermez, sadece launch yönetir.
        ],
    },
)