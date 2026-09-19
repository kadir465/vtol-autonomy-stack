import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vtol_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'models', 'gazebo_models'), glob('models/gazebo_models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kadir',
    maintainer_email='kadir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
        'console_scripts': [
            'stream_node = vtol_vision.stream_node:main',
            'vision_node = vtol_vision.vision_node:main',
        ],
    },
)
