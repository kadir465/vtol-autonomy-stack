from setuptools import find_packages, setup

package_name = 'vtol_control'

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
           'gcs_listener = vtol_control.gcs_listener:main',
        'state_machine = vtol_control.state_machine:main',
        'visual_servo  = vtol_control.visual_servo:main',
        'gcs_panel     = vtol_control.gcs_panel:main',
        ],
    },
)
