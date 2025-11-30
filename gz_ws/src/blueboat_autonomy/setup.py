from setuptools import setup
import os
from glob import glob

package_name = 'blueboat_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.perception', f'{package_name}.mapping', f'{package_name}.planning', f'{package_name}.control', f'{package_name}.mission'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Modular autonomy system for BlueBoat using ZED camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_node = blueboat_autonomy.main:main',
        ],
    },
)
