import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'hex_docking'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'),
         glob('models/*.sdf')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nisaanth',
    maintainer_email='nisaanth@todo.todo',
    description='Standalone hexagonal robot docking demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'docking_node = hex_docking.docking_node:main',
        ],
    },
)
