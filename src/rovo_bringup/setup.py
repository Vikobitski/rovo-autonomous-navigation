# import os
# from glob import glob
# from setuptools import find_packages, setup

# package_name = 'rovo_bringup'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # WICHTIG: Diese Zeilen kopieren deine Launch-Dateien
#         (os.path.join('share', package_name, 'launch'), 
#             glob(os.path.join('launch', '*.launch.py'))),
            
#         # WICHTIG: Diese Zeilen kopieren deine Config-Dateien
#         (os.path.join('share', package_name, 'config'), 
#             glob(os.path.join('config', '*.yaml'))),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='rovo',
#     maintainer_email='kirsjo01@thu.de',
#     description='TODO: Package description',
#     license='TODO: License declaration',
#     extras_require={
#         'test': [
#             'pytest',
#         ],
#     },
#     entry_points={
#         'console_scripts': [
#         ],
#     },
# )

import os
from glob import glob
from setuptools import setup

package_name = 'rovo_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # WICHTIG: Installiert alle .launch.py Dateien aus dem 'launch' Ordner
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
            
        # WICHTIG: Installiert alle .yaml Dateien aus dem 'config' Ordner
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovo',
    maintainer_email='rovo@todo.todo',
    description='Bringup-Paket für den Rovo-Roboter',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    # Dieser Abschnitt MUSS für dieses Paket leer sein
    entry_points={
        'console_scripts': [
        ],
    },
)
