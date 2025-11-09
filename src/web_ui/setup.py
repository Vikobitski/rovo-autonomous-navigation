import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Installiere alle Dateien aus dem 'ui_files'-Ordner
        # in das Verzeichnis 'install/web_ui/share/web_ui/ui_files'
        (os.path.join('share', package_name, 'ui_files'), glob('ui_files/*.*'))
        
        # Wenn du auch Launch-Dateien in diesem Paket hättest:
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovo',
    maintainer_email='kirsjo01@thu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_server_node = web_ui.web_server_node:main',
        ],
    },
)
