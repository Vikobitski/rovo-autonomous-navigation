from setuptools import find_packages, setup

package_name = 'rovo_driver'

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
            'rovo_driver_node = rovo_driver.rovo_driver_node:main',
            'rovo_driver_node_testing = rovo_driver.rovo_driver_node_testing:main',
        ],
    },
)
