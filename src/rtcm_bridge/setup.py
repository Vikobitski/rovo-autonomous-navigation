from setuptools import find_packages, setup

package_name = 'rtcm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rtcm_bridge_launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
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
            'rtcm_bridge_node = rtcm_bridge.rtcm_bridge_node:main',
            'rtcm_bridge = rtcm_bridge.rtcm_bridge_node:main'
        ],
    },
)
