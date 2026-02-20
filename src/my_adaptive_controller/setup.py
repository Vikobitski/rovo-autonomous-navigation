from setuptools import find_packages, setup

package_name = 'my_adaptive_controller'

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
    maintainer='samuel',
    maintainer_email='samuel@todo.todo',
    description='Adaptive controller for Pi 5',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nickname = Package.FileName:FunctionName
            'trajectory_follower = my_adaptive_controller.trajectory_follower_node:main',
        ],
    },
)