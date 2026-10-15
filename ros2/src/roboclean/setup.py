from setuptools import find_packages, setup
from glob import glob

package_name = 'roboclean'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_data={
        'roboclean.utils': ['*.rb'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.yml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Lukas Wrana',
    maintainer_email='dn303@uni-heidelberg.de',
    description='A Lidar-based vacuum cleaner robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = roboclean.explorer_node:main',
            'cleaner = roboclean.cleaner_node:main',
            'web_bridge = roboclean.web_bridge:main',
            'base_controller = roboclean.base_controller:main',
            'system_monitor = roboclean.system_monitor_node:main',
            'initial_pose_publisher = roboclean.initial_pose_publisher:main',
            'lidar_watchdog = roboclean.utils.lidar_watchdog_node:main',
            'wait_for_scan = roboclean.utils.wait_for_scan_node:main',
            'barrier_publisher = roboclean.barrier_publisher_node:main'
        ],
    },
)
