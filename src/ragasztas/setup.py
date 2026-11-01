from glob import glob

from setuptools import find_packages, setup

package_name = 'ragasztas'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matecsizma2004',
    maintainer_email='mate.csizmadia@ewiser.hu',
    description='Ragasztás szimuláció KUKA KR 6 R900-2 robottal.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'glue_map_sensor = ragasztas.glue_map_sensor:main',
            'glue_motion = ragasztas.glue_motion:main',
        ],
    },
)
