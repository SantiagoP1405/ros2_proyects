from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'greenhouse_robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'params'), 
         glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santiago_gomez',
    maintainer_email='santiago_gomez@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'nav2_test = greenhouse_robot_navigation.nav2_test:main',
            'routine_test = greenhouse_robot_navigation.routine_test:main',
        ],
    },
)
