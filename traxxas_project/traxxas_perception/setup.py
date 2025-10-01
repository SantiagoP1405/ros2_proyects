from setuptools import find_packages, setup

package_name = 'traxxas_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['traxxas_perception/modelo_senales.tflite']),
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
            'traxxas_neural_net = traxxas_perception.traxxas_neural_net:main',
            'traxxas_line_follower = traxxas_perception.traxxas_line_follower:main'
        ],
    },
)
