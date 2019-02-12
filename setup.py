from setuptools import setup, find_packages
import platform

setup(
    name='IODynamixel',
    version='0.5.0',
    packages=['IODynamixel', 'vrep'],
    package_dir={'': 'src'},
    package_data={'vrep': ['*.so'], 'IODynamixel': ['creatures/*', 'movements/*']},
    license='MIT',
    description='Library to control Dynamixel motors vía creature scheme.',
    long_description=open('README.md').read(),
    url='https://github.com/cstopics/IODynamixel',
    author='Camilo Camacho',
    author_email='camilo.im93@gmail.com',
    install_requires=['pyserial', 'dynamixel_sdk']
)