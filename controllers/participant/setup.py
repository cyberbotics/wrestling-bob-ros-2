from setuptools import setup

package_name = 'participant'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, 'utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/resource', ['resource/webots_controller.urdf']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Demonstrates how to use the Motion_library class to play a motion file in ROS 2.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={}
)
