from setuptools import setup
import os
from glob import glob

package_name = 'misty_speech_therapy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max Knaefler',
    maintainer_email='mjknaefler@gmail.com',
    description='ROS 2 package for using Misty II robot in speech therapy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'misty_wrapper = misty_speech_therapy.misty_wrapper:main',
            'speech_processor = misty_speech_therapy.speech_processor:main',
            'conversation_manager = misty_speech_therapy.conversation_manager:main',
        ],
    },
)