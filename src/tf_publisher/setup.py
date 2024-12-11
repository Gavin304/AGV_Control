from setuptools import setup
from glob import glob
import os

package_name = 'tf_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asrlab',
    maintainer_email='gavin.arpandy@gmail.com',
    description='Description of the package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_publisher_node = tf_publisher.tf_publisher:main',
            'dynamic_frame_tf2_broadcaster = tf_publisher.dynamic_frame_tf2_broadcaster:main',
            'fixed_frame_tf2_broadcaster = tf_publisher.fixed_frame_tf2_broadcaster:main',
            'tf2_broadcaster = tf_publisher.tf2_broadcaster:main'
        ],
    },
)
