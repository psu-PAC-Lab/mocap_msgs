import os
from glob import glob
from setuptools import setup

package_name = 'jetmax_optitrack_feedback'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Fortier',
    maintainer_email='dean4ta@gmail.com',
    description='application to receive pose and actuate the robot',
    license='unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetmax_feedback = jetmax_optitrack_feedback.jetmax_feedback_node:main'
        ],
    },
)
