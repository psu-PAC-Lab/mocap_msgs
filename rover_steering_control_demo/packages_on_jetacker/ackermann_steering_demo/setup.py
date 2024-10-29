from setuptools import find_packages, setup

package_name = 'ackermann_steering_demo'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Fortier',
    maintainer_email='dean4ta@gmail.com',
    description='read rigid body pose messages and send commands to drive the rover',
    license='unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steering_demo = ackermann_steering_demo.steering_demo_node:main'
        ],
    },
)
