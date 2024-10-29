from setuptools import find_packages, setup

package_name = 'tf_repub'

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
    description='republish tf messages to pose messaged readable by ROS2 foxy',
    license='unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'transform_republisher = tf_repub.transform_republisher:main',
        ],
    },
)
