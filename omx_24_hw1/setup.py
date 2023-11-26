from setuptools import find_packages, setup

package_name = 'omx_24_hw1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='redlightning',
    maintainer_email='nilampooranan@wpi.edu',
    description='Subscribes to Joint state values of robot and prints the pose of robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'fk_sub = omx_24_hw1.fk_sub:main'
        ],
    },
)
