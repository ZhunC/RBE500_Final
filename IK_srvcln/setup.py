from setuptools import find_packages, setup
package_name = 'IK_srvcln'

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
    maintainer='zcheng',
    maintainer_email='zcheng2@wpi.edu',
    description='RBE 500 IK server and client',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'service = IK_srvcln.IK_server:main'
        ],
    },
)
