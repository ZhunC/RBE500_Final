from setuptools import find_packages, setup

package_name = 'omx_24_vel_kin'

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
    maintainer_email='redlightning@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'service = omx_24_vel_kin.velkin:main',
        	'qinc = omx_24_vel_kin.q_inc:main',
        	'client = omx_24_vel_kin.move_incremental:main'
        ],
    },
)
