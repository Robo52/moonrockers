from setuptools import find_packages, setup

package_name = 'rover_control'

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
    maintainer='moon_rockers_desk',
    maintainer_email='john.miller@mines.sdsmt.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_rover_executable = rover_control.autonomous_rover_node:main'
        ],
    },
)
