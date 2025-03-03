from setuptools import find_packages, setup

package_name = 'rover_power_monitor'

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
    maintainer='John Miller',
    maintainer_email='john.miller@mines.sdsmt.edu',
    description='Monitors Rover Power via Can Bus',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_power_monitor_node = rover_power_monitor.rover_power_monitor_node:main'
        ],
    },
)
