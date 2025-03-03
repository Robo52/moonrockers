from setuptools import find_packages, setup

package_name = 'rover_perception'

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
    description='Hosts the RealSense driver node to publish rectified images, depth, and IMU data. Contains the AprilTag detection node that run on the camera data. Also includes a node to rotate the camera via the stepper motor (using the encoder feedback).',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_pose_estimator = rover_perception.apriltag_pose_estimator:main'
        ],
    },
)
