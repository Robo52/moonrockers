# moonrockers
Repo for all of the Moonrockers team programming work

# rover_perception
Hosts the RealSense driver node to publish rectified images, depth, and IMU data.
Contains the AprilTag detection node that runs on the camera data.
Includes a node to rotate the camera via the stepper motor (using the encoder feedback).

# rover_power_monitor
ROS 2 Python node that uses SocketCAN (via the python-can library) to read telemetry from the REV Power Distribution Hub (PDH) on your CAN bus and publish a standard sensor_msgs/BatteryState message. In the design the PDH and the four SparkMax controllers are daisy chained on the same CAN bus.

