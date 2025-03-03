#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import can
import struct
import math

class RoverPowerMonitor(Node):
    def __init__(self):
        super().__init__('rover_power_monitor_node')
        self.get_logger().info("Rover Power Monitor Node Starting...")

        # Publisher for battery state
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)

        # Set up a timer callback to periodically check for new CAN messages.
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize the CAN bus (using SocketCAN on channel 'can0')
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.get_logger().info("CAN bus initialized on channel 'can0'.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {e}")
            self.bus = None

        # Define the CAN ID for PDH telemetry (this is a placeholder; adjust according to your PDH documentation)
        self.PDH_CAN_ID = 0x200

    def timer_callback(self):
        if self.bus is None:
            return

        # Read messages from the CAN bus (non-blocking)
        msg = self.bus.recv(timeout=0.0)
        if msg is None:
            # No new message this cycle.
            return

        # Check if this message is from the PDH (based on the CAN ID)
        if msg.arbitration_id == self.PDH_CAN_ID:
            # Parse the data.
            # (The exact format depends on REV’s PDH documentation.
            # For illustration, assume the message data layout is:
            #   - bytes 0-1: Battery Voltage in mV (unsigned short)
            #   - bytes 2-3: Battery Current in mA (signed short)
            #   - bytes 4: Battery temperature in °C (signed char)
            # Adjust unpack format as needed.)
            try:
                voltage_mV, current_mA, temp = struct.unpack('>HhB', msg.data[:5])
                voltage = voltage_mV / 1000.0  # convert to volts
                current = current_mA / 1000.0  # convert to amps
            except Exception as e:
                self.get_logger().warn(f"Failed to parse PDH message: {e}")
                return

            # Create and populate the BatteryState message
            batt_msg = BatteryState()
            batt_msg.header.stamp = self.get_clock().now().to_msg()
            batt_msg.header.frame_id = "battery_link"  # use a suitable frame id
            batt_msg.voltage = voltage
            batt_msg.current = current
            # Here we assume you have some battery design capacity in amp-hours:
            batt_msg.charge = float('nan')
            batt_msg.capacity = float('nan')
            # For percentage, you might calculate if you know the battery specs:
            batt_msg.percentage = float('nan')
            batt_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            batt_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            batt_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            batt_msg.present = True

            self.battery_pub.publish(batt_msg)
            self.get_logger().debug(
                f"Published BatteryState: voltage={voltage:.2f}V, current={current:.2f}A, temp={temp}°C"
            )

def main(args=None):
    rclpy.init(args=args)
    node = RoverPowerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
