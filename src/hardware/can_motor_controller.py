#!/usr/bin/env python3
"""
CAN Motor Controller for REV NEO Brushless Motors
Handles low-level CAN communication and motor control
John Miller john.miller@mines.sdsmt.edu
"""

import can
import struct
import time
import threading
import math
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
import logging

# logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class MotorStatus:
    velocity: float = 0.0
    position: float = 0.0
    current: float = 0.0
    temperature: float = 0.0
    voltage: float = 0.0
    faults: int = 0

class CANMotorController:
    """
    Controls REV motors via CAN bus
    Implements REV CAN protocol
    """
    
    # REV NEO CAN API IDs
    API_IDS = {
        'SET_DUTY_CYCLE': 0x02050080,
        'SET_VELOCITY': 0x02050480,
        'SET_POSITION': 0x02050C80,
        'SET_VOLTAGE': 0x02050180,
        'SET_CURRENT': 0x02051480,
        'STATUS0': 0x02051800,  # Velocity, voltage, current
        'STATUS1': 0x02051840,  # Position, temperature
        'STATUS2': 0x02051880,  # Faults
    }
    
    def __init__(self, config: dict):
        """
        Initialize CAN motor controller
        
        Args:
            config: Dictionary containing CAN and motor configuration
        """
        self.config = config
        self.motors = {}
        self.motor_status = {}
        self.can_bus = None
        self.running = False
        self.status_thread = None
        
        # Initialize motor status tracking
        for motor_name, can_id in config['MOTOR_CONFIG'].items():
            if isinstance(can_id, int):  # Skip non-motor config items
                self.motor_status[can_id] = MotorStatus()
        
        self.initialize_can()
    
    def initialize_can(self):
        """Initialize CAN bus connection"""
        try:
            # Set up CAN interface
            self.can_bus = can.interface.Bus(
                channel=self.config['CAN_CONFIG']['interface'],
                bustype='socketcan',
                bitrate=self.config['CAN_CONFIG']['bitrate']
            )
            logger.info(f"CAN bus initialized on {self.config['CAN_CONFIG']['interface']}")
            
            # Start status monitoring thread
            self.running = True
            self.status_thread = threading.Thread(target=self._status_monitor_loop, daemon=True)
            self.status_thread.start()
            
        except Exception as e:
            logger.error(f"Failed to initialize CAN bus: {e}")
            raise
    
    def _status_monitor_loop(self):
        """Background thread to monitor motor status messages"""
        while self.running:
            try:
                message = self.can_bus.recv(timeout=self.config['CAN_CONFIG']['timeout'])
                if message:
                    self._process_status_message(message)
            except can.CanTimeoutError:
                continue
            except Exception as e:
                logger.warning(f"CAN receive error: {e}")
    
    def _process_status_message(self, message: can.Message):
        """Process incoming status messages from motors"""
        can_id = message.arbitration_id
        motor_id = can_id & 0x3F  # Extract motor ID from CAN ID
        api_id = can_id & ~0x3F   # Extract API ID
        
        if motor_id not in self.motor_status:
            return
        
        try:
            if api_id == self.API_IDS['STATUS0']:
                # Velocity, voltage, current
                velocity, voltage, current = struct.unpack('<fff', message.data[:12])
                self.motor_status[motor_id].velocity = velocity
                self.motor_status[motor_id].voltage = voltage
                self.motor_status[motor_id].current = current
                
            elif api_id == self.API_IDS['STATUS1']:
                # Position, temperature
                position, temperature = struct.unpack('<ff', message.data[:8])
                self.motor_status[motor_id].position = position
                self.motor_status[motor_id].temperature = temperature
                
            elif api_id == self.API_IDS['STATUS2']:
                # Faults
                faults = struct.unpack('<I', message.data[:4])[0]
                self.motor_status[motor_id].faults = faults
                
        except Exception as e:
            logger.warning(f"Error processing status message: {e}")
    
    def set_velocity(self, motor_id: int, velocity: float):
        """
        Set motor velocity in RPM
        
        Args:
            motor_id: CAN ID of the motor
            velocity: Target velocity in RPM
        """
        try:
            # Clamp velocity to limits
            max_vel = self.config['MOTOR_CONFIG']['max_velocity'] * 60 / (math.pi * self.config['MOTOR_CONFIG']['wheel_diameter'])
            velocity = max(-max_vel, min(max_vel, velocity))
            
            # Prepare CAN message
            can_id = self.API_IDS['SET_VELOCITY'] | motor_id
            data = struct.pack('<f', velocity) + b'\x00' * 4
            
            message = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True
            )
            
            self.can_bus.send(message)
            
        except Exception as e:
            logger.error(f"Failed to set velocity for motor {motor_id}: {e}")
    
    def set_voltage(self, motor_id: int, voltage: float):
        """
        Set motor voltage (-12V to +12V)
        
        Args:
            motor_id: CAN ID of the motor
            voltage: Target voltage
        """
        try:
            # Clamp voltage to limits (DO NOT RAISE EVER FOR ANY REASON IDOT)
            voltage = max(-12.0, min(12.0, voltage))
            
            # Prepare CAN message
            can_id = self.API_IDS['SET_VOLTAGE'] | motor_id
            data = struct.pack('<f', voltage) + b'\x00' * 4
            
            message = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True
            )
            
            self.can_bus.send(message)
            
        except Exception as e:
            logger.error(f"Failed to set voltage for motor {motor_id}: {e}")
    
    def get_motor_status(self, motor_id: int) -> MotorStatus:
        """Get current status of a motor"""
        return self.motor_status.get(motor_id, MotorStatus())
    
    def stop_all_motors(self):
        """Emergency stop - set all motors to zero velocity"""
        logger.info("Stopping all motors")
        motor_config = self.config['MOTOR_CONFIG']
        for motor_name in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            if motor_name in motor_config:
                self.set_velocity(motor_config[motor_name], 0.0)
    
    def drive_robot(self, linear_vel: float, angular_vel: float):
        """
        Drive robot using differential drive kinematics
        
        Args:
            linear_vel: Forward velocity (m/s)
            angular_vel: Angular velocity (rad/s)
        """
        try:
            # Get wheel separation from config
            wheel_sep = self.config['MOTOR_CONFIG']['wheel_separation']
            wheel_diameter = self.config['MOTOR_CONFIG']['wheel_diameter']
            
            # Calculate wheel velocities (differential drive)
            left_vel_ms = linear_vel - (angular_vel * wheel_sep / 2.0)
            right_vel_ms = linear_vel + (angular_vel * wheel_sep / 2.0)
            
            # Convert m/s to RPM
            left_rpm = (left_vel_ms / (math.pi * wheel_diameter)) * 60
            right_rpm = (right_vel_ms / (math.pi * wheel_diameter)) * 60
            
            # Send commands to motors
            motor_config = self.config['MOTOR_CONFIG']
            self.set_velocity(motor_config['front_left'], left_rpm)
            self.set_velocity(motor_config['rear_left'], left_rpm)
            self.set_velocity(motor_config['front_right'], right_rpm)
            self.set_velocity(motor_config['rear_right'], right_rpm)
            
        except Exception as e:
            logger.error(f"Drive command failed: {e}")
    
    def drive_individual_wheels(self, fl: float, fr: float, rl: float, rr: float):
        """
        Drive individual wheels with specified velocities
        
        Args:
            fl, fr, rl, rr: Front-left, front-right, rear-left, rear-right velocities in m/s
        """
        try:
            wheel_diameter = self.config['MOTOR_CONFIG']['wheel_diameter']
            
            # Convert m/s to RPM
            fl_rpm = (fl / (math.pi * wheel_diameter)) * 60
            fr_rpm = (fr / (math.pi * wheel_diameter)) * 60
            rl_rpm = (rl / (math.pi * wheel_diameter)) * 60
            rr_rpm = (rr / (math.pi * wheel_diameter)) * 60
            
            motor_config = self.config['MOTOR_CONFIG']
            self.set_velocity(motor_config['front_left'], fl_rpm)
            self.set_velocity(motor_config['front_right'], fr_rpm)
            self.set_velocity(motor_config['rear_left'], rl_rpm)
            self.set_velocity(motor_config['rear_right'], rr_rpm)
            
        except Exception as e:
            logger.error(f"Individual wheel drive failed: {e}")
    
    def get_odometry(self) -> Tuple[float, float]:
        """
        Calculate robot odometry from wheel encoders
        Returns (linear_velocity, angular_velocity) in m/s and rad/s
        """
        try:
            motor_config = self.config['MOTOR_CONFIG']
            wheel_sep = motor_config['wheel_separation']
            wheel_diameter = motor_config['wheel_diameter']
            
            # Get wheel velocities from status
            left_status = self.get_motor_status(motor_config['front_left'])
            right_status = self.get_motor_status(motor_config['front_right'])
            
            # Convert RPM to m/s
            left_vel_ms = (left_status.velocity / 60) * (math.pi * wheel_diameter)
            right_vel_ms = (right_status.velocity / 60) * (math.pi * wheel_diameter)
            
            # Calculate robot velocities
            linear_vel = (left_vel_ms + right_vel_ms) / 2.0
            angular_vel = (right_vel_ms - left_vel_ms) / wheel_sep
            
            return linear_vel, angular_vel
            
        except Exception as e:
            logger.error(f"Odometry calculation failed: {e}")
            return 0.0, 0.0
    
    def shutdown(self):
        """Shutdown the motor controller"""
        logger.info("Shutting down motor controller")
        self.stop_all_motors()
        self.running = False
        
        if self.status_thread:
            self.status_thread.join(timeout=1.0)
        
        if self.can_bus:
            self.can_bus.shutdown()


# Test function
if __name__ == "__main__":
    from config import *
    
    # Test configuration
    config = {
        'MOTOR_CONFIG': MOTOR_CONFIG,
        'CAN_CONFIG': CAN_CONFIG
    }
    
    try:
        controller = CANMotorController(config)
        print("Motor controller initialized successfully!")
        
        # Test basic movement
        print("Testing forward movement...")
        controller.drive_robot(0.2, 0.0)  # 0.2 m/s forward
        time.sleep(2)
        
        print("Testing rotation...")
        controller.drive_robot(0.0, 0.5)  # 0.5 rad/s rotation
        time.sleep(2)
        
        print("Stopping...")
        controller.stop_all_motors()
        
        # Wait and shutdown
        time.sleep(1)
        controller.shutdown()
        
    except Exception as e:
        print(f"Test failed: {e}")
