#!/usr/bin/env python3
"""
PWM Actuator Controller for Firgelli Linear Actuators
Controls linear actuators via PWM for bucket lift/tilt operations
John Miller
"""

import time
import threading
from typing import Optional, Dict, Callable
from dataclasses import dataclass
from enum import Enum
import logging

# GPIO lib
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    try:
        import RPi.GPIO as GPIO
        GPIO_AVAILABLE = True
    except ImportError:
        print("Warning: No GPIO library available - running in sim")
        GPIO_AVAILABLE = False

logger = logging.getLogger(__name__)

class ActuatorDirection(Enum):
    EXTEND = 1
    RETRACT = -1
    STOP = 0

@dataclass
class ActuatorStatus:
    position_percent: float = 50.0  # 0-100% (0=fully retracted, 100=fully extended)
    target_position: float = 50.0
    is_moving: bool = False
    pwm_value: int = 1500  # Current PWM microseconds
    direction: ActuatorDirection = ActuatorDirection.STOP

class PWMActuatorController:
    """
    Controls Firgelli linear actuators using PWM
    Supports position control, speed control, and safety limits
    """
    
    def __init__(self, config: dict, actuator_name: str):
        """
        Init PWM actuator controller
        
        Args:
            config: Configuration dictionary containing actuator settings
            actuator_name: Name of actuator ('bucket_lift' or 'bucket_tilt')
        """
        if actuator_name not in config['ACTUATOR_CONFIG']:
            raise ValueError(f"Actuator '{actuator_name}' not found in configuration")
        
        self.name = actuator_name
        self.config = config['ACTUATOR_CONFIG'][actuator_name]
        self.pin = self.config['pwm_pin']
        
        # PWM parameters
        self.min_pwm = self.config['min_pwm']  # Microseconds for full retract
        self.max_pwm = self.config['max_pwm']  # Microseconds for full extend  
        self.neutral_pwm = self.config['neutral_pwm']  # Microseconds for stop
        self.pwm_range = self.max_pwm - self.min_pwm
        
        # Control parameters
        self.max_speed = 50.0  # Max speed as % per second
        self.position_tolerance = 2.0  # Position tolerance in %
        self.safety_timeout = 10.0  # Max time for any single movement
        
        # Current state
        self.status = ActuatorStatus()
        self.pwm_instance = None
        self.move_thread = None
        self.stop_movement = False
        self.last_command_time = time.time()
        
        # Safety and callbacks
        self.position_limits = (0.0, 100.0)  # Min/max position limits
        self.on_position_changed = None
        self.on_limit_reached = None
        
        # Initialize PWM
        self.initialize_pwm()
        
        logger.info(f"Actuator '{actuator_name}' initialized on pin {self.pin}")
    
    def initialize_pwm(self):
        """Initialize PWM output for actuator control"""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO not available - running in simulation mode")
            return
            
        try:
            # Set GPIO mode and pin
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.pin, GPIO.OUT)
            
            # Create PWM instance
            self.pwm_instance = GPIO.PWM(self.pin, 50)  # 50Hz
            
            # Start PWM with neutral position
            duty_cycle = self.microseconds_to_duty_cycle(self.neutral_pwm)
            self.pwm_instance.start(duty_cycle)
            
            self.status.pwm_value = self.neutral_pwm
            
            logger.info(f"PWM initialized on pin {self.pin} at 50Hz")
            
        except Exception as e:
            logger.error(f"Failed to initialize PWM: {e}")
            raise
    
    def microseconds_to_duty_cycle(self, microseconds: int) -> float:
        """
        Convert PWM pulse width in microseconds to duty cycle percentage
        For 50Hz PWM: 20ms period, so 1500µs = 7.5% duty cycle
        """
        period_us = 20000  # 20ms in microseconds
        return (microseconds / period_us) * 100.0
    
    def set_pwm_microseconds(self, microseconds: int):
        """Set PWM output in microseconds"""
        # Clamp to safe range
        microseconds = max(self.min_pwm, min(self.max_pwm, microseconds))
        
        if GPIO_AVAILABLE and self.pwm_instance:
            duty_cycle = self.microseconds_to_duty_cycle(microseconds)
            self.pwm_instance.ChangeDutyCycle(duty_cycle)
        
        self.status.pwm_value = microseconds
        self.last_command_time = time.time()
    
    def set_position_percent(self, position_percent: float, speed_percent_per_sec: Optional[float] = None):
        """
        Set actuator position as percentage (0=retracted, 100=extended)
        
        Args:
            position_percent: Target position (0-100%)
            speed_percent_per_sec: Movement speed (% per second)
        """
        # Clamp position to limits
        position_percent = max(self.position_limits[0], 
                              min(self.position_limits[1], position_percent))
        
        if self.status.is_moving:
            logger.warning("Actuator already moving - stopping current movement")
            self.stop_movement_now()
        
        self.status.target_position = position_percent
        
        # If already at target, do nothing
        if abs(self.status.position_percent - position_percent) < self.position_tolerance:
            logger.info(f"Already at target position {position_percent:.1f}%")
            return True
        
        # Use default speed if not specified
        if speed_percent_per_sec is None:
            speed_percent_per_sec = self.max_speed
        
        # Start movement in separate thread
        self.stop_movement = False
        self.move_thread = threading.Thread(
            target=self._move_to_position_blocking,
            args=(position_percent, speed_percent_per_sec)
        )
        self.move_thread.start()
        
        logger.info(f"Moving to {position_percent:.1f}% at {speed_percent_per_sec:.1f}%/sec")
        return True
    
    def _move_to_position_blocking(self, target_percent: float, speed_percent_per_sec: float):
        """Execute position movement (runs in separate thread)"""
        self.status.is_moving = True
        start_time = time.time()
        
        try:
            while not self.stop_movement:
                # Check safety timeout
                if time.time() - start_time > self.safety_timeout:
                    logger.warning("Movement timeout reached - stopping")
                    break
                
                # Calculate current error
                error = target_percent - self.status.position_percent
                
                # Check if we've reached the target
                if abs(error) < self.position_tolerance:
                    logger.info(f"Target position {target_percent:.1f}% reached")
                    break
                
                # Determine movement direction and speed
                if error > 0:
                    direction = ActuatorDirection.EXTEND
                else:
                    direction = ActuatorDirection.RETRACT
                
                # Calculate PWM value based on desired speed
                if direction == ActuatorDirection.EXTEND:
                    pwm_value = self.neutral_pwm + int(self.pwm_range * 0.3)  # 30% towards extend
                else:
                    pwm_value = self.neutral_pwm - int(self.pwm_range * 0.3)  # 30% towards retract
                
                # Set PWM output
                self.set_pwm_microseconds(pwm_value)
                self.status.direction = direction
                
                # Update position from actuator feedback
                # use built-in position feedback via current sensing
                # or potentiometer
                dt = 0.05  # 50ms update rate
                
                # Read actual position from actuator
                try:
                    # Current method: monitor PWM response and current draw
                    position_change = speed_percent_per_sec * dt * direction.value
                    self.status.position_percent += position_change
                    
                    # Clamp position to valid range
                    self.status.position_percent = max(0, min(100, self.status.position_percent))
                    
                    # Potentiometer usage below
                    # actual_position = self.read_position_feedback()
                    # if actual_position is not None:
                    #     self.status.position_percent = actual_position
                    
                except Exception as e:
                    logger.warning(f"Position feedback error: {e}")
                    # Fallback to estimated position
                    pass
                
                # Position change callback
                if self.on_position_changed:
                    self.on_position_changed(self.status.position_percent)
                
                time.sleep(dt)
            
        finally:
            # Stop movement
            self.set_pwm_microseconds(self.neutral_pwm)
            self.status.is_moving = False
            self.status.direction = ActuatorDirection.STOP
            
        logger.info(f"Movement complete - Final position: {self.status.position_percent:.1f}%")
    
    def extend(self, speed_percent: float = 100.0):
        """Extend actuator at specified speed"""
        pwm_value = self.neutral_pwm + int(self.pwm_range * (speed_percent / 100.0) * 0.5)
        self.set_pwm_microseconds(pwm_value)
        self.status.direction = ActuatorDirection.EXTEND
        logger.info(f"Extending at {speed_percent:.1f}% speed")
    
    def retract(self, speed_percent: float = 100.0):
        """Retract actuator at specified speed"""
        pwm_value = self.neutral_pwm - int(self.pwm_range * (speed_percent / 100.0) * 0.5)
        self.set_pwm_microseconds(pwm_value)
        self.status.direction = ActuatorDirection.RETRACT
        logger.info(f"Retracting at {speed_percent:.1f}% speed")
    
    def stop(self):
        """Stop actuator movement"""
        self.set_pwm_microseconds(self.neutral_pwm)
        self.status.direction = ActuatorDirection.STOP
        logger.info("Actuator stopped")
    
    def stop_movement_now(self):
        """Stop any position movement"""
        self.stop_movement = True
        self.stop()
        
        if self.move_thread and self.move_thread.is_alive():
            self.move_thread.join(timeout=1.0)
    
    def home_actuator(self, home_position: float = 50.0):
        """Move actuator to home position"""
        logger.info(f"Homing actuator to {home_position:.1f}%")
        return self.set_position_percent(home_position, self.max_speed * 0.5)
    
    def get_status(self) -> ActuatorStatus:
        """Get current actuator status"""
        return self.status
    
    def set_position_limits(self, min_percent: float, max_percent: float):
        """Set software position limits"""
        self.position_limits = (min_percent, max_percent)
        logger.info(f"Position limits set: {min_percent:.1f}% - {max_percent:.1f}%")
    
    def read_position_feedback(self) -> Optional[float]:
        """
        Read position feedback from actuator
        Returns:
            Position as percentage (0-100%) or None if no feedback available
        """
        try:
            # Potentiometer implement
            
            # Need imports:
            # import board
            # import busio
            # import adafruit_ads1x15.ads1015 as ADS
            # from adafruit_ads1x15.analog_in import AnalogIn
            return None
            
        except Exception as e:
            logger.debug(f"Position feedback read error: {e}")
            return None
    
    def calibrate_position_feedback(self):
        """
        Calibrate the position feedback system
        This should be run when the actuator is at known positions
        """
        logger.info(f"Calibrating position feedback for '{self.name}'...")
        
        # Move to fully retracted position and record feedback
        logger.info("Move actuator to fully RETRACTED position and press Enter...")
        input("Press Enter when actuator is fully retracted...")
        
        retracted_feedback = self.read_position_feedback()
        if retracted_feedback is not None:
            logger.info(f"Retracted position feedback: {retracted_feedback}")
        
        # Move to fully extended position and record feedback  
        logger.info("Move actuator to fully EXTENDED position and press Enter...")
        input("Press Enter when actuator is fully extended...")
        
        extended_feedback = self.read_position_feedback()
        if extended_feedback is not None:
            logger.info(f"Extended position feedback: {extended_feedback}")
        
        # Store calibration values
        # These would be saved to configuration file in real system
        logger.info("Position feedback calibration complete")
        
        return retracted_feedback, extended_feedback
    def calibrate_physical_positions(self):
        """
        Calibrate actuator positions by moving to extremes
        IMPORTANT: Supervise this process to prevent damage
        """
        logger.info("Starting PHYSICAL position calibration - SUPERVISE MANUALLY")
        logger.warning("Ensure actuator has clear range of motion!")
        
        # Move to fully retracted position slowly
        logger.info("Moving to fully retracted position...")
        self.set_position_percent(0.0, 20.0)  # Slow speed for safety
        
        while self.status.is_moving:
            time.sleep(0.5)
        
        time.sleep(2)  # Allow settling
        logger.info("Retracted position reached")
        
        # Move to fully extended position  
        logger.info("Moving to fully extended position...")
        self.set_position_percent(100.0, 20.0)
        
        while self.status.is_moving:
            time.sleep(0.5)
        
        time.sleep(2)  # Allow settling
        logger.info("Extended position reached")
        
        # Return to center
        logger.info("Returning to center position...")
        self.set_position_percent(50.0, 30.0)
        
        while self.status.is_moving:
            time.sleep(0.5)
        
        logger.info("Physical calibration complete - verify all positions manually")
    
    def shutdown(self):
        """Shutdown the actuator controller"""
        logger.info(f"Shutting down actuator '{self.name}'")
        
        # Stop any movement
        self.stop_movement_now()
        
        # Set to neutral position
        self.stop()
        
        # Cleanup PWM
        if GPIO_AVAILABLE and self.pwm_instance:
            self.pwm_instance.stop()
            GPIO.cleanup([self.pin])


class BucketController:
    """
    High-level controller for bucket operations
    Combines lift and tilt actuators
    """
    
    def __init__(self, config: dict):
        """Initialize bucket controller with lift and tilt actuators"""
        self.lift_actuator = PWMActuatorController(config, 'bucket_lift')
        self.tilt_actuator = PWMActuatorController(config, 'bucket_tilt')
        
        # Digging operation positions (Will need to change based on found values
        self.positions = {
            'transport': {'lift': 80.0, 'tilt': 45.0},  # Raised for transport
            'digging': {'lift': 20.0, 'tilt': 75.0},    # Low and tilted for digging
            'dumping': {'lift': 60.0, 'tilt': 10.0},    # Medium height, tilted to dump
            'neutral': {'lift': 50.0, 'tilt': 50.0},    # Neutral position
        }
        
        logger.info("Bucket controller initialized with lift and tilt actuators")
    
    def move_to_position(self, position_name: str, speed: float = 30.0):
        """Move bucket to predefined position"""
        if position_name not in self.positions:
            logger.error(f"Unknown position: {position_name}")
            return False
        
        pos = self.positions[position_name]
        logger.info(f"Moving bucket to '{position_name}' position")
        
        # Move both actuators simultaneously
        self.lift_actuator.set_position_percent(pos['lift'], speed)
        self.tilt_actuator.set_position_percent(pos['tilt'], speed)
        
        return True
    
    def wait_for_movement_complete(self, timeout: float = 10.0):
        """Wait for both actuators to complete movement"""
        start_time = time.time()
        
        while (self.lift_actuator.status.is_moving or self.tilt_actuator.status.is_moving):
            if time.time() - start_time > timeout:
                logger.warning("Movement timeout reached")
                return False
            time.sleep(0.1)
        
        return True
    
    def perform_dig_cycle(self):
        """Perform a complete digging cycle"""
        logger.info("Starting dig cycle")
        
        # 1. Move to digging position
        self.move_to_position('digging')
        self.wait_for_movement_complete()
        
        # 2. Push forward (this would be done by the rover movement)
        logger.info("Ready for forward push")
        
        # 3. Tilt back to scoop material
        self.tilt_actuator.set_position_percent(85.0, 20.0)
        self.wait_for_movement_complete()
        
        # 4. Raise bucket for transport
        self.move_to_position('transport')
        self.wait_for_movement_complete()
        
        logger.info("Dig cycle complete - bucket ready for transport")
    
    def perform_dump_cycle(self):
        """Perform dumping operation"""
        logger.info("Starting dump cycle")
        
        # 1. Move to dump position
        self.move_to_position('dumping')
        self.wait_for_movement_complete()
        
        # 2. Wait for material to fall out
        time.sleep(2.0)
        
        # 3. Shake bucket
        for _ in range(3):
            self.tilt_actuator.set_position_percent(5.0, 50.0)
            time.sleep(0.5)
            self.tilt_actuator.set_position_percent(15.0, 50.0)
            time.sleep(0.5)
        
        # 4. Return to transport position
        self.move_to_position('transport')
        self.wait_for_movement_complete()
        
        logger.info("Dump cycle complete")
    
    def emergency_stop(self):
        """Emergency stop for both actuators"""
        logger.warning("EMERGENCY STOP - Stopping all actuators")
        self.lift_actuator.stop_movement_now()
        self.tilt_actuator.stop_movement_now()
    
    def home_bucket(self):
        """Home bucket to neutral position"""
        logger.info("Homing bucket to neutral position")
        self.move_to_position('neutral')
        return self.wait_for_movement_complete()
    
    def shutdown(self):
        """Shutdown bucket controller"""
        logger.info("Shutting down bucket controller")
        self.emergency_stop()
        self.lift_actuator.shutdown()
        self.tilt_actuator.shutdown()


# Test
if __name__ == "__main__":
    from config import ACTUATOR_CONFIG
    
    # Test config
    config = {'ACTUATOR_CONFIG': ACTUATOR_CONFIG}
    
    try:
        print("Testing individual actuator...")
        
        actuator = PWMActuatorController(config, 'bucket_lift')
        
        print("Moving to 25%...")
        actuator.set_position_percent(25.0)
        time.sleep(3)
        
        print("Moving to 75%...")
        actuator.set_position_percent(75.0)
        time.sleep(3)
        
        print("Returning to center...")
        actuator.set_position_percent(50.0)
        time.sleep(2)
        
        actuator.shutdown()
        
        print("\nTesting bucket controller...")
        
        bucket = BucketController(config)
        
        print("Homing bucket...")
        bucket.home_bucket()
        
        print("Performing dig cycle...")
        bucket.perform_dig_cycle()
        
        time.sleep(1)
        
        print("Performing dump cycle...")
        bucket.perform_dump_cycle()
        
        bucket.shutdown()
        print("Test completed successfully!")
        
    except Exception as e:
        print(f"Test failed: {e}")
        if 'bucket' in locals():
            bucket.shutdown()
        if 'actuator' in locals():
            actuator.shutdown()
