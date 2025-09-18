#!/usr/bin/evn python3
"""
Stepper Motor Controller for Camera Pan/Tilt
Controls NEMA 17 stepper motor via GPIO for camera positioning
John Miller
"""

import time
import threading
import math
from typing import Optional, Callable
from dataclasses import dataclass
from enum import Enum
import logging

# GPIO lib
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
    logger.info("Jetson.GPIO library loaded successfully")
except ImportError:
    try:
        import RPi.GPIO as GPIO
        GPIO_AVAILABLE = True
        logger.info("RPi.GPIO library loaded (compatibility mode)")
    except ImportError:
        logger.error("No GPIO library available")
        logger.error("Install with: sudo apt install python3-jetson-gpio")
        GPIO_AVAILABLE = False

logger = logging.getLogger(__name__)

class MotorDirection(Enum):
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1

@dataclass
class StepperStatus:
    position_steps: int = 0
    position_degrees: float = 0.0
    target_position: int = 0
    is_moving: bool = False
    is_homed: bool = False
    speed_steps_per_sec: float = 0.0

class StepperMotorController:
    """
    Controls NEMA 17 stepper motor for camera pan/tilt
    Supports position control, speed control, and homing
    """
    
    def __init__(self, config: dict, step_pin: int, dir_pin: int, enable_pin: Optional[int] = None):
        """
        Initialize stepper motor controller
        
        Args:
            config: Configuration dictionary
            step_pin: GPIO pin for step control
            dir_pin: GPIO pin for direction control  
            enable_pin: GPIO pin for motor enable
        """
        self.config = config['CAMERA_CONFIG']
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        
        # Motor parameters
        self.steps_per_rev = self.config['stepper_steps_per_rev'] * self.config['stepper_microsteps']
        self.degrees_per_step = 360.0 / self.steps_per_rev
        self.max_speed = self.config['camera_rotation_speed']  # degrees/second
        self.max_angle = self.config['camera_max_angle']  # max rotation from center
        
        # Current state
        self.status = StepperStatus()
        self.current_position_steps = 0
        self.home_position_steps = 0
        self.is_enabled = False
        self.move_thread = None
        self.stop_movement = False
        
        # Movement callbacks
        self.on_move_complete = None
        self.on_position_changed = None
        
        # Init GPIO
        self.initialize_gpio()
        
        logger.info(f"Stepper controller initialized: {self.steps_per_rev} steps/rev, "
                   f"{self.degrees_per_step:.3f} degrees/step")
    
    def initialize_gpio(self):
        """Initialize GPIO pins for stepper control"""
        if not GPIO_AVAILABLE:
            logger.warning("GPIO not available - running in simulation mode")
            return
            
        try:
            # Set GPIO mode
            GPIO.setmode(GPIO.BOARD)
            
            # Setup pins
            GPIO.setup(self.step_pin, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
            
            if self.enable_pin:
                GPIO.setup(self.enable_pin, GPIO.OUT, initial=GPIO.HIGH)  # Usually active low
            
            logger.info(f"GPIO initialized - Step: {self.step_pin}, Dir: {self.dir_pin}, Enable: {self.enable_pin}")
            
        except Exception as e:
            logger.error(f"Failed to initialize GPIO: {e}")
            raise
    
    def enable_motor(self, enable: bool = True):
        """Enable or disable the stepper motor"""
        if not GPIO_AVAILABLE:
            self.is_enabled = enable
            return
            
        if self.enable_pin:
            # Also usually active low
            GPIO.output(self.enable_pin, GPIO.LOW if enable else GPIO.HIGH)
            
        self.is_enabled = enable
        logger.info(f"Motor {'enabled' if enable else 'disabled'}")
    
    def set_direction(self, direction: MotorDirection):
        """Set motor rotation direction"""
        if not GPIO_AVAILABLE:
            return
            
        if direction == MotorDirection.CLOCKWISE:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
    
    def step_once(self, pulse_width: float = 0.000002):  # 2 microseconds
        """Execute a single step pulse with precise timing"""
        if not GPIO_AVAILABLE:
            logger.error("Cannot execute step - GPIO not available")
            return
            
        # Generate precise step pulse
        GPIO.output(self.step_pin, GPIO.HIGH)
        time.sleep(pulse_width)
        GPIO.output(self.step_pin, GPIO.LOW)
        time.sleep(pulse_width)  # Minimum low time
    
    def move_steps(self, steps: int, speed_degrees_per_sec: Optional[float] = None):
        """
        Move a specific number of steps
        
        Args:
            steps: Number of steps to move (positive = clockwise)
            speed_degrees_per_sec: Speed in degrees per second (uses max if None)
        """
        if not self.is_enabled:
            logger.warning("Motor not enabled - cannot move")
            return False
            
        if self.status.is_moving:
            logger.warning("Motor already moving - stop current movement first")
            return False
        
        # move params
        direction = MotorDirection.CLOCKWISE if steps > 0 else MotorDirection.COUNTERCLOCKWISE
        abs_steps = abs(steps)
        
        if speed_degrees_per_sec is None:
            speed_degrees_per_sec = self.max_speed
            
        # Convert speed to steps per second
        steps_per_second = speed_degrees_per_sec / self.degrees_per_step
        step_delay = 1.0 / steps_per_second if steps_per_second > 0 else 0.001
        
        # Check pos limits
        new_position = self.current_position_steps + steps
        new_angle = new_position * self.degrees_per_step
        
        if abs(new_angle) > self.max_angle:
            logger.warning(f"Move would exceed limits: {new_angle:.1f}° (max: ±{self.max_angle}°)")
            return False
        
        # Start movement in separate thread
        self.stop_movement = False
        self.move_thread = threading.Thread(
            target=self._move_steps_blocking,
            args=(direction, abs_steps, step_delay)
        )
        self.move_thread.start()
        
        return True
    
    def _move_steps_blocking(self, direction: MotorDirection, steps: int, step_delay: float):
        """Execute movement in blocking manner (runs in separate thread)"""
        self.status.is_moving = True
        self.status.speed_steps_per_sec = 1.0 / step_delay
        
        try:
            # Set direction
            self.set_direction(direction)
            time.sleep(0.001)  # delay after direction change
            
            # Execute steps
            for i in range(steps):
                if self.stop_movement:
                    logger.info("Movement stopped by user")
                    break
                    
                self.step_once()
                time.sleep(step_delay)
                
                # Update position
                self.current_position_steps += direction.value
                self.status.position_steps = self.current_position_steps
                self.status.position_degrees = self.current_position_steps * self.degrees_per_step
                
                # Callback for pos updates
                if self.on_position_changed:
                    self.on_position_changed(self.status.position_degrees)
            
        finally:
            self.status.is_moving = False
            self.status.speed_steps_per_sec = 0.0
            
            # Movement complete callback
            if self.on_move_complete:
                self.on_move_complete(self.status.position_degrees)
        
        logger.info(f"Movement complete - Position: {self.status.position_degrees:.1f}°")
    
    def move_to_angle(self, target_degrees: float, speed_degrees_per_sec: Optional[float] = None):
        """
        Move to absolute angle position
        
        Args:
            target_degrees: Target angle in degrees (0 = center)
            speed_degrees_per_sec: Movement speed
        """
        # Check limits
        if abs(target_degrees) > self.max_angle:
            logger.warning(f"Target angle {target_degrees:.1f}° exceeds limits (±{self.max_angle}°)")
            return False
        
        # Calculate required steps
        target_steps = int(target_degrees / self.degrees_per_step)
        steps_to_move = target_steps - self.current_position_steps
        
        if steps_to_move == 0:
            logger.info("Already at target position")
            return True
        
        self.status.target_position = target_steps
        
        logger.info(f"Moving to {target_degrees:.1f}° ({steps_to_move} steps)")
        return self.move_steps(steps_to_move, speed_degrees_per_sec)
    
    def home_motor(self, home_speed: float = 10.0):
        """
        Home the motor to center position (0 degrees)
        This assumes center is the home position
        """
        logger.info("Starting homing sequence...")
        
        # Move to center position
        if self.move_to_angle(0.0, home_speed):
            # Wait for movement to complete
            while self.status.is_moving:
                time.sleep(0.1)
            
            # Set current pos as home
            self.current_position_steps = 0
            self.home_position_steps = 0
            self.status.position_steps = 0
            self.status.position_degrees = 0.0
            self.status.is_homed = True
            
            logger.info("Homing complete - Motor at center position")
            return True
        
        return False
    
    def stop_movement(self):
        """Stop current movement immediately"""
        self.stop_movement = True
        
        if self.move_thread and self.move_thread.is_alive():
            self.move_thread.join(timeout=2.0)
        
        logger.info("Movement stopped")
    
    def get_position_degrees(self) -> float:
        """Get current position in degrees"""
        return self.current_position_steps * self.degrees_per_step
    
    def get_status(self) -> StepperStatus:
        """Get current motor status"""
        self.status.position_steps = self.current_position_steps
        self.status.position_degrees = self.get_position_degrees()
        return self.status
    
    def scan_for_targets(self, start_angle: float = -90, end_angle: float = 90, 
                        scan_speed: float = 30.0, callback: Optional[Callable] = None):
        """
        Perform scanning motion between two angles
        Useful for searching for ArUco tags
        
        Args:
            start_angle: Starting angle for scan
            end_angle: Ending angle for scan  
            scan_speed: Scanning speed in degrees/second
            callback: Optional callback function called at each position
        """
        logger.info(f"Starting scan from {start_angle}° to {end_angle}°")
        
        # Clamp angles to limits
        start_angle = max(-self.max_angle, min(self.max_angle, start_angle))
        end_angle = max(-self.max_angle, min(self.max_angle, end_angle))
        
        # Move to start position
        if not self.move_to_angle(start_angle, scan_speed):
            return False
        
        # Wait for movement to complete
        while self.status.is_moving:
            time.sleep(0.1)
            if callback:
                callback(self.get_position_degrees())
        
        # Scan to end position
        return self.move_to_angle(end_angle, scan_speed)
    
    def shutdown(self):
        """Shutdown the stepper controller"""
        logger.info("Shutting down stepper motor controller")
        
        # Stop any moves
        self.stop_movement()
        
        # Disable motor
        self.enable_motor(False)
        
        # Cleanup GPIO
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup([self.step_pin, self.dir_pin])
                if self.enable_pin:
                    GPIO.cleanup([self.enable_pin])
            except Exception as e:
                logger.warning(f"GPIO cleanup warning: {e}")


# Test
if __name__ == "__main__":
    from config import CAMERA_CONFIG
    
    # Test config
    config = {'CAMERA_CONFIG': CAMERA_CONFIG}
    
    # GPIO pin assignments (will need to adjust for orin)
    STEP_PIN = 18  # Board pin 12
    DIR_PIN = 16   # Board pin 10  
    ENABLE_PIN = 22  # Board pin 15
    
    try:
        # Init controller
        stepper = StepperMotorController(config, STEP_PIN, DIR_PIN, ENABLE_PIN)
        stepper.enable_motor(True)
        
        print("Stepper motor controller test starting...")
        
        # Home
        stepper.home_motor()
        time.sleep(2)
        
        # Test moves
        print("Moving to +45 degrees...")
        stepper.move_to_angle(45.0)
        time.sleep(3)
        
        print("Moving to -45 degrees...")  
        stepper.move_to_angle(-45.0)
        time.sleep(3)
        
        print("Returning to center...")
        stepper.move_to_angle(0.0)
        time.sleep(2)
        
        # Test scanning
        print("Performing scan...")
        stepper.scan_for_targets(-60, 60, 20.0)
        time.sleep(5)
        
        # Return to center and shutdown
        stepper.move_to_angle(0.0)
        time.sleep(2)
        stepper.shutdown()
        
        print("Test completed successfully!")
        
    except Exception as e:
        print(f"Test failed: {e}")
        if 'stepper' in locals():
            stepper.shutdown()
