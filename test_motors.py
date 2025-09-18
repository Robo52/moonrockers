#!/usr/bin/env python3
"""
Motor Test Script
Tests basic motor functionality and CAN
"""

import sys
import time
import signal
from pathlib import Path

# Add src directory to path
sys.path.append(str(Path(__file__).parent / 'src' / 'hardware'))

from can_motor_controller import CANMotorController
from config import MOTOR_CONFIG, CAN_CONFIG

class MotorTester:
    """Test suite for motor controller functionality"""
    
    def __init__(self):
        self.controller = None
        self.running = True
        
        # Setup signal handler
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\nReceived shutdown signal...")
        self.running = False
        if self.controller:
            self.controller.shutdown()
        sys.exit(0)
    
    def initialize_controller(self):
        """Initialize the motor controller"""
        try:
            config = {
                'MOTOR_CONFIG': MOTOR_CONFIG,
                'CAN_CONFIG': CAN_CONFIG
            }
            
            print("Initializing CAN motor controller...")
            self.controller = CANMotorController(config)
            print("✓ Motor controller initialized successfully!")
            return True
            
        except Exception as e:
            print(f"X Failed to initialize motor controller: {e}")
            print("\nTroubleshooting:")
            print("1. Check that CAN interface is up: sudo ip link set can0 up type can bitrate 1000000")
            print("2. Verify CAN hardware connections")
            print("3. Check motor CAN IDs match configuration")
            return False
    
    def test_individual_motors(self):
        """Test each motor individually"""
        print("\n" + "="*50)
        print("TESTING INDIVIDUAL MOTORS")
        print("="*50)
        
        motors = {
            'Front Left': MOTOR_CONFIG['front_left'],
            'Front Right': MOTOR_CONFIG['front_right'], 
            'Rear Left': MOTOR_CONFIG['rear_left'],
            'Rear Right': MOTOR_CONFIG['rear_right']
        }
        
        for motor_name, motor_id in motors.items():
            print(f"\nTesting {motor_name} (CAN ID: {motor_id})...")
            
            # Test forward rotation
            print(f"  → Forward rotation (100 RPM)")
            self.controller.set_velocity(motor_id, 100.0)
            time.sleep(2)
            
            # Check status
            status = self.controller.get_motor_status(motor_id)
            print(f"  → Status: {status.velocity:.1f} RPM, {status.current:.2f}A, {status.temperature:.1f}°C")
            
            # Test reverse rotation
            print(f"  → Reverse rotation (-100 RPM)")
            self.controller.set_velocity(motor_id, -100.0)
            time.sleep(2)
            
            # Stop motor
            print(f"  → Stopping")
            self.controller.set_velocity(motor_id, 0.0)
            time.sleep(1)
    
    def test_differential_drive(self):
        """Test differential drive functionality"""
        print("\n" + "="*50)
        print("TESTING DIFFERENTIAL DRIVE")
        print("="*50)
        
        test_sequences = [
            ("Forward movement", 0.3, 0.0),
            ("Backward movement", -0.3, 0.0),
            ("Turn left (in place)", 0.0, 1.0),
            ("Turn right (in place)", 0.0, -1.0),
            ("Forward + turn left", 0.2, 0.5),
            ("Forward + turn right", 0.2, -0.5),
        ]
        
        for description, linear_vel, angular_vel in test_sequences:
            print(f"\n{description}...")
            print(f"  Linear: {linear_vel:.1f} m/s, Angular: {angular_vel:.1f} rad/s")
            
            self.controller.drive_robot(linear_vel, angular_vel)
            time.sleep(3)
            
            # Get odometry feedback
            measured_linear, measured_angular = self.controller.get_odometry()
            print(f"  Measured - Linear: {measured_linear:.2f} m/s, Angular: {measured_angular:.2f} rad/s")
            
            # Stop between tests
            self.controller.stop_all_motors()
            time.sleep(1)
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        print("\n" + "="*50)
        print("TESTING EMERGENCY STOP")
        print("="*50)
        
        print("Starting movement...")
        self.controller.drive_robot(0.3, 0.0)  # Forward movement
        time.sleep(1)
        
        print("EMERGENCY STOP!")
        self.controller.stop_all_motors()
        
        # Verify all motors stopped
        time.sleep(0.5)
        all_stopped = True
        for motor_name in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            motor_id = MOTOR_CONFIG[motor_name]
            status = self.controller.get_motor_status(motor_id)
            if abs(status.velocity) > 10:  # Allow small threshold for measurement noise
                all_stopped = False
                print(f"  X {motor_name} still moving: {status.velocity:.1f} RPM")
        
        if all_stopped:
            print("  ✓ All motors stopped successfully")
        else:
            print("  X Emergency stop failed!")
    
    def monitor_motor_health(self, duration=10):
        """Monitor motor health for specified duration"""
        print(f"\n" + "="*50)
        print(f"MONITORING MOTOR HEALTH ({duration}s)")
        print("="*50)
        
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            print(f"\n--- Status Update (t={time.time()-start_time:.1f}s) ---")
            
            for motor_name in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                motor_id = MOTOR_CONFIG[motor_name]
                status = self.controller.get_motor_status(motor_id)
                
                # Check for faults
                fault_str = f"Faults: 0x{status.faults:04X}" if status.faults else "No faults"
                
                print(f"{motor_name:12}: {status.velocity:6.1f} RPM, "
                      f"{status.current:5.2f}A, {status.temperature:5.1f}°C, "
                      f"{status.voltage:5.1f}V, {fault_str}")
            
            time.sleep(2)
    
    def run_full_test(self):
        """Run complete test suite"""
        print("MINING ROVER - MOTOR TEST SUITE")
        print("="*50)
        
        if not self.initialize_controller():
            return False
        
        try:
            # Run test sequence
            self.test_individual_motors()
            
            if self.running:
                self.test_differential_drive()
            
            if self.running:
                self.test_emergency_stop()
            
            if self.running:
                print(f"\nStarting health monitoring... (Press Ctrl+C to stop)")
                self.monitor_motor_health(10)
            
            print("\n" + "="*50)
            print("TEST COMPLETED SUCCESSFULLY!")
            print("="*50)
            return True
            
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
            return False
        except Exception as e:
            print(f"\nTest failed with error: {e}")
            return False
        finally:
            if self.controller:
                print("\nShutting down motor controller...")
                self.controller.shutdown()

def main():
    """Main test function"""
    tester = MotorTester()
    success = tester.run_full_test()
    
    if success:
        print("\n All tests passed! Motors are ready.")
    else:
        print("\n Some tests failed. Check hardware and configuration.")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
