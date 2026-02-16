#!/usr/bin/env python3
"""
Sabertooth 2x12 Test Code for Raspberry Pi 4
GPIO 17 -> S1 (Motor 1 control)
GPIO 27 -> S2 (Motor 2 control)

DIP Switch Configuration:
- Switch 1: DOWN (R/C mode)
- Switch 2: UP (R/C mode)
- Switch 3: DOWN (Lithium cutoff enabled)
- Switch 4: DOWN (Independent mode)
- Switch 5: UP (Linear response)
- Switch 6: DOWN (Microcontroller mode)

Modified to read joystick commands and map to motor control
"""

import pigpio
import time
import re
import os

# GPIO Pin Configuration
MOTOR1_PIN = 13  # S1 input
MOTOR2_PIN = 26  # S2 input

# Pulse width ranges (microseconds)
PULSE_MIN = 1000   # Full reverse
PULSE_NEUTRAL = 1500  # Stop
PULSE_MAX = 2000   # Full forward

# Deadband around neutral to prevent drift (adjust if needed)
DEADBAND = 10  # microseconds

# Joystick log file
JOYSTICK_LOG_FILE = "numbers.txt"

# Joystick threshold for detecting meaningful input
JOYSTICK_THRESHOLD = 0.15


class Sabertooth2x12:
    """Driver class for Sabertooth 2x12 in R/C Microcontroller Mode using pigpio"""
    
    def __init__(self, motor1_pin=MOTOR1_PIN, motor2_pin=MOTOR2_PIN):
        """Initialize the motor controller"""
        self.motor1_pin = motor1_pin
        self.motor2_pin = motor2_pin
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise Exception("Failed to connect to pigpio daemon. Run 'sudo pigpiod' first.")
        
        # Set GPIO modes
        self.pi.set_mode(self.motor1_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.motor2_pin, pigpio.OUTPUT)
        
        # Calibration offsets (can be adjusted if motors drift)
        self.motor1_neutral_offset = 0
        self.motor2_neutral_offset = 0
        
        # Start with neutral pulses
        self.pi.set_servo_pulsewidth(self.motor1_pin, PULSE_NEUTRAL)
        self.pi.set_servo_pulsewidth(self.motor2_pin, PULSE_NEUTRAL)
        
        print("Sabertooth 2x12 initialized with pigpio")
        print(f"Motor 1 on GPIO {self.motor1_pin}")
        print(f"Motor 2 on GPIO {self.motor2_pin}")
        time.sleep(0.5)
    
    def _speed_to_pulse(self, speed, neutral_offset=0):
        """
        Convert speed (-100 to 100) to pulse width in microseconds
        -100 = full reverse, 0 = stop, 100 = full forward
        """
        # Clamp speed to valid range
        speed = max(-100, min(100, speed))
        
        # Apply deadband for speeds near zero
        if abs(speed) < 2:
            return PULSE_NEUTRAL + neutral_offset
        
        # Map speed to pulse width
        if speed >= 0:
            # Forward: 1500us to 2000us
            pulse = PULSE_NEUTRAL + neutral_offset + (speed / 100.0) * (PULSE_MAX - PULSE_NEUTRAL)
        else:
            # Reverse: 1000us to 1500us
            pulse = PULSE_NEUTRAL + neutral_offset + (speed / 100.0) * (PULSE_NEUTRAL - PULSE_MIN)
        
        # Clamp to valid range
        pulse = max(PULSE_MIN, min(PULSE_MAX, pulse))
        
        return int(pulse)
    
    def set_motor1(self, speed):
        """
        Set Motor 1 speed
        speed: -100 (full reverse) to 100 (full forward), 0 = stop
        """
        pulse = self._speed_to_pulse(speed, self.motor1_neutral_offset)
        self.pi.set_servo_pulsewidth(self.motor1_pin, pulse)
    
    def set_motor2(self, speed):
        """
        Set Motor 2 speed
        speed: -100 (full reverse) to 100 (full forward), 0 = stop
        """
        pulse = self._speed_to_pulse(speed, self.motor2_neutral_offset)
        self.pi.set_servo_pulsewidth(self.motor2_pin, pulse)
    
    def set_both_motors(self, speed1, speed2):
        """Set both motors simultaneously"""
        self.set_motor1(speed1)
        self.set_motor2(speed2)
    
    def stop(self):
        """Stop both motors"""
        self.set_both_motors(0, 0)
    
    def calibrate_motor1(self, offset):
        """
        Adjust Motor 1 neutral point
        offset: microseconds to add to neutral pulse (typically -20 to +20)
        """
        self.motor1_neutral_offset = offset
        print(f"Motor 1 neutral offset set to {offset}μs")
        self.set_motor1(0)
    
    def calibrate_motor2(self, offset):
        """
        Adjust Motor 2 neutral point
        offset: microseconds to add to neutral pulse (typically -20 to +20)
        """
        self.motor2_neutral_offset = offset
        print(f"Motor 2 neutral offset set to {offset}μs")
        self.set_motor2(0)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        print("Stopping motors...")
        self.stop()
        time.sleep(0.2)
        
        # Turn off servo pulses
        self.pi.set_servo_pulsewidth(self.motor1_pin, 0)
        self.pi.set_servo_pulsewidth(self.motor2_pin, 0)
        
        # Set pins to ground
        self.pi.write(self.motor1_pin, 0)
        self.pi.write(self.motor2_pin, 0)
        
        # Stop pigpio
        self.pi.stop()
        print("Cleanup complete - All outputs set to ground")


def parse_joystick_command(line):
    """
    Parse joystick command from log line
    Expected format: "2025-12-01 01:09:16.780 | JOYSTICK | Joystick x=-0.28, y=0.56"
    Returns: (x, y) tuple or None if parse fails
    """
    try:
        match = re.search(r'Joystick x=([-\d.]+),\s*y=([-\d.]+)', line)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            return (x, y)
    except (ValueError, AttributeError):
        pass
    return None


def get_latest_joystick_command():
    """
    Read the latest joystick command from log file
    Returns: (x, y) tuple or None if no valid command found
    """
    if not os.path.exists(JOYSTICK_LOG_FILE):
        return None
    
    try:
        with open(JOYSTICK_LOG_FILE, 'r') as f:
            lines = f.readlines()
        
        # Find the last JOYSTICK line
        for line in reversed(lines):
            if 'JOYSTICK' in line:
                return parse_joystick_command(line)
    except IOError as e:
        print(f"Error reading joystick log: {e}")
    
    return None


def erase_latest_joystick_command():
    """
    Remove the most recent joystick command from the log file
    """
    if not os.path.exists(JOYSTICK_LOG_FILE):
        return False
    
    try:
        with open(JOYSTICK_LOG_FILE, 'r') as f:
            lines = f.readlines()
        
        # Find and remove the last JOYSTICK line
        for i in range(len(lines) - 1, -1, -1):
            if 'JOYSTICK' in lines[i]:
                lines.pop(i)
                break
        
        # Write back the modified file
        with open(JOYSTICK_LOG_FILE, 'w') as f:
            f.writelines(lines)
        
        return True
    except IOError as e:
        print(f"Error erasing joystick command: {e}")
        return False


def map_joystick_to_motors(x, y):
    """
    Map joystick x,y values to motor commands
    
    Mapping logic:
    - y (forward/back): controls forward/backward motion
    - x (left/right): controls turning
    
    Returns: (motor1_speed, motor2_speed)
    """
    # Clamp values to -1.0 to 1.0
    x = max(-1.0, min(1.0, x))
    y = max(-1.0, min(1.0, y))
    
    # Apply threshold to filter small noise
    if abs(x) < JOYSTICK_THRESHOLD:
        x = 0.0
    if abs(y) < JOYSTICK_THRESHOLD:
        y = 0.0
    
    # Convert to percentage (-100 to 100)
    y_speed = int(y * 100)  # Forward/backward
    x_turn = int(x * 100)   # Left/right turning
    
    # Differential drive motor mixing:
    # Motor 1 (left):  forward speed - turning (positive x = right turn, so reduce left motor)
    # Motor 2 (right): forward speed + turning
    
    motor1_speed = y_speed - x_turn
    motor2_speed = y_speed + x_turn
    
    # Clamp to valid range
    motor1_speed = max(-100, min(100, motor1_speed))
    motor2_speed = max(-100, min(100, motor2_speed))
    
    return (motor1_speed, motor2_speed)


def determine_direction(x, y):
    """
    Determine direction description from joystick values
    Returns: string describing the direction
    """
    # Apply threshold
    if abs(x) < JOYSTICK_THRESHOLD:
        x = 0.0
    if abs(y) < JOYSTICK_THRESHOLD:
        y = 0.0
    
    if y == 0 and x == 0:
        return "STOP"
    elif abs(x) < abs(y):
        if y > 0:
            return "FORWARD"
        else:
            return "BACK"
    else:
        if x > 0:
            return "RIGHT"
        else:
            return "LEFT"


def process_joystick_input(controller):
    """
    Main loop to read joystick commands and control motors
    """
    print("Joystick Motor Control Active")
    print("Reading from:", JOYSTICK_LOG_FILE)
    print("Press Ctrl+C to stop\n")
    
    last_command = None
    
    try:
        while True:
            # Get the latest joystick command
            command = get_latest_joystick_command()
            
            if command and command != last_command:
                x, y = command
                direction = determine_direction(x, y)
                motor1_speed, motor2_speed = map_joystick_to_motors(x, y)
                
                # Apply motor commands
                controller.set_both_motors(motor1_speed, motor2_speed)
                
                # Print status
                print(f"Joystick: x={x:6.2f}, y={y:6.2f} | Direction: {direction:8} | "
                      f"Motors: M1={motor1_speed:4d}%, M2={motor2_speed:4d}%")
                
                # Erase the processed command
                erase_latest_joystick_command()
                
                last_command = command
            else:
                # Small delay to avoid busy waiting
                time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\n\nStopping...")


def calibration_mode(controller):
    """Interactive calibration to eliminate drift"""
    print("\n=== CALIBRATION MODE ===")
    print("This will help you find the correct neutral point for each motor")
    print("Watch the motors and adjust until they don't move at 'stop'\n")
    
    # Calibrate Motor 1
    print("--- Calibrating Motor 1 ---")
    offset1 = 0
    while True:
        controller.calibrate_motor1(offset1)
        print(f"Current Motor 1 offset: {offset1}μs")
        print("Is Motor 1 completely stopped? (y=yes, +=increase, -=decrease, q=quit)")
        choice = input().strip().lower()
        if choice == 'y':
            break
        elif choice == '+':
            offset1 += 5
        elif choice == '-':
            offset1 -= 5
        elif choice == 'q':
            return
    
    # Calibrate Motor 2
    print("\n--- Calibrating Motor 2 ---")
    offset2 = 0
    while True:
        controller.calibrate_motor2(offset2)
        print(f"Current Motor 2 offset: {offset2}μs")
        print("Is Motor 2 completely stopped? (y=yes, +=increase, -=decrease, q=quit)")
        choice = input().strip().lower()
        if choice == 'y':
            break
        elif choice == '+':
            offset2 += 5
        elif choice == '-':
            offset2 -= 5
        elif choice == 'q':
            return
    
    print(f"\n✓ Calibration complete!")
    print(f"Motor 1 offset: {offset1}μs")
    print(f"Motor 2 offset: {offset2}μs")
    print("Add these lines to your code after initialization:")
    print(f"controller.calibrate_motor1({offset1})")
    print(f"controller.calibrate_motor2({offset2})\n")


def main():
    """Main function"""
    print("Sabertooth 2x12 Joystick Control Program")
    print("Configuration: R/C Microcontroller Mode, Independent, Linear")
    
    # Check if pigpiod is running
    print("\nChecking for pigpio daemon...")
    import subprocess
    try:
        result = subprocess.run(['pgrep', 'pigpiod'], capture_output=True)
        if result.returncode != 0:
            print("pigpio daemon not running. Starting it now...")
            subprocess.run(['sudo', 'pigpiod'], check=True)
            time.sleep(1)
    except Exception as e:
        print(f"Could not start pigpiod: {e}")
        print("Please run: sudo pigpiod")
        return
    
    print("\nPress Ctrl+C to stop\n")
    
    try:
        # Initialize controller
        controller = Sabertooth2x12()
        
        # Ask if user wants to calibrate
        print("\nDo you want to run calibration first? (y/n)")
        choice = input().strip().lower()
        if choice == 'y':
            calibration_mode(controller)
        
        # Run joystick control loop
        process_joystick_input(controller)
        
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            print("\n✓ Bringing motors to complete stop...")
            controller.stop()
            time.sleep(1)  # Give motors time to come to a complete stop
            controller.cleanup()
            print("✓ All motors stopped and cleaned up")


if __name__ == "__main__":
    main()
