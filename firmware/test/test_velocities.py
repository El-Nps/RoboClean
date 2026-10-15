import serial
import time

# Configure serial port (change 'COM3' to your Arduino's port)
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

WHEEL_SEPARATION = 0.25  # meters

# Test cases: [vl, vr]
test_cases = [
    [0.2, 0.2],    # Forward
    [0.4, 0.4],    # Fast Forward
    [-0.2, 0.2],   # Rotate Left
    [0.2, -0.2],   # Rotate Right
    [-0.2, -0.2],  # Backward
]

for vl, vr in test_cases:
    command = f"M,{vl:.3f},{vr:.3f}\n"
    arduino.write(command.encode())
    print(f"Sent: {command.strip()}")

    time.sleep(2)  # Execute command for 2 seconds


arduino.write(b"M,0,0\n")  # Stop the robot
arduino.close()
