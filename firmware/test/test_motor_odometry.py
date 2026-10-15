import serial
import time

# Configure serial port (change 'COM3' to your Arduino's port)
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

WHEEL_SEPARATION = 0.25

vl = 0.4
vr = -0.4

command = f"M,{vl:.3f},{vr:.3f}\n"
arduino.write(command.encode())
print(f"Sent: {command.strip()}")
while True:
    # Read encoder feedback
    response = arduino.readline().decode().strip()
    if response:
        print(f"Encoder counts: {response}")
