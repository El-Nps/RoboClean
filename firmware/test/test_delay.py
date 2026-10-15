import serial
import time

# Configure serial port (change 'COM3' to your Arduino's port)
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# Read from arduino
now = time.time_ns() // 1_000_000
while True:
    if arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        if response:
            print(f"Received: {response}")
            diff = time.time_ns() // 1_000_000 - now
            print(f"Time since start: {diff:.2f} ms")
    time.sleep(0.001)
