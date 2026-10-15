import serial
import time

# Configure serial port (change 'COM3' to your Arduino's port)
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# to test analog read of pin A0 connect A0 to 5V or GND or a Digital power source
read = 0

# Send command
command = f"R,a,{read}\n"
arduino.write(command.encode())
print(f"Sent: {command.strip()}")

time.sleep(2)  # Execute command for 2 seconds
if (arduino.in_waiting > 0):   # if there is data to be read
    if hasattr(arduino, "read_until"):
       print("Data received: " + arduino.read_until(b'\n').decode().strip())
    else:
       print("Data received: " + arduino.readline().decode().strip())

arduino.close()
