import serial
import time
import threading

# Robot Parameters
WHEEL_SEPARATION = 0.25  # meters

# Configure serial port (change 'COM3' to your Arduino's port)
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# Global variables for movement control
current_left = 0.0
current_right = 0.0
vacuum_state = 0
brush_state = 0
running = True

def get_keyboard_input():
    """Handle keyboard input in a separate thread"""
    global current_left, current_right, vacuum_state, brush_state, running
    
    while running:
        key = input().strip().lower()
        
        if key == 'w':
            current_left = 150   # PWM value (0-255)
            current_right = 150
        elif key == 's':
            current_left = -150
            current_right = -150
        elif key == 'a':
            current_left = -100
            current_right = 100
        elif key == 'd':
            current_left = 100
            current_right = -100
        elif key == 'v':
            vacuum_state = 1 - vacuum_state
            send_vacuum_command(vacuum_state)
        elif key == 'b':
            brush_state = 1 - brush_state
            send_brush_command(brush_state)
        elif key == 'stop' or key == '':
            current_left = 0
            current_right = 0
        elif key == 'q':
            running = False
            break

def send_command(vl, vr):
    """Send velocity command to Arduino"""
    command = f"M,{int(vl)},{int(vr)}\n"
    arduino.write(command.encode())
    print(f"Sent: {command.strip()}")

def send_vacuum_command(state):
    """Send vacuum command to Arduino"""
    command = f"V,{state}\n"
    arduino.write(command.encode())
    print(f"Sent: {command.strip()}")

def send_brush_command(state):
    """Send brush command to Arduino"""
    command = f"B,{state}\n"
    arduino.write(command.encode())
    print(f"Sent: {command.strip()}")

print("Robot Control - Type commands and press Enter:")
print("w: Forward")
print("s: Backward")
print("a: Turn Left")
print("d: Turn Right")
print("v: Toggle Vacuum")
print("b: Toggle Brush")
print("stop: Stop (or just press Enter)")
print("q: Quit")

# Start keyboard input thread
input_thread = threading.Thread(target=get_keyboard_input, daemon=True)
input_thread.start()

try:
    last_print = 0
    last_left = None
    last_right = None
    
    while running:
        # Only send command if values changed
        if current_left != last_left or current_right != last_right:
            send_command(current_left, current_right)
            last_left = current_left
            last_right = current_right
        
        # Read available data from serial
        if arduino.in_waiting:
            try:
                line = arduino.readline().decode('utf-8').strip()
                if line:
                    now = time.time()
                    if line.startswith("E,"):
                        # E,deltaLeft,deltaRight
                        parts = line.strip().split(',')
                        if len(parts) >= 3 and now - last_print > 0.5:
                           # print(f"Encoder: L={parts[1]} R={parts[2]}")
                           # Frequency 20Hz
                           pass 
                    elif line.startswith("S,"):
                        # S,left,middle,right
                        parts = line.strip().split(',')
                        if len(parts) >= 4 and now - last_print > 0.5:
                            # print(f"Sensors: L={parts[1]} M={parts[2]} R={parts[3]}")
                            # Frequency 10Hz
                            last_print = now
                    elif line.startswith("V,") or line.startswith("B,"):
                       print(f"Ack: {line}")
                    elif line.startswith("SA,") or line.startswith("SD,"):
                       print(f"Sensor Read: {line}")
                    else:
                        pass # print(f"Unknown: {line}")
            except Exception as e:
                print(f"Error reading serial: {e}")
        
        time.sleep(0.01)  # Small sleep to prevent high CPU usage

finally:
    send_command(0, 0)  # Stop before quitting
    arduino.close()
