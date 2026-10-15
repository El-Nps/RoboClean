#!/usr/bin/env python3
import time
import serial
from pynput import keyboard

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

MIN_PWM = -255
MAX_PWM = 255

class MotorTester:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2.0)  # allow Arduino reset
        self.pwm = 0
        self.inc_pressed = False
        self.dec_pressed = False
        print(f"Connected on {port} @ {baudrate}")
        print("Controls:")
        print("  ↑ : increase PWM by 1 (hold for continuous change)")
        print("  ↓ : decrease PWM by 1")
        print("  SPACE : set PWM to 0 (stop)")
        print("  ESC : quit")

    def send_pwm(self):
        cmd = f"M,{self.pwm},{self.pwm}\n"
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()
        print(f"PWM = {self.pwm}")

    def close(self):
        # stop motors
        self.pwm = 0
        self.send_pwm()
        self.ser.close()

    def step(self):
        if self.inc_pressed and self.pwm < MAX_PWM:
            self.pwm += 1
            self.send_pwm()
        if self.dec_pressed and self.pwm > MIN_PWM:
            self.pwm -= 1
            self.send_pwm()

def main():
    tester = MotorTester(PORT, BAUDRATE)

    def on_press(key):
        try:
            if key == keyboard.Key.up:
                tester.inc_pressed = True
            elif key == keyboard.Key.down:
                tester.dec_pressed = True
            elif key == keyboard.Key.space:
                tester.pwm = 0
                tester.send_pwm()
        except Exception as e:
            print(f"Error on_press: {e}")

    def on_release(key):
        try:
            if key == keyboard.Key.up:
                tester.inc_pressed = False
            elif key == keyboard.Key.down:
                tester.dec_pressed = False
            elif key == keyboard.Key.esc:
                # stop listener -> end program
                return False
        except Exception as e:
            print(f"Error on_release: {e}")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while listener.is_alive():
            tester.step()
            time.sleep(0.01)
    finally:
        tester.close()
        listener.stop()

if __name__ == "__main__":
    main()
