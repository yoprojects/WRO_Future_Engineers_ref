import serial
import RPi.GPIO as GPIO
import time
import os

BUTTON_PIN = 17
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# === Retry until port is available ===
ser = None
while ser is None:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException:
        print("ESP32 not ready yet... retrying")
        time.sleep(2)

time.sleep(2)  # give ESP32 time to reset

print("Press the button to start the ESP32 program...")

while True:
    if GPIO.input(BUTTON_PIN) == GPIO.LOW:
        print("Button pressed! Sending START to ESP32...")
        ser.write(b"START\n")
        break
    time.sleep(0.05)

ser.close()
GPIO.cleanup()
