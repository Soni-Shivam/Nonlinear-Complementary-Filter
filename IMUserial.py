import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)  # Replace 'COM3' with your port and 9600 with your baud rate
time.sleep(2)  # Allow time for serial connection to establish

with open('serial_output.txt', 'w') as f:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            f.write(line + '\n')
            print(line)  # Optional: print to console as well