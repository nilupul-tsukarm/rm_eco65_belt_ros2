import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
while True:
    if ser.in_waiting > 0:
        print(ser.readline().decode('utf-8').strip())
