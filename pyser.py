import serial

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
ser.write(b"Hello Arduino\n")

while True:
    if ser.in_waiting:
        print("Received: ", ser.readline().decode('utf-8').strip())