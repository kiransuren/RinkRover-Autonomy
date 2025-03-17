import smbus2
import struct
import time

# I2C bus
bus = smbus2.SMBus(1)  # Use 1 for Raspberry Pi 3, 4, 5

# BNO055 I2C address
BNO055_ADDRESS = 0x29  # Alternative: 0x29

# Register addresses for offsets
ACCEL_OFFSET_X_LSB = 0x55
GYRO_OFFSET_X_LSB = 0x61

# Read 16-bit signed values from registers
def read_word(reg):
    data = bus.read_i2c_block_data(BNO055_ADDRESS, reg, 2)
    value = struct.unpack('<h', bytes(data))[0]  # Convert little-endian bytes to signed short
    return value

# Function to read all offsets
def read_offsets():
    print("Reading applied offsets...")
    
    acc_x = read_word(ACCEL_OFFSET_X_LSB)
    acc_y = read_word(ACCEL_OFFSET_X_LSB + 2)
    
    gyro_x = read_word(GYRO_OFFSET_X_LSB)
    gyro_y = read_word(GYRO_OFFSET_X_LSB + 2)
    gyro_z = read_word(GYRO_OFFSET_X_LSB + 4)
    
    print(f"Accelerometer Offsets -> X: {acc_x}, Y: {acc_y}")
    print(f"Gyroscope Offsets -> X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}")

# Main execution
try:
    read_offsets()
except KeyboardInterrupt:
    print("\nExiting...")
    bus.close()
