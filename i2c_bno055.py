import smbus2
import time
import struct

# I2C bus (1 for Raspberry Pi 3 and later)
bus = smbus2.SMBus(1)

# BNO055 I2C address
BNO055_ADDRESS = 0x29

# BNO055 Registers
BNO055_CHIP_ID_ADDR = 0x00
BNO055_OPR_MODE_ADDR = 0x3D
BNO055_PWR_MODE_ADDR = 0x3E
BNO055_SYS_TRIGGER_ADDR = 0x3F
BNO055_TEMP_ADDR = 0x34

# Operation modes
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_NDOF = 0x0C
OPERATION_MODE_ACCONLY = 0x01

# Power modes
POWER_MODE_NORMAL = 0x00

# Function to read a signed 16-bit value from the sensor
def read_signed_16bit(register):
    low_byte = bus.read_byte_data(BNO055_ADDRESS, register)
    high_byte = bus.read_byte_data(BNO055_ADDRESS, register + 1)
    value = (high_byte << 8) | low_byte
    if value > 32767:
        value -= 65536
    return value

# Initialize BNO055
def initialize_bno055():
    # Check if BNO055 is connected
    chip_id = bus.read_byte_data(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR)
    if chip_id != 0xA0:
        print("BNO055 not detected. Chip ID:", chip_id)
        return False

    # # Set to configuration mode
    # bus.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
    # time.sleep(0.025)

    # # Set power mode to normal
    # bus.write_byte_data(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
    # time.sleep(0.01)

    # Set operation mode to NDOF (Nine Degrees of Freedom)
    bus.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF)
    time.sleep(0.02)
    
    # Check calibration status
    CALIB_STAT_REG = 0x35
    calib_status = bus.read_byte_data(BNO055_ADDRESS, CALIB_STAT_REG)
    sys_cal = (calib_status >> 6) & 0x03
    acc_cal = (calib_status >> 2) & 0x03

    print(f"System Calibration: {sys_cal}, Accelerometer Calibration: {acc_cal}")


    return True

# Main function
def main():
    if not initialize_bno055():
        return

    while True:
        # Read temperature
        temp = bus.read_byte_data(BNO055_ADDRESS, BNO055_TEMP_ADDR)

        # Read accelerometer data (in m/s²)
        accel_x = read_signed_16bit(0x28) / 100.0
        accel_y = read_signed_16bit(0x2A) / 100.0
        accel_z = read_signed_16bit(0x2C) / 100.0

        # Read magnetometer data (in µT)
        mag_x = read_signed_16bit(0x0E) / 16.0
        mag_y = read_signed_16bit(0x10) / 16.0
        mag_z = read_signed_16bit(0x12) / 16.0

        # Read gyroscope data (in °/s)
        gyro_x = read_signed_16bit(0x14) / 16.0
        gyro_y = read_signed_16bit(0x16) / 16.0
        gyro_z = read_signed_16bit(0x18) / 16.0

        # Read Euler angles (in degrees)
        euler_h = read_signed_16bit(0x1A) / 16.0
        euler_r = read_signed_16bit(0x1C) / 16.0
        euler_p = read_signed_16bit(0x1E) / 16.0

        # Print sensor data
        print(f"Temperature: {temp} °C")
        print(f"Accelerometer (m/s²): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}")
        print(f"Magnetometer (µT): X={mag_x:.2f}, Y={mag_y:.2f}, Z={mag_z:.2f}")
        print(f"Gyroscope (°/s): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}")
        print(f"Euler angles (°): Heading={euler_h:.2f}, Roll={euler_r:.2f}, Pitch={euler_p:.2f}")
        print()

        time.sleep(1)

if __name__ == "__main__":
    main()

