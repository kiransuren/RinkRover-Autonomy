import smbus2
import time
import struct

# I2C bus
bus = smbus2.SMBus(1)  # Use 1 for Raspberry Pi 3, 4, 5

# BNO055 I2C address
BNO055_ADDRESS = 0x29  # Alternative address: 0x29
BNO055_CHIP_ID_ADDR = 0x00
# BNO055 Registers
OPR_MODE_REG = 0x3D
ACCEL_DATA_X_LSB = 0x08
GYRO_DATA_X_LSB = 0x14
ACCEL_OFFSET_X_LSB = 0x55
GYRO_OFFSET_X_LSB = 0x61

# Switch to CONFIG mode to write offsets
CONFIG_MODE = 0x00
IMU_MODE = 0x0C  # Normal IMU mode

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

# Power modes
POWER_MODE_NORMAL = 0x00

# Read 16-bit signed values
def read_word(reg):
    data = bus.read_i2c_block_data(BNO055_ADDRESS, reg, 2)
    value = struct.unpack('<h', bytes(data))[0]  # Little-endian signed short
    return value

# Write a 16-bit signed value to a register
def write_word(reg, value):
    lsb = value & 0xFF
    msb = (value >> 8) & 0xFF
    bus.write_byte_data(BNO055_ADDRESS, reg, lsb)
    bus.write_byte_data(BNO055_ADDRESS, reg + 1, msb)

# Read multiple IMU samples and compute offsets
def compute_offsets(samples=100):
    acc_x_total, acc_y_total = 0, 0
    gyro_x_total, gyro_y_total, gyro_z_total = 0, 0, 0

    print(f"Collecting {samples} samples to calculate offsets...")

    for _ in range(samples):
        acc_x_total += read_word(ACCEL_DATA_X_LSB)
        acc_y_total += read_word(ACCEL_DATA_X_LSB + 2)
        gyro_x_total += read_word(GYRO_DATA_X_LSB)
        gyro_y_total += read_word(GYRO_DATA_X_LSB + 2)
        gyro_z_total += read_word(GYRO_DATA_X_LSB + 4)
        time.sleep(0.01)  # Small delay between readings

    # Calculate average drift
    print(f"Computed Totals Offsets -> AccX: {acc_x_total}, AccY: {acc_y_total}, "
          f"GyroX: {gyro_x_total}, GyroY: {gyro_y_total}, GyroZ: {gyro_z_total}")
    acc_x_offset = -acc_x_total // samples
    acc_y_offset = -acc_y_total // samples
    gyro_x_offset = -gyro_x_total // samples
    gyro_y_offset = -gyro_y_total // samples
    gyro_z_offset = -gyro_z_total // samples

    print(f"Computed Offsets -> AccX: {acc_x_offset}, AccY: {acc_y_offset}, "
          f"GyroX: {gyro_x_offset}, GyroY: {gyro_y_offset}, GyroZ: {gyro_z_offset}")

    return acc_x_offset, acc_y_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset

# Apply computed offsets
def apply_offsets():
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

    # # Set operation mode to NDOF (Nine Degrees of Freedom)
    # bus.write_byte_data(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF)
    # time.sleep(0.02)

    # # Compute offsets
    # acc_x_offset, acc_y_offset, gyro_x_offset, gyro_y_offset, gyro_z_offset = compute_offsets()
    
    acc_x_offset = 0#53
    acc_y_offset = 0#7
    gyro_x_offset = 0#-1
    gyro_y_offset = 0
    gyro_z_offset = 0#-1
    
    print("Switching to CONFIG mode...")
    write_byte(OPR_MODE_REG, CONFIG_MODE)
    time.sleep(0.1)
    
    # Write offsets to registers
    write_word(ACCEL_OFFSET_X_LSB, acc_x_offset)
    write_word(ACCEL_OFFSET_X_LSB + 2, acc_y_offset)
    write_word(GYRO_OFFSET_X_LSB, gyro_x_offset)
    write_word(GYRO_OFFSET_X_LSB + 2, gyro_y_offset)
    write_word(GYRO_OFFSET_X_LSB + 4, gyro_z_offset)

    print("Switching back to IMU mode...")
    write_byte(OPR_MODE_REG, IMU_MODE)
    time.sleep(0.1)

    print("Offsets applied successfully!")

# Function to write a byte
def write_byte(reg, value):
    bus.write_byte_data(BNO055_ADDRESS, reg, value)

# Main execution
try:
    input("Press Enter to start automatic IMU calibration...")
    apply_offsets()
    print("Calibration complete! Offsets are now applied.")
except KeyboardInterrupt:
    print("\nExiting...")
    bus.close()
