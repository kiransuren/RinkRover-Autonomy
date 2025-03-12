#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_DEVICE "/dev/i2c-1"
#define ARDUINO_ADDRESS 0x08  // Arduino Mega I2C address
#define BNO055_ADDRESS 0x29   // BNO055 I2C address

#define BNO055_OPR_MODE 0x3D
#define BNO055_QUAT_START 0x20  // Quaternion (W, X, Y, Z)
#define BNO055_GYRO_START 0x14   // Gyroscope (X, Y, Z)
#define BNO055_ACCEL_START 0x08  // Accelerometer (X, Y, Z)
#define BNO055_NDOF_MODE 0x0C  // Full sensor fusion mode

class I2CBus {
private:
    int file;

public:
    I2CBus() {
        // Do nothing
    }

    ~I2CBus() {
        close(file);
    }

    bool openBus() {
        if ((file = open(I2C_DEVICE, O_RDWR)) < 0) {
            std::cerr << "Failed to open I2C bus" << std::endl;
            return false;
        }

        // Initialize BNO055
        if (setSlave(BNO055_ADDRESS)) {
            setMode(BNO055_NDOF_MODE);
        }
        return true;
    }

    bool setSlave(uint8_t address) {
        if (ioctl(file, I2C_SLAVE, address) < 0) {
            std::cerr << "Failed to connect to I2C slave at 0x" 
                      << std::hex << (int)address << std::dec << std::endl;
            return false;
        }
        return true;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        if (write(file, data, 2) != 2) {
            std::cerr << "I2C Write Error" << std::endl;
            return false;
        }
        //usleep(10000);  // Delay to ensure mode change
        return true;
    }

    bool readData(uint8_t reg, uint8_t *buffer, size_t length) {
        if (write(file, &reg, 1) != 1) {
            std::cerr << "Failed to select register" << std::endl;
            return false;
        }
        if (read(file, buffer, length) != (ssize_t)length) {
            std::cerr << "I2C Read Error" << std::endl;
            return false;
        }
        return true;
    }

    void setMode(uint8_t mode) {
        writeRegister(BNO055_OPR_MODE, mode);
    }

    bool readEncoders(int &enc1, int &enc2) {
        if (!setSlave(ARDUINO_ADDRESS)) return false;

        uint8_t data[4];
        if (!readData(0x00, data, 4)) return false;

        enc1 = (data[0] << 8) | data[1];  
        enc2 = (data[2] << 8) | data[3];

        return true;
    }

    bool readQuatAngles(int16_t &qw, int16_t &qx, int16_t &qy, int16_t &qz) {
        if (!setSlave(BNO055_ADDRESS)) return false;

        uint8_t data[6];
        if (!readData(BNO055_QUAT_START, data, 8)) return false;

        // Combine low and high bytes (Little Endian format)
        qw = (data[1] << 8) | data[0];
        qx = (data[3] << 8) | data[2];
        qy = (data[5] << 8) | data[4];
        qz = (data[7] << 8) | data[6];

        return true;
    }

    bool readGyroscope(int16_t &gx, int16_t &gy, int16_t &gz) {
        if (!setSlave(BNO055_ADDRESS)) return false;

        uint8_t data[6];
        if (!readData(BNO055_GYRO_START, data, 6)) return false;

        gx = (data[1] << 8) | data[0];
        gy = (data[3] << 8) | data[2];
        gz = (data[5] << 8) | data[4];

        return true;
    }

    bool readAccelerometer(int16_t &ax, int16_t &ay, int16_t &az) {
        if (!setSlave(BNO055_ADDRESS)) return false;

        uint8_t data[6];
        if (!readData(BNO055_ACCEL_START, data, 6)) return false;

        ax = (data[1] << 8) | data[0];
        ay = (data[3] << 8) | data[2];
        az = (data[5] << 8) | data[4];

        return true;
    }

    bool writeMotorCommand(int16_t left_motor_cmd, int16_t right_motor_cmd, int16_t steering_motor_cmd) {
        if (!setSlave(ARDUINO_ADDRESS)) return false;

        uint8_t data[7];
        data[0] = 0x01; // Command identifier (optional)
        data[1] = (left_motor_cmd >> 8) & 0xFF;   // Left Motor High Byte
        data[2] = left_motor_cmd & 0xFF;          // Left Motor Low Byte
        data[3] = (right_motor_cmd >> 8) & 0xFF;  // Right Motor High Byte
        data[4] = right_motor_cmd & 0xFF;         // Right Motor Low Byte
        data[5] = (steering_motor_cmd >> 8) & 0xFF;  // Steering Motor High Byte
        data[6] = steering_motor_cmd & 0xFF;         // Steering Motor Low Byte

        if (write(file, data, 7) != 7) {
            std::cerr << "I2C Write Error" << std::endl;
            return false;
        }

        std::cout << "Sent I2C command: L=" << left_motor_cmd 
                  << ", R=" << right_motor_cmd 
                  << ", S=" << steering_motor_cmd << std::endl;

        return true;
    }
};
