#ifndef RR_ARDUINO_ARDUINO_COMMS_HPP
#define RR_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  void send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);
    serial_conn_.Write("\r");
    return;
  }

  std::string send_msg_ack(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);
    serial_conn_.Write("\r");

    std::string response = "";
    try
    {
      // Responses end with \r so we will read up to (and including) the \rs.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    send_msg("");
  }

  void activate_controller()
  {
    std::string response = send_msg_ack("a");
    std::cerr << "RESPONSE: " << response.c_str() << std::endl;

    int response_key = 0;
    int scan_result = sscanf(response.c_str(), "q,%d", &response_key);

    if(scan_result < 1)
    {
      // error, bad scan
      std::cerr << "Bad Activation!" << std::endl;
    }
  }

  void deactivate_controller()
  {
    send_msg("d\r");
  }

  bool read_encoder_values(int &val_1, int &val_2)
  {
    std::cout << "Sending Message..." << std::endl;
    std::string response = send_msg_ack("e");
    std::cerr << "RESPONSE: " << response.c_str() << std::endl;

    int scan_result = sscanf(response.c_str(), "g,%d,%d", &val_1, &val_2);

    if(scan_result < 2)
    {
      // error, bad scan
      std::cerr << "Bad encoder value reading!" << std::endl;
      return false;
    }
  }
  void set_motor_values(int left_traction_motor, int right_traction_motor, int steering_motor)
  {
    std::stringstream ss;
    ss << "m," << left_traction_motor << "," << right_traction_motor << "," << steering_motor;
    send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o;
    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // RR_ARDUINO_ARDUINO_COMMS_HPP