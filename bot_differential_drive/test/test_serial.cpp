#include "SerialPort.hpp"

using namespace mn::CppLinuxSerial;

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "SerialProtocol.h"

int main() {
  SerialProtocol serial;
  auto port = std::make_shared<SerialPort>();
  // Create serial port object and open serial port
  port->SetBaudRate(BaudRate::B_9600);
  port->SetDevice("/dev/ttyACM0");
  // SerialPort serialPort("/dev/ttyACM0", 13000);
  port->SetTimeout(100); // Block when reading until any data is received
  port->Open();

  // Write some ASCII datae
  const float T_MS = 10;

  for (int i = 0; i < 1000; i++) {
    float setPoint = std::sin((float)i / 10.0) * 12.0;
    if (setPoint < 4) {
      setPoint = 4;
    }
    //        setPoint = 0;
    auto msgOut =
        std::make_shared<SerialProtocol::MsgCmdVel>(setPoint, -1 * setPoint, i);
    //        auto msgOut =
    //        std::make_shared<SerialProtocol::MsgCmdVel>(0,-1*setPoint,i); auto
    //        msgOut =
    //        std::make_shared<SerialProtocol::MsgCmdVel>(setPoint,0,i);
    std::cout << "Sending:\n" << msgOut->str() << std::endl;

    port->Write(msgOut->serialStr());
    try {
      serial.parse(10, port);

    } catch (const SerialProtocol::ParseError &e) {
      std::cerr << e.what() << "\n Message: " << e._parsedMessage << std::endl;
    }

    // Handle newest state message
    if (!serial._messagesState.empty()) {
      auto msg = serial._messagesState[serial._messagesState.size() - 1];
      std::cout << msg->str() << std::endl;
    }

    for (const auto &msg : serial._messages) {
      std::cout << msg->str() << std::endl;
    }

    serial._messagesState.clear();
    serial._messages.clear();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Close the serial port
  port->Close();
}
