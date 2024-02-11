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
  auto port = std::make_shared<SerialPort>("/dev/ttyACM0", BaudRate::B_9600);
  port->SetTimeout(100); // Block when reading until any data is received
  port->Open();

  const int T_MS = 100;

  for (int i = 0; i < 1000; i++) {
    float setPoint = std::sin((float)i / 10.0) * 12.0;
    if (setPoint < 4) {
      setPoint = 4;
    }
    serial_protocol::MsgCmdVel setPointMsg{setPoint, -1 * setPoint,
                                           (uint64_t)i};
    std::cout << "Sending:\n" << setPointMsg.str() << std::endl;
    port->Write(setPointMsg.serialStr());

    serial_protocol::MsgQueryState queryState{(uint64_t)i};
    std::cout << "Sending:\n" << queryState.str() << std::endl;
    port->Write(queryState.serialStr());

    auto raw = port->ReadUntil('\n', 100);
    if (serial_protocol::parseType(raw) == serial_protocol::MsgType::STATE) {
      serial_protocol::MsgState state{raw};
      std::cout << "Received:\n" << state.str() << std::endl;
    } else {
      std::cerr << "Error! Received:\n" << raw << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(T_MS));
  }

  // Close the serial port
  port->Close();
}
