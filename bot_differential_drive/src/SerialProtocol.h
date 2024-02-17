//
// Created by phil on 18.05.21.
//

#ifndef ROBOPI_SERIALPROTOCOL_H
#define ROBOPI_SERIALPROTOCOL_H

#include "SerialPort.hpp"
using namespace mn::CppLinuxSerial;

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <thread>
namespace serial_protocol {
enum class MsgType { UNKNOWN = 0, SETPOINT, STATE, QUERY, INFO };
constexpr std::array MsgTypeToString = {"unknown", "set", "state", "query",
                                        "info"};

enum class MsgTypeSetPoint { DUTY_CYCLE = 0, VELOCITY, RESET, CONFIG };
constexpr std::array MsgTypeSetPointToString = {"duty", "vel", "rst", "cfg"};

enum class MsgTypeQuery {
  VELOCITY = 0,
  POSITION,
  VELOCITY_AND_POSITION,
  CONFIG
};
constexpr std::array MsgTypeQueryToString = {"vel", "pos", "vap", "cfg"};

enum class MsgTypeState {
  VELOCITY = 0,
  POSITION,
  VELOCITY_AND_POSITION,
  CONFIG
};
constexpr std::array MsgTypeState = {"vel", "pos", "vap", "cfg"};

MsgType parseType(const std::string &raw);

struct MsgCmdVel {
  float _vl, _vr;
  std::uint64_t _t;
  MsgCmdVel(float vl, float vr, uint64_t t);
  std::string strSerial() const;
};

struct MsgStateVelocityAndPosition {
  float _vl, _vr, _pl, _pr;
  std::uint64_t _t;
  MsgStateVelocityAndPosition(const std::string &raw);
  std::string str() const;
};

struct MsgQueryState {
  std::string strSerial() const;
};

static std::vector<std::string> split(const std::string &s, char delimiter);
}; // namespace serial_protocol

#endif // SRC_SERIALPROTOCOL_H
