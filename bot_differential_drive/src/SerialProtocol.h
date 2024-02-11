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
enum class MsgType { UNKNOWN, CMD_VEL, STATE, QUERY, INFO };
MsgType parseType(const std::string &str);
std::string to_string(MsgType msgType);

struct Message {
public:
  Message(const std::string &msg);
  Message(MsgType type, uint64_t t);
  const std::string &serialStr() const { return _str; };
  const MsgType &type() const { return _type; };
  const uint64_t &t() const { return _t; }
  virtual std::string str() const { return _str; }

protected:
  std::string _str;
  MsgType _type;
  uint64_t _t;
  std::vector<std::string> _fields;
};
struct MsgCmdVel : public Message {
  float _vl, _vr;

  MsgCmdVel(const std::string &raw);
  MsgCmdVel(float vl, float vr, uint64_t t);
};

struct State {
  float angularVelocityCmd, angularVelocity, position, err, dutySet;
};
struct MsgState : public Message {
  State _stateLeft, _stateRight;
  MsgState(const std::string &raw);
  MsgState(const State &stateLeft, const State &stateRight, uint64_t t);
  std::string str() const override;
};

struct MsgQueryState : public Message {
  MsgQueryState(uint64_t t);
};

static std::vector<std::string> split(const std::string &s, char delimiter);
}; // namespace serial_protocol

#endif // SRC_SERIALPROTOCOL_H
