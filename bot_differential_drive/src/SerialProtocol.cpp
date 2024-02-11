//
// Created by phil on 18.05.21.
//

#include "SerialProtocol.h"
#include <algorithm>
namespace serial_protocol {
MsgType parseType(const std::string &raw) {
  auto t = raw.substr(0, 1);
  if (t == "S") {
    return MsgType::STATE;
  }
  if (t == "v") {
    return MsgType::CMD_VEL;
  }
  return MsgType::UNKNOWN;
}

std::string to_string(MsgType msgType) {
  switch (msgType) {
  case MsgType ::CMD_VEL:
    return "v";
  case MsgType ::STATE:
    return "S";
  case MsgType ::Q_STATE:
    return "Q";
  default:
    return "";
  }
}

Message::Message(const std::string &msg) : _str(msg) {
  _fields = split(msg, ' ');
  _type = parseType(msg);
  _t = std::stoull(_fields[1]);
}

Message::Message(serial_protocol::MsgType type, uint64_t t)
    : _t(t), _type(type) {
  std::stringstream ss;
  ss << to_string(_type) << " " << t << "\n";
  _str = ss.str();
}

MsgCmdVel::MsgCmdVel(const std::string &raw) : Message(raw) {
  _vl = std::stod(_fields[2]);
  _vr = std::stod(_fields[3]);
}
MsgCmdVel::MsgCmdVel(float vl, float vr, uint64_t t)
    : Message(MsgType::CMD_VEL, t), _vl(vl), _vr(vr) {
  std::stringstream ss;
  ss << to_string(_type) << " " << t << " " << vl << " " << vr << "\n";
  _str = ss.str();
}

MsgState::MsgState(const std::string &raw) : Message(raw) {
  _stateLeft.position = std::stof(_fields[2]);
  _stateLeft.angularVelocityCmd = std::stof(_fields[3]);
  _stateLeft.angularVelocity = std::stof(_fields[4]);
  _stateLeft.err = std::stof(_fields[5]);
  _stateLeft.dutySet = std::stof(_fields[6]);

  _stateRight.position = std::stof(_fields[7]);
  _stateRight.angularVelocityCmd = std::stof(_fields[8]);
  _stateRight.angularVelocity = std::stof(_fields[9]);
  _stateRight.err = std::stof(_fields[10]);
  _stateRight.dutySet = std::stof(_fields[11]);
}
MsgState::MsgState(const State &stateLeft, const State &stateRight, uint64_t t)
    : _stateLeft(stateLeft), _stateRight(stateRight),
      Message(MsgType::STATE, t) {
  std::stringstream ss;
  ss << to_string(_type) << " " << t << " " << stateLeft.position << " "
     << stateLeft.angularVelocityCmd << " " << stateLeft.angularVelocity << " "
     << stateLeft.err;
  ss << stateRight.position << " " << stateRight.angularVelocityCmd << " "
     << stateRight.angularVelocity << " " << stateRight.err;
  ss << "\n";
  _str = ss.str();
}
std::string MsgState::str() const {
  std::stringstream ss;
  ss.precision(4);
  ss << "       |" << std::setw(6) << "L"
     << " |" << std::setw(8) << "R|\n";
  ss << " t |" << std::setw(6) << (double)_t << " |" << std::setw(6)
     << (double)_t << "|\n";
  ss << " p |" << std::setw(6) << _stateLeft.position << " |" << std::setw(6)
     << _stateRight.position << "|\n";
  ss << " v*    |" << std::setw(6) << _stateLeft.angularVelocityCmd << " |"
     << std::setw(6) << _stateRight.angularVelocityCmd << "|\n";
  ss << " v     |" << std::setw(6) << _stateLeft.angularVelocity << " |"
     << std::setw(6) << _stateRight.angularVelocity << "|\n";
  ss << " err   |" << std::setw(6) << _stateLeft.err << " |" << std::setw(6)
     << _stateRight.err << "|\n";
  ss << " pwm   |" << std::setw(6) << _stateLeft.dutySet << " |" << std::setw(6)
     << _stateRight.dutySet << "|\n";
  return ss.str();
}

MsgQueryState::MsgQueryState(uint64_t t) : Message(MsgType::Q_STATE, t) {
  std::stringstream ss;
  ss << to_string(_type) << " " << t << " s"
     << "\n";
  _str = ss.str();
}

std::vector<std::string> split(const std::string &s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}
} // namespace serial_protocol