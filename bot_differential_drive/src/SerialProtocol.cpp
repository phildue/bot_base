//
// Created by phil on 18.05.21.
//

#include "SerialProtocol.h"
#include <algorithm>
namespace serial_protocol {

MsgType parseType(const std::string &raw) {
  auto typeMsg = raw.substr(0, raw.find(' '));
  int i = 0;
  for (const auto &type : MsgTypeToString) {
    if (typeMsg == type) {
      return (MsgType)i;
    }
    i++;
  }
  return MsgType::UNKNOWN;
}

MsgCmdVel::MsgCmdVel(float vl, float vr, uint64_t t)
    : _vl(vl), _vr(vr), _t(t) {}

std::string MsgCmdVel::strSerial() const {
  std::stringstream ss;
  ss << "set vel " << _t << " " << _vl << " " << _vr << "\n";
  return ss.str();
}
MsgStateVelocityAndPosition::MsgStateVelocityAndPosition(
    const std::string &raw) {

  auto fields = split(raw, ' ');
  if (fields[1] != "vap") {
    throw std::invalid_argument(
        "This is not a [velocity and position] message: " + raw);
  }
  if (fields.size() < 7) {
    std::stringstream ss;
    for (const auto &f : fields) {
      ss << f << ",";
    }
    throw std::invalid_argument(
        "The [velocity and position] message is not complete [" +
        std::to_string(fields.size()) + "/7]: " + ss.str());
  }
  _t = std::stoull(fields[2]);
  _pl = std::stof(fields[3]);
  _vl = std::stof(fields[4]);
  _pr = std::stof(fields[5]);
  _vr = std::stof(fields[6]);
}
std::string MsgStateVelocityAndPosition::str() const {
  std::stringstream ss;
  ss.precision(4);
  ss << " vl = " << _vl << " pl = " << _pl << " vr = " << _vr
     << " pr = " << _pr;
  return ss.str();
}

std::string MsgQueryState::strSerial() const {
  std::stringstream ss;
  ss << MsgTypeToString[(int)MsgType::QUERY] << " "
     << MsgTypeQueryToString[(int)MsgTypeQuery::VELOCITY_AND_POSITION] << "\n";
  return ss.str();
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