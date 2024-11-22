#include "EthCommunication.h"

EthCommunication::EthCommunication(const char* ip,
                                   const char* netmask,
                                   const char* gateway,
                                   int port) :
    _socketAddress(ip, port) {
  _ip = ip;
  _port = port;
  _lastMsgType = msgType::NONE;
  _eth.set_network(_ip, netmask, gateway);
  _eth.set_blocking(false);
  _isActive = false;
}

bool EthCommunication::setup() {
  if (!_eth.connect()) {
    printf("Ethernet Connected! Listening on Port: %d\n", _port);
    _socket.open(&_eth);
    _socket.bind(_port);
    _socket.set_timeout(0);
    _isActive = true;
  } else {
    printf("Ethernet Connection Has Failed!\n");
  }
  return _isActive;
}

uint16_t EthCommunication::update_last_message() {
  // only updates last received msg if buffer is not empty
  uint16_t msgLen = 0;
  uint8_t recvMsgCheck[protoPositionSSL_size];
  std::memset(recvMsgCheck, 0, sizeof(_lastMsg));
  // checks buffer until it has no message
  while (true) {
    uint16_t checkMsgLen = _socket.recvfrom(&_socketAddress, recvMsgCheck, sizeof(_lastMsg));
    if (checkMsgLen == EMPTY_MESSAGE_LENGTH) {
      return msgLen;
    } else {
      msgLen = checkMsgLen;
      // copies not-empty message to _lastMsg
      std::memcpy(this->_lastMsg, recvMsgCheck, msgLen);
      // noMsgTimer.reset(); // reset no msg timer if msg is valid
    }
  }
}

void EthCommunication::decode_proto_position_msg(protoPositionSSL protoMessage) {
  this->_lastMsgType = msgType::POSITION;
  this->_packetPos.v.x = protoMessage.x;
  this->_packetPos.v.y = protoMessage.y;
  this->_packetPos.v.w = protoMessage.w;
  this->_packetPos.resetOdometry = protoMessage.resetOdometry;
  this->_packetPos.maxSpeed = protoMessage.max_speed;
  this->_packetPos.minSpeed = protoMessage.min_speed;
  this->_packetPos.type = static_cast<PositionType>(protoMessage.posType);
  this->_kick.front = static_cast<bool>(protoMessage.front);
  this->_kick.chip = static_cast<bool>(protoMessage.chip);
  this->_kick.charge = static_cast<bool>(protoMessage.charge);
  this->_kick.kickStrength = protoMessage.kickStrength / 10;
  this->_kick.dribbler = static_cast<bool>(protoMessage.dribbler);
  this->_kick.dribblerSpeed = protoMessage.dribSpeed / 10;
}

void EthCommunication::decode_proto_motors_msg(protoMotorsPWMSSL protoMessage) {
  this->_lastMsgType = msgType::SSL_MOTORS_PWM;
  this->_packetMotors.m1 = protoMessage.m1;
  this->_packetMotors.m2 = protoMessage.m2;
  this->_packetMotors.m3 = protoMessage.m3;
  this->_packetMotors.m4 = protoMessage.m4;
}

bool EthCommunication::eth_recv() {
  if (!_isActive) {
    return false;
  }
  uint16_t msgLen = this->update_last_message();
  protoPositionSSL protoMessage = protoPositionSSL_init_zero;
  if (msgLen == 0)
    return false;
  else {
    pb_istream_t recvStream = pb_istream_from_buffer(_lastMsg, msgLen);
    if (pb_decode(&recvStream, &protoPositionSSL_msg, &protoMessage)) {
      this->decode_proto_position_msg(protoMessage);
    }
    return true;
  }
}

msgType EthCommunication::get_last_msg_type() {
  return this->_lastMsgType;
}

void EthCommunication::get_position(RobotPosition& pos) {
  pos = _packetPos;
}

void EthCommunication::get_motors_pwm(Motors& motors) {
  motors = _packetMotors;
}

void EthCommunication::get_kick(KickFlags& isKick) {
  isKick.front = _kick.front;
  isKick.chip = _kick.chip;
  isKick.charge = _kick.charge;
  isKick.kickStrength = _kick.kickStrength;
  isKick.dribbler = _kick.dribbler;
  isKick.bypassIR = (_kick.front | _kick.chip) & _kick.charge;
  isKick.dribbler = _kick.dribbler;
  isKick.dribblerSpeed = _kick.dribblerSpeed;
}

protoOdometrySSL EthCommunication::encode_proto_odometry(RobotInfo robotInfo,
                                                         Vector robotOdometry,
                                                         Vector robotSpeed) {
  protoOdometrySSL protoMessage;
  protoMessage = protoOdometrySSL_init_zero;
  protoMessage.x = static_cast<double>(robotOdometry.x);
  protoMessage.y = static_cast<double>(robotOdometry.y);
  protoMessage.w = static_cast<double>(robotOdometry.w);
  protoMessage.hasBall = static_cast<bool>(robotInfo.ball);
  protoMessage.kickLoad = static_cast<double>(robotInfo.kickLoad);
  protoMessage.battery = static_cast<double>(robotInfo.battery);
  protoMessage.count = static_cast<int32_t>(robotInfo.count);
  protoMessage.vision_x = static_cast<double>(robotInfo.v.x);
  protoMessage.vision_y = static_cast<double>(robotInfo.v.y);
  protoMessage.vision_w = static_cast<double>(robotInfo.v.w);
  protoMessage.vx = static_cast<double>(robotSpeed.x);
  protoMessage.vy = static_cast<double>(robotSpeed.y);
  protoMessage.vw = static_cast<double>(robotSpeed.w);

  return protoMessage;
}

protoMotorsDataSSL EthCommunication::encode_proto_motors_data(Motors currentMotorsSpeed,
                                                              Motors motorsPWM,
                                                              Motors desiredMotorsSpeed,
                                                              double msgTime) {
  protoMotorsDataSSL protoMessage;
  protoMessage = protoMotorsDataSSL_init_zero;
  protoMessage.current_m1 = static_cast<double>(currentMotorsSpeed.m1);
  protoMessage.current_m2 = static_cast<double>(currentMotorsSpeed.m2);
  protoMessage.current_m3 = static_cast<double>(currentMotorsSpeed.m3);
  protoMessage.current_m4 = static_cast<double>(currentMotorsSpeed.m4);
  protoMessage.pwm_m1 = static_cast<double>(motorsPWM.m1);
  protoMessage.pwm_m2 = static_cast<double>(motorsPWM.m2);
  protoMessage.pwm_m3 = static_cast<double>(motorsPWM.m3);
  protoMessage.pwm_m4 = static_cast<double>(motorsPWM.m4);
  protoMessage.desired_m1 = static_cast<double>(desiredMotorsSpeed.m1);
  protoMessage.desired_m2 = static_cast<double>(desiredMotorsSpeed.m2);
  protoMessage.desired_m3 = static_cast<double>(desiredMotorsSpeed.m3);
  protoMessage.desired_m4 = static_cast<double>(desiredMotorsSpeed.m4);
  protoMessage.msgTime = static_cast<double>(msgTime);
  
  return protoMessage;
}

void EthCommunication::eth_sendto(const char* ip,
                                  uint16_t port,
                                  RobotInfo robotInfo,
                                  Vector robotOdometry,
                                  Vector robotSpeed) {
  if (!_isActive)
    return;
  uint8_t buffer[protoOdometrySSL_size];
  size_t message_length;

  protoOdometrySSL protoMessage = this->encode_proto_odometry(robotInfo, robotOdometry, robotSpeed);

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  if (pb_encode(&stream, &protoOdometrySSL_msg, &protoMessage)) {
    // message receiver breaks if full buffer is sent, so use stream.bytes_written property to limit
    // length
    message_length = stream.bytes_written;
    SocketAddress target(ip, port);
    // packets are decoded by protobuf, not nanopb, so send buffer, not nanopb stream
    _socket.sendto(target, buffer, message_length);
  } else {
    printf("encoding failed!\n");
  }
}

void EthCommunication::eth_sendto(const char* ip,
                                  uint16_t port,
                                  Motors currentMotorsSpeed,
                                  Motors motorsPWM,
                                  Motors desiredMotorsSpeed,
                                  double msgTime) {
  if (!_isActive)
    return;
  uint8_t buffer[protoMotorsDataSSL_size];
  size_t message_length;

  protoMotorsDataSSL protoMessage = this->encode_proto_motors_data(currentMotorsSpeed, 
                                                                   motorsPWM, 
                                                                   desiredMotorsSpeed, 
                                                                   msgTime);

  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  if (pb_encode(&stream, &protoMotorsDataSSL_msg, &protoMessage)) {
    // message receiver breaks if full buffer is sent, so use stream.bytes_written property to limit
    // length
    message_length = stream.bytes_written;
    SocketAddress target(ip, port);
    // packets are decoded by protobuf, not nanopb, so send buffer, not nanopb stream
    _socket.sendto(target, buffer, message_length);
  } else {
    printf("encoding failed!\n");
  }
}

void EthCommunication::printProtoPositionMsg(protoPositionSSL protoMessage) {
  printf("x: %f, y: %f, w: %f, type: %d\n",
         protoMessage.x,
         protoMessage.y,
         protoMessage.w,
         protoMessage.posType);
}
