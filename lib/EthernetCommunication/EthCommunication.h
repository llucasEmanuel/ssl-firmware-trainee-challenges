#include <mbed.h>
#include <commConfig.h>
#include <commTypes.h>
#include <EthernetInterface.h>
#include <iostream>

#include <CommTypes.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

#define EMPTY_MESSAGE_LENGTH 62535

class EthCommunication {
 public:
  // Class constructor
  EthCommunication(const char* ip, const char* netmask, const char* gateway, int port);

  // Class destructor
  // ~EthCommunication();

  bool setup();

  // FOR RECEIVING MESSAGES
  uint16_t update_last_message();
  void decode_proto_position_msg(protoPositionSSL protoMessage);
  void decode_proto_motors_msg(protoMotorsPWMSSL protoMessage);
  bool eth_recv();
  void get_position(RobotPosition& pos);
  void get_motors_pwm(Motors& motors);
  msgType get_last_msg_type();
  void get_kick(KickFlags& isKick);

  // FOR SENDING MESSAGES
  protoOdometrySSL encode_proto_odometry(RobotInfo robotInfo, 
                                         Vector robotOdometry, 
                                         Vector robotSpeed);
  protoMotorsDataSSL encode_proto_motors_data(Motors currentMotorsSpeed,
                                              Motors motorsPWM,
                                              Motors desiredMotorsSpeed,
                                              double msgTime);
  void eth_sendto(const char* ip,
                  uint16_t port,
                  RobotInfo robotInfo,
                  Vector robotOdometry,
                  Vector robotSpeed);
  void eth_sendto(const char* ip,
                  uint16_t port,
                  Motors currentMotorsSpeed,
                  Motors motorsPWM,
                  Motors desiredMotorsSpeed,
                  double msgTime);

  // FOR DEBUGGING
  void printProtoPositionMsg(protoPositionSSL protoMessage);

 private:
  bool _isActive;
  const char* _ip;
  int _port;
  EthernetInterface _eth;
  UDPSocket _socket;
  SocketAddress _socketAddress;
  uint8_t _lastMsg[protoPositionSSL_size];
  msgType _lastMsgType;
  RobotPosition _packetPos;
  KickFlags _kick;
  Motors _packetMotors;
};
