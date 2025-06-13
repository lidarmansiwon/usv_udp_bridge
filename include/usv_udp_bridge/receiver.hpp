#ifndef RECEIVER__RECEIVER_HPP_
#define RECEIVER__RECEIVER_HPP_

#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <data_struct/data_struct.hpp>

#define RECV_PORT 5005

class UdpReceiver {
public:
  UdpReceiver();
  ~UdpReceiver();

  // 수신 함수: 성공하면 true 반환
  bool receive(NavigationMsg &nav_msg);

private:
  int sockfd_;
  struct sockaddr_in my_addr_, sender_addr_;
};

#endif  // RECEIVER__RECEIVER_HPP_
