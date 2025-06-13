#include "usv_udp_bridge/receiver.hpp"
#include <iostream>

int main() {
  UdpReceiver receiver;
  NavigationMsg msg;

  while (true) {
    if (receiver.receive(msg)) {
      std::cout << "Received nav data (ID " << msg.usv_id << "): ["
                << msg.x << ", " << msg.y << ", " << msg.psi << ", "
                << msg.u << ", " << msg.v << ", " << msg.r << ", " << msg.w << "]"
                << std::endl;
    } else {
      std::cerr << "Failed to receive data." << std::endl;
    }
  }

  return 0;
}
