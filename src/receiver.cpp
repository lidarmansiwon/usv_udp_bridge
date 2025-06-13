#include "usv_udp_bridge/receiver.hpp"

UdpReceiver::UdpReceiver() {
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    perror("Socket creation failed");
    exit(EXIT_FAILURE);
  }

  memset(&my_addr_, 0, sizeof(my_addr_));
  my_addr_.sin_family = AF_INET;
  my_addr_.sin_port = htons(RECV_PORT);
  my_addr_.sin_addr.s_addr = INADDR_ANY;

  if (bind(sockfd_, (struct sockaddr*)&my_addr_, sizeof(my_addr_)) < 0) {
    perror("Bind failed");
    close(sockfd_);
    exit(EXIT_FAILURE);
  }

  std::cout << "UDP Receiver initialized on port " << RECV_PORT << std::endl;
}

UdpReceiver::~UdpReceiver() {
  close(sockfd_);
}

// 함수 인자를 참조(&, reference)로 넘김
// NavigationMsg &nav_msg → 참조로 전달됨 (즉, 복사본이 아님)
// recvfrom() 함수가 nav_msg에 직접 데이터를 씀

bool UdpReceiver::receive(NavigationMsg &nav_msg) {
  socklen_t addr_len = sizeof(sender_addr_);
  // recvfrom()는 소켓에서 수신한 데이터를 nav_msg의 메모리 공간에 직접 저장
  // &nav_msg는 NavigationMsg 구조체의 주소이므로, 해당 주소에 데이터를 복사함.
  ssize_t len = recvfrom(sockfd_, &nav_msg, sizeof(nav_msg), 0,
                         (struct sockaddr*)&sender_addr_, &addr_len);
  // 결과로 nav_msg의 멤버 변수들(usv_id, x, y, psi 등)은 모두 수신된 값으로 갱신
  //수신된 데이터의 길이(len)가 NavigationMsg 구조체 크기(sizeof(nav_msg))와 동일하면 정상 수신으로 판단
  return len == sizeof(nav_msg);
}
