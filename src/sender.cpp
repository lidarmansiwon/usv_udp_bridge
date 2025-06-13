#include "usv_udp_bridge/sender.hpp"
#include <arpa/inet.h>
#include <unistd.h>
#include <regex>

#define PORT 5005

UdpSendor::UdpSendor(const rclcpp::NodeOptions & node_options)
: Node("udp_sender", node_options)
{
  this->declare_parameter<std::string>("receiverIP", "192.168.0.215");
  this->declare_parameter<std::string>("navigation_topic", "/agent0/navigation_data");

  receiver_ip_ = this->get_parameter("receiverIP").as_string();
  std::string navigation_topic = this->get_parameter("navigation_topic").as_string();
  
  std::regex agent_regex("agent(\\d+)");
  std::smatch matches;

  if (std::regex_search(navigation_topic, matches, agent_regex) && matches.size() > 1) {
    usv_id_ = std::stoul(matches[1].str());
    RCLCPP_INFO(this->get_logger(), "Extracted USV ID: %u from topic: %s", usv_id_, navigation_topic.c_str());
  } else {
    usv_id_ = 0;
    RCLCPP_WARN(this->get_logger(), "No agent ID found in topic: %s. Using default ID: %u", navigation_topic.c_str(), usv_id_);
  }

  // UDP 소켓 초기화
  sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
    rclcpp::shutdown();
  }

  memset(&dest_addr_, 0, sizeof(dest_addr_));
  dest_addr_.sin_family = AF_INET;
  dest_addr_.sin_port = htons(PORT);
  inet_pton(AF_INET, receiver_ip_.c_str(), &dest_addr_.sin_addr);

  RCLCPP_INFO(this->get_logger(), "UDP Sender initialized. Sending to %s:%d", receiver_ip_.c_str(), PORT);

  // ROS2 Subscription 설정
  navigation_subscription_ = this->create_subscription<NavigationType>(
    navigation_topic,
    10,
    std::bind(&UdpSendor::navigation_callback, this, std::placeholders::_1));
}

UdpSendor::~UdpSendor() {
  close(sockfd_);
}

void UdpSendor::navigation_callback(const NavigationType::SharedPtr msg)
{
  // 데이터 수신 시 바로 UDP로 전송
  NavigationMsg nav_msg {
    usv_id_,  // 반드시 멤버변수를 사용해야 함!
    static_cast<float>(msg->x),
    static_cast<float>(msg->y),
    static_cast<float>(msg->psi),
    static_cast<float>(msg->u),
    static_cast<float>(msg->v),
    static_cast<float>(msg->r),
    static_cast<float>(msg->w)
  };
  ssize_t sent = sendto(sockfd_, &nav_msg, sizeof(nav_msg), 0,
                        (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
  
  if (sent < 0) {
    RCLCPP_ERROR(this->get_logger(), "UDP Send failed");
  } else {
    RCLCPP_INFO(this->get_logger(), "Sent nav data (ID %u): [%f, %f, %f, %f, %f, %f, %f]",
                nav_msg.usv_id, nav_msg.x, nav_msg.y, nav_msg.psi,
                nav_msg.u, nav_msg.v, nav_msg.r, nav_msg.w);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpSendor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
