#ifndef SENDOR__SENDOR_HPP_
#define SENDOR__SENDOR_HPP_

#include <memory>
#include <string>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "mk3_msgs/msg/navigation_type.hpp"

class UdpSendor : public rclcpp::Node
{
public:
  using NavigationType = mk3_msgs::msg::NavigationType;
  
  explicit UdpSendor(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~UdpSendor();

private:
  void navigation_callback(const NavigationType::SharedPtr msg);

  rclcpp::Subscription<NavigationType>::SharedPtr navigation_subscription_;

  int sockfd_;
  struct sockaddr_in dest_addr_;
  std::string receiver_ip_;

  uint32_t usv_id_; 

  struct NavigationMsg {
    uint32_t usv_id;
    float x, y, psi, u, v, r, w;
  };
};

#endif  // SENDOR__SENDOR_HPP_
