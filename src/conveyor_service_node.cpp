#include <chrono>
#include <memory>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "conveyor_control/srv/conveyor_command.hpp"  // generated header

using ConveyorCommand = conveyor_control::srv::ConveyorCommand;
using std::placeholders::_1;
using std::placeholders::_2;

class ConveyorServiceNode : public rclcpp::Node
{
public:
  ConveyorServiceNode()
  : Node("conveyor_service_node")
  {
    udp_ip_ = "10.1.100.222";
    udp_port_ = 30001;

    // create UDP socket
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
      rclcpp::shutdown();
      return;
    }

    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(udp_port_);
    inet_pton(AF_INET, udp_ip_.c_str(), &server_addr_.sin_addr);

    service_ = this->create_service<ConveyorCommand>(
      "conveyor_command",
      std::bind(&ConveyorServiceNode::handle_command, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Conveyor UDP Service Node started.");
  }

  ~ConveyorServiceNode()
  {
    close(sock_);
  }

private:
  void handle_command(
    const std::shared_ptr<ConveyorCommand::Request> request,
    std::shared_ptr<ConveyorCommand::Response> response)
  {
    int8_t cmd = request->command;

    if (cmd != -1 && cmd != 0 && cmd != 1) {
      response->success = false;
      response->message = "Invalid command, allowed only -1, 0, 1";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::string cmd_str = std::to_string(cmd);
    ssize_t sent_bytes = sendto(sock_, cmd_str.c_str(), cmd_str.size(), 0,
                                (struct sockaddr *)&server_addr_, sizeof(server_addr_));
    if (sent_bytes == -1) {
      response->success = false;
      response->message = "Failed to send UDP command";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    } else {
      response->success = true;
      response->message = "Command sent: " + cmd_str;
      RCLCPP_INFO(this->get_logger(), "Sent UDP command: %s", cmd_str.c_str());
    }
  }

  int sock_;
  struct sockaddr_in server_addr_;
  std::string udp_ip_;
  int udp_port_;
  rclcpp::Service<ConveyorCommand>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConveyorServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
