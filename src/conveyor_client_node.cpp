#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "conveyor_control/srv/conveyor_command.hpp"

using ConveyorCommand = conveyor_control::srv::ConveyorCommand;
using namespace std::chrono_literals;

class ConveyorClientNode : public rclcpp::Node
{
public:
  ConveyorClientNode()
  : Node("conveyor_client_node")
  {
    client_ = this->create_client<ConveyorCommand>("conveyor_command");

    // รอ service พร้อม
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for conveyor_command service...");
    }

    // example send command
    send_command(1);
    // send_command(0);
    // send_command(-1);
  }

private:
  void send_command(int8_t cmd)
  {
    auto request = std::make_shared<ConveyorCommand::Request>();
    request->command = cmd;

    auto result_future = client_->async_send_request(request,
      [this, cmd](rclcpp::Client<ConveyorCommand>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "Command %d sent successfully: %s", cmd, response->message.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to send command %d: %s", cmd, response->message.c_str());
        }
      });
  }

  rclcpp::Client<ConveyorCommand>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConveyorClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
