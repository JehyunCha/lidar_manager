#include "lidar_manager/lidar_commander.hpp"
#include "ConstForSF45.hpp"

#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>

using namespace std::chrono_literals;

LidarCommander::LidarCommander(const rclcpp::NodeOptions& node_options)
    : Node("lidar_commander", node_options)
{
    RCLCPP_INFO(this->get_logger(), "LidarCommander node has been started.");

    m_commandClient = this->create_client<SF45Command>("sf45_command");
    while (!m_commandClient->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
}



void LidarCommander::send_request()
{
  auto request = std::make_shared<SF45Command::Request>();
  request->command = static_cast<uint16_t>(SF45Commandset::ScanEnable);

  using ServiceResponseFuture = rclcpp::Client<SF45Command>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %d", response->result);
      return;
    };

  auto future_result = m_commandClient->async_send_request(request, response_received_callback);
}



int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}



int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}



bool pull_trigger()
{
  const uint8_t KEY_ENTER = 10;
  while (1) {
    if (kbhit()) {
      uint8_t c = getch();
      if (c == KEY_ENTER) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto lidarCommanderNode = std::make_shared<LidarCommander>();

  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(lidarCommanderNode);
    lidarCommanderNode->send_request();

    printf("Press Enter for next service call.\n");
    if (pull_trigger() == false) {
      rclcpp::shutdown();
      return 0;
    }
  }
}