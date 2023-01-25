#include "lidar_manager/lidar_manager.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
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

LidarManager::LidarManager(const rclcpp::NodeOptions& node_options)
    : Node("lidar_manager", node_options)
{
    RCLCPP_INFO(this->get_logger(), "LidarManager node has been started.");

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V(rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile());

    m_lidarSubscriber = this->create_subscription<PointCloud2>(
        "pointcloud",
        QOS_RKL10V,
        [this](const PointCloud2::SharedPtr msg) -> void
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");
            while (iter_x != iter_x.end()) {
                // TODO: do something with the values of x, y, z
                std::cout << *iter_x << ", " << *iter_y << ", " << *iter_z << std::endl;
                ++iter_x; ++iter_y; ++iter_z;
            }

            // RCLCPP_INFO(this->get_logger(), "Timestamp of the message: %ld nanosec %ld", msg->stamp.sec, msg->stamp.nanosec);
            // RCLCPP_INFO(this->get_logger(), "Received args: [%.2f, %.2f]", m_argumentA, m_argumentB);
        }
    );

    m_commandClient = this->create_client<srv::SF45Command>("SF45Command");
    while (!m_commandClient->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }
}



void LidarManager::send_request()
{
  auto request = std::make_shared<srv::SF45Command::Request>();
  request->command = SF45Commandset::ScanEnable;

  using ServiceResponseFuture = rclcpp::Client<srv::SF45Command>::SharedFuture;
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

  auto lidarManagerNode = std::make_shared<LidarManager>();

  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(lidarManagerNode);
    lidarManagerNode->send_request();

    printf("Press Enter for next service call.\n");
    if (pull_trigger() == false) {
      rclcpp::shutdown();
      return 0;
    }
  }
}