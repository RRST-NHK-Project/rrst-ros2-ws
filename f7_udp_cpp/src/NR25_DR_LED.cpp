// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "include/UDP.hpp"

// IPアドレスとポートの指定
std::string udp_ip = "192.168.0.215"; // 送信先IPアドレス、宛先マイコンで設定したIPv4アドレスを指定
int udp_port = 5000;                  // 送信元ポート番号、宛先マイコンで設定したポート番号を指定

using std::placeholders::_1;


class LEDListener : public rclcpp::Node
{
  public:
   LEDListener(const std::string &ip, int port)
    : Node("nr25_dr_led"), udp_(ip, port)
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/dr_led", 10, std::bind(&LEDListener::sensor_callback, this, _1));
    }

  private:
    void sensor_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard: ");
      for (const auto& data : msg->data) {
        RCLCPP_INFO(this->get_logger(), "%d", data);
       }

        std::vector<int> meter = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        
        
        meter[1] = msg->data[0];
        meter[2] = msg->data[1];
        meter[3] = msg->data[2];
        meter[4] = msg->data[3];
        meter[5] = msg->data[4];
        meter[6] = msg->data[5];
        meter[7] = msg->data[6];
        meter[8] = msg->data[7];
        

  
      udp_.send(meter);

    }
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    UDP udp_;
};

int main(int argc, char *argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LEDListener>(udp_ip,udp_port));
    rclcpp::shutdown();
    return 0;
}
