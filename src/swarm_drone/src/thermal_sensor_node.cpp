#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class ThermalSensorNode : public rclcpp::Node
{
public:
  ThermalSensorNode() : Node("thermal_sensor_node")
  {
    // Create a publisher on /thermal_data topic
    publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "thermal_data", 10);

    // Publish every 500ms
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ThermalSensorNode::publish_temperature, this));

    RCLCPP_INFO(this->get_logger(), "Thermal sensor node started");
  }

private:
  void publish_temperature()
  {
    // Fake temperature reading (replace with real sensor later)
    auto msg = sensor_msgs::msg::Temperature();
    msg.header.stamp = this->now();
    msg.temperature = 20.0 + (rand() % 100);  // 20-120°C range
    msg.variance = 0.1;

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing temp: %.1f°C", msg.temperature);
  }

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSensorNode>());
  rclcpp::shutdown();
  return 0;
}