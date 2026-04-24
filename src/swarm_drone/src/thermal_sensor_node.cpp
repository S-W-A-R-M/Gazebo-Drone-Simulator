#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class ThermalSensorNode : public rclcpp::Node
{
public:
  ThermalSensorNode() : Node("thermal_sensor_node")
  {
    // Create a publisher on /thermal_data topic
    temp_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "/drone/temperature", 10);

    // Publish every 500ms
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ThermalSensorNode::publish_temperature, this));
      
    //local position subsription   
    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", 
        rclcpp::SensorDataQoS(), 
        std::bind(&ThermalSensorNode::position_callback, this, std::placeholders::_1)
        );

    RCLCPP_INFO(this->get_logger(), "Thermal sensor node started");
  }

private:
  void publish_temperature()
  {
    // Fake temperature reading (replace with real sensor later)
    // Fake heat source centered at (0, 0)
    float dx = current_pos_.x - 0.0;
    float dy = current_pos_.y - 0.0;
    float temperature = 20.0 + 80.0 * exp(- (dx*dx + dy*dy) / 50.0); //heat blob

    //create temperature msg
    auto msg = sensor_msgs::msg::Temperature();
    msg.header.stamp = this->now();
    msg.temperature = temperature;
    msg.variance = 0.0;

    temp_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing temp: %.1f°C", msg.temperature);
  }

    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){ //always updating in the background
    current_pos_ = *msg;
    }

  //position subscriber
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_; 
  px4_msgs::msg::VehicleLocalPosition current_pos_;  // stores latest position

  //temp publisher
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThermalSensorNode>());
  rclcpp::shutdown();
  return 0;
}