#include <memory>
#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/temperature.hpp>

using namespace std::chrono_literals;

class OffBoardControlNode : public rclcpp::Node
{
    public: 
    OffBoardControlNode() : Node("offboard_control_node"), offboard_setpoint_counter_(0)
    {
        control_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        trajec_set_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", 
        rclcpp::SensorDataQoS(), 
        std::bind(&OffBoardControlNode::position_callback, this, std::placeholders::_1)
        );


        //subscribe to target position
        target_pos_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/mission/target_position", 10,
        std::bind(&OffBoardControlNode::target_position_callback, this, std::placeholders::_1));

        // Publish every 100ms/ 10Hz
        timer_ = this->create_wall_timer(100ms, std::bind(&OffBoardControlNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Offboard control node started");
    }

    private:
    void timer_callback(){
        offboard_setpoint_counter_++;

        //TAKEOFF 
        if(offboard_setpoint_counter_ == 10){ //takeoff after 1 second 100ms * 10 = 1 sec
            // Switch to offboard mode
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); //1.0 indicates custom mode. 6.0 is the specific ID for Offboard mode in PX4
            // Arm the drone
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // 1.0 to arm and 0.0 to disarm
        }

        //RCLCPP_INFO(this->get_logger(), "Moving to Waypoint: %ld", waypointIndex);
        
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
    
}

    //tell flight controller which setpoint field (postion in this case) is active 
    void publish_offboard_control_mode(){
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.position = true; //actively controlling postition
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;

        control_pub_->publish(msg);
    }

    //defines the desired position, velocity, acceleration, jerk, and yaw setpoints to the controller
    // use 'ros2 interface show px4_msgs/msg/TrajectorySetpoint' to see msg fields
    void publish_trajectory_setpoint(){
        auto target = current_target_; //grabs waypoint from mission control node

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {target.x, target.y, target.z};  
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajec_set_pub_->publish(msg);
    }

    //Sends high-level instructions to PX4, such as Arming, changing flight mode, or triggering actions like landing. 
    //Used for one-time events like switching modes. 
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0){
        px4_msgs::msg::VehicleCommand msg{};
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;      // Typically 1 for the vehicle
        msg.target_component = 1;   // Typically 1 for the flight controller
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        vehicle_cmd_pub_->publish(msg); 
    }


    struct xyzPoint { //struct for waypoint coordinate pairs
        float x, y, z;
    };


    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){ //always updating position in the background
        current_pos_ = *msg;
    }

    void target_position_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg){
        // Update the current target position based on the received message
        current_target_.x = msg->position[0];
        current_target_.y = msg->position[1];
        current_target_.z = msg->position[2];
    }

    xyzPoint current_target_ = {0.0, 0.0, 0.0};  // default target position


    //Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_; 
    px4_msgs::msg::VehicleLocalPosition current_pos_;  // stores latest position
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr target_pos_sub_;

    //publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajec_set_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t offboard_setpoint_counter_; 
    bool landing_ = false;
     

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffBoardControlNode>());
    rclcpp::shutdown();
    return 0;
}