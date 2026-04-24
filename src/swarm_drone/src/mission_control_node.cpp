//Mission_control_node takes in subscribes to temperature and position data and makes waypoint decisions based on that data.
//It will send those waypoints to the offboard control node to be executed by the drone.

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class MissionControlNode : public rclcpp::Node
{
public:
  MissionControlNode() : Node("mission_control_node")
  {
         // Initialize heatmap grid (e.g., 100x100, adjust as needed)
        heat_map.resize(grid_size, std::vector<float>(grid_size, 0.0f));

        //Create waypoint publisher
        target_position_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          "/mission/target_position", 10); 

        //publish every 100ms
        timer_ = this->create_wall_timer(
          100ms, std::bind(&MissionControlNode::updateState, this));

        //local position subscription
        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", 
            rclcpp::SensorDataQoS(), 
            std::bind(&MissionControlNode::position_callback, this, std::placeholders::_1)
            );

        //temperature subscriber
        temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
        "/drone/temperature", 10,
        std::bind(&MissionControlNode::temperature_callback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "Mission control node started");
  }

private:

  //drone state machine
  enum class DroneState {
    TAKEOFF,
    SEARCH,
    PERIMETER,
    RETURN_HOME
  };

  DroneState current_state = DroneState::TAKEOFF; //initialize state to takeoff

  void updateState(){
    switch(current_state){
      case DroneState::TAKEOFF:
        // Handle takeoff logic
        publish_waypoint_direct(takeoff_waypoint_); 
        if(check_waypoint_reached({takeoff_waypoint_})){
            current_state = DroneState::SEARCH; //switch to search state once takeoff waypoint reached
        }
        break;

      case DroneState::SEARCH:
        // Handle search logic
        //check if specific temperature is reached, if so then switch to perimeter state
        if(current_temp_ >= 50.0){ //example temp threshold for hot spot
            current_state = DroneState::PERIMETER;
            break;
        }
        if(check_waypoint_reached(search_waypoints)){
            advance_waypoint(search_waypoints); //move to next search waypoint once current one reached
        }
        publish_waypoint(search_waypoints); //keep publishing current search waypoint until reached

        break; 

      case DroneState::PERIMETER:
        // Handle perimeter logic
        //Circle around detected hot spot by publishing waypoints in a circular pattern around the location of the hot spot 
        //until a starting position is found (starting position being the start of perimeter tracing), then return home

        generate_perimeter_waypoint(); //generate waypoints in a circular pattern around the detected hot spot for perimeter tracing
        publish_waypoint(perimeter_waypoints); //publish perimeter waypoints to be executed by drone

        if(!perimeter_start_set_){
            perimeter_start_pos_ = {current_pos_.x, current_pos_.y, current_pos_.z};
            perimeter_start_set_ = true;
        }

        //calculate deltas
        float dx = std::abs(current_pos_.x - perimeter_start_pos_.x);
        float dy = std::abs(current_pos_.y - perimeter_start_pos_.y);
        float dz = std::abs(current_pos_.z - perimeter_start_pos_.z); 

        if(dx <= 0.25 && dy <= 0.25 && dz <= 0.25){ //return true when within 0.25 meters of waypoint
            current_state = DroneState::RETURN_HOME; //if we have come full circle around the hot spot, return home
          break;
        }

        break;
      case DroneState::RETURN_HOME:
        // Handle return home logic
        break;
    }
    
  }
  

  // ================ waypoint functions and variables =================

    struct xyzPoint{
        float x, y, z;
    };

    std::vector<xyzPoint> search_waypoints = {  //waypoints for lawnmower algorithm
        {0,0, -5}, //takeoff
        {0,10, -5}, 
        {2,10, -5}, 
        {2,10, -5},
        {2,8, -5},
        {0,8, -5},
        {0,6, -5},
        {2,6, -5},
        {2,4, -5},
        {0,4, -5},
        {0,2, -5},
        {2,2, -5},
        {2,0, -5},
        {0,0,-5} //return to start
    };

    void publish_waypoint(std::vector<xyzPoint> waypoints){
        auto target = get_current_waypoint(waypoints);

        //create trajectory setpoint msg
        auto msg = px4_msgs::msg::TrajectorySetpoint();
        msg.position = {target.x, target.y, target.z};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        target_position_pub_->publish(msg);
    }

    //function to publish a specific waypoint instead of using waypoint vector
    void publish_waypoint_direct(xyzPoint target){ 
        auto msg = px4_msgs::msg::TrajectorySetpoint();
        msg.position = {target.x, target.y, target.z};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        target_position_pub_->publish(msg);
    }

    void advance_waypoint(std::vector<xyzPoint> waypoints){
        if(check_waypoint_reached(waypoints)){
            waypointIndex++;
        }
    }

    xyzPoint get_current_waypoint(std::vector<xyzPoint> waypoints){
        if(waypointIndex >= waypoints.size()){
            return waypoints[waypoints.size() - 1]; //return last waypoint 
        }
        return waypoints[waypointIndex];
    }

    bool check_waypoint_reached(std::vector<xyzPoint> waypoints){
      auto const target = get_current_waypoint(waypoints);

      //calculate deltas
      float dx = std::abs(current_pos_.x - target.x);
      float dy = std::abs(current_pos_.y - target.y);
      float dz = std::abs(current_pos_.z - target.z); 

      if(dx <= 0.25 && dy <= 0.25 && dz <= 0.25){ //return true when within 0.25 meters of waypoint
          return true;
      }
      return false;
    }

  size_t waypointIndex = 0;

  xyzPoint takeoff_waypoint_ = {0.0, 0.0, -5.0};  // dedicated takeoff point

  // ===================== temperature methods and variables ===================

    //temperature callback 
  void temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg){ 
    current_temp_ = msg->temperature; //update temperature 
    update_heat_map(current_pos_.x, current_pos_.y, current_temp_);  // Update heatmap with current position and temp
}

  float current_temp_ = 0.0; //initialize temperature

  // Grid where each cell stores the highest temp reading at that position
  std::vector<std::vector<float>> heat_map;

  float cell_size_ = 1.0; // each cell represents 1m x 1m area
  int grid_size = 100; // 100x100 grid representing 100m x 100m area


  // Update it every time you get a temperature reading
  void update_heat_map(float x, float y, float temp){
      int grid_x = (int)(x / cell_size_);
      int grid_y = (int)(y / cell_size_);

      // Bounds check 
      if(grid_x < 0 || grid_x >= grid_size || grid_y < 0 || grid_y >= grid_size) return;

      heat_map[grid_x][grid_y] = std::max(heat_map[grid_x][grid_y], temp);
  }


  // ===================== position methods and variables ===================
  void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg){ //always updating position in the background
        current_pos_ = *msg;
    }

  xyzPoint perimeter_start_pos_; //stores starting position of perimeter tracing to know when to stop perimeter tracing and return home

  //generate waypoints in a circular pattern around the detected hot spot for perimeter tracing
  void generate_perimeter_waypoint(){
        //if temp > high_threshold (e.g 70°C) → move outward
        //if temp < low_threshold (e.g 50°C) → move inward
        //always move laterally
        if(current_temp_ >= 70.0){ //example high temp threshold
            //generate waypoints in a circular pattern with increasing radius
        } else if (current_temp_ <= 50.0){ //example low temp threshold
            //generate waypoints in a circular pattern with decreasing radius
        } else {
            //generate waypoints in a circular pattern with constant radius
        }
      }
  

  //waypoints for perimeter tracing, generated dynamically based on location of detected hot spot and added to search waypoints once generated
  std::vector<xyzPoint> perimeter_waypoints = {}; 

  bool perimeter_start_set_ = false;



  //===================== publishers and subscribers ===================
  //subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_; 
  px4_msgs::msg::VehicleLocalPosition current_pos_;  // stores latest position
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  
  //publishers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr target_position_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionControlNode>());
  rclcpp::shutdown();
  return 0;
}