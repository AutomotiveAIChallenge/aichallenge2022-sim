#include "aichallenge_score_msgs/msg/score.hpp"

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_vehicle_msgs/msg/engage.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cstdio>
#include <iostream>
#include <fstream>

class Score : public rclcpp::Node
{
public:
  Score() : system_clock(RCL_ROS_TIME), Node("score_node")
  {
    // publisher
    score_publisher =
      this->create_publisher<aichallenge_score_msgs::msg::Score>("/score/result", 1);

    // subscriber
    start_subscriber = this->create_subscription<std_msgs::msg::Int64>(
      "/score/start_time", 1, std::bind(&Score::startTimeCallback, this, std::placeholders::_1));
    check_subscriber = this->create_subscription<std_msgs::msg::Int64>(
      "/score/check_time", 1, std::bind(&Score::checkTimeCallback, this, std::placeholders::_1));
    end_subscriber = this->create_subscription<std_msgs::msg::Int64>(
      "/score/end_time", 1, std::bind(&Score::endTimeCallback, this, std::placeholders::_1));
    vehicle_speed_subscriber = this->create_subscription<std_msgs::msg::Float64>(
      "/score/vehicle_speed", 1, std::bind(&Score::vehicleSpeedCallback, this, std::placeholders::_1));
    vehicle_collision_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/score/collision", 1, std::bind(&Score::vehicleCollisionCallback, this, std::placeholders::_1));
    stop_point_collision_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/score/not_stop_collision", 1,
      std::bind(&Score::stopPointCollisionCallback, this, std::placeholders::_1));
    timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0),
    std::bind(&Score::timerCallback, this));
  }

private:
  // Subscribe The Start Time
  void startTimeCallback(const std_msgs::msg::Int64 msg)
  {
    std::cout << "Start:" << msg.data << std::endl;
    start_time = msg.data;
    has_start_time_set = true;
  }
  // Subscribe The Check Point Time
  void checkTimeCallback(const std_msgs::msg::Int64 msg)
  {
    std::cout << "Check:" << msg.data << std::endl;
    check_count++;
  }

  // Subscribe The End Time
  void endTimeCallback(const std_msgs::msg::Int64 msg)
  {
    if (has_published_result)
      return;

    std::cout << "Finish:" << msg.data << std::endl;
    end_time = msg.data;
    score_time = end_time - start_time;
    result_msg.score = score_time;
    result_msg.has_finished = check_count == 2;
    result_msg.has_park_failed = false;
    result_msg.has_collided = false;
    result_msg.has_exceeded_speed_limit = false;
    result_msg.check_point_count = check_count;
    score_publisher->publish(result_msg);
    writeResultJson(result_msg);

    has_published_result = true;
  }
  //Subscribe The Vehicle Speed 
  void vehicleSpeedCallback(const std_msgs::msg::Float64 msg)
  {
    if (has_published_result)
      return;

    // 50km/h to m/s
    if (msg.data < 50 * 1000 / 60 / 60)
      return;

    result_msg.score = 0;
    result_msg.has_finished = false;
    result_msg.has_park_failed = false;
    result_msg.has_collided = false;
    result_msg.has_exceeded_speed_limit = true;
    result_msg.check_point_count = check_count;
    score_publisher->publish(result_msg);
    writeResultJson(result_msg);

    has_published_result = true;
  }

  //Subscribe The  Vehicle Collision Info
  void vehicleCollisionCallback(const geometry_msgs::msg::PoseStamped msg)
  {
    if (has_published_result)
      return;

    std::cout << "Collision:" << msg.pose.position.x << "," << msg.pose.position.y << ","
              << msg.pose.position.z << "," << std::endl;

    result_msg.score = 0;
    result_msg.has_finished = false;
    result_msg.has_park_failed = false;
    result_msg.has_exceeded_speed_limit = false;
    result_msg.has_collided = true;
    result_msg.check_point_count = check_count;
    score_publisher->publish(result_msg);
    writeResultJson(result_msg);

    has_published_result = true;
  }

  //Subscribe The  Stop Point Collision Info
  void stopPointCollisionCallback(const geometry_msgs::msg::PoseStamped msg)
  {
    if (has_published_result)
      return;

    std::cout << "Disregard of traffic signals or temporary stops:" << msg.pose.position.x << ","
              << msg.pose.position.y << "," << msg.pose.position.z << "," << std::endl;

    result_msg.score = 0;
    result_msg.has_finished = false;
    result_msg.has_park_failed = true;
    result_msg.has_collided = true;
    result_msg.has_exceeded_speed_limit = false;
    result_msg.check_point_count = check_count;
    score_publisher->publish(result_msg);
    writeResultJson(result_msg);

    has_published_result = true;
  }

  void timerCallback() {
    if (!has_start_time_set)
      return;

    auto sec = system_clock.now().seconds();
    if (!is_start_sec_initialized) {
      start_sec = sec;
      is_start_sec_initialized = true;
    }

    if (sec - start_sec < 10 * 60)
      return;

    if (has_published_result)
      return;

    std::cout << "Timeout" << std::endl;

    result_msg.score = 0;
    result_msg.has_finished = false;
    result_msg.has_park_failed = false;
    result_msg.has_collided = false;
    result_msg.has_exceeded_speed_limit = false;
    result_msg.check_point_count = check_count;
    score_publisher->publish(result_msg);
    writeResultJson(result_msg);

    has_published_result = true;
  }

  void writeResultJson(const aichallenge_score_msgs::msg::Score& score_msg) {
    std::ofstream ofs("score.json");
    ofs << "{" << std::endl;
    ofs << "  \"score\": " << score_msg.score << "," << std::endl;
    ofs << "  \"hasFinished\": " << score_msg.has_finished << "," << std::endl;
    ofs << "  \"hasCollided\": " << score_msg.has_collided << "," << std::endl;
    ofs << "  \"hasExceededSpeedLimit\": " << score_msg.has_exceeded_speed_limit << "," << std::endl;
    ofs << "  \"checkPointCount\": " << score_msg.check_point_count << "," << std::endl;
    ofs << "}" << std::endl;
    ofs.close();
  }

  // publisher
  rclcpp::Publisher<aichallenge_score_msgs::msg::Score>::SharedPtr score_publisher;

  // subscriber
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr start_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr check_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr end_subscriber;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vehicle_speed_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_collision_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr stop_point_collision_subscriber;

  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Clock system_clock;
  double start_sec = 0;
  bool is_start_sec_initialized = false;

  // msg
  aichallenge_score_msgs::msg::Score result_msg = aichallenge_score_msgs::msg::Score();

  // other
  int start_time = 0;
  bool has_start_time_set = false;
  int end_time = 0;
  int score_time = 0;
  int check_count = 0;
  bool has_published_result = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Score>());
  rclcpp::shutdown();
  return 0;
}
