#ifndef BATTERY_MANAGER_HPP_
#define BATTERY_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

// BehaviorTree.CPP v3 includes
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

class BatteryManager : public rclcpp::Node
{
public:
  BatteryManager();

public:
  struct BatteryUnit
  {
    std::string id;
    float voltage;
    bool discharging;
    std::string location;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::BatteryState>> publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr discharge_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr charge_service;

    // Add this line:
    std::string prev_state = "idle";
  };


  int num_channels_;
  std::vector<BatteryUnit> batteries_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr bt_tick_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr log_timer_;
  std::unordered_map<std::string, std::shared_ptr<BatteryUnit>> battery_lookup;

  // Behavior Tree
  BT::Tree tree_;
  BT::BehaviorTreeFactory factory_;
  bool bt_triggered_ = false;

  void update_and_publish(const std::shared_ptr<BatteryUnit>& unit);

  static constexpr float BATTERY_MAX_VOLTAGE = 120.0f;
  static constexpr float BATTERY_MIN_VOLTAGE = 80.0f;
  static constexpr float BATTERY_WARNING_VOLTAGE = 60.0f;
  static constexpr float BATTERY_CRITICAL_VOLTAGE = 30.0f;
};

#endif  // BATTERY_MANAGER_HPP_
