#ifndef BATTERY_MANAGER_HPP_
#define BATTERY_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

class BatteryManager : public rclcpp::Node
{
public:
  BatteryManager();

private:
  struct BatteryUnit
  {
    std::string id;                                // battery_bms_<i>
    std::string location;                          // e.g., channel_1
    float voltage = BATTERY_MAX_VOLTAGE;           // starting at full charge
    bool discharging = false;                      // discharging state

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
  };

  int num_channels_;
  std::vector<BatteryUnit> batteries_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;

  void update_and_publish(const std::shared_ptr<BatteryUnit>& unit);

  static constexpr float BATTERY_MAX_VOLTAGE = 120.0f;
  static constexpr float BATTERY_MIN_VOLTAGE = 80.0f;
};

#endif  // BATTERY_MANAGER_HPP_
