#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <demo_eps/msg/bcdu_status.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <future>

namespace demo_eps
{

struct BatteryInfo
{
  double voltage{0.0};
  bool discharging{false};
  rclcpp::Time last_update;
};

class BcduNode : public rclcpp::Node
{
public:
  explicit BcduNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Params
  double max_discharge_current_A_{127.0};
  double max_charge_current_A_{65.0};
  double regulation_min_V_{130.0};
  double regulation_max_V_{180.0};
  double regulation_voltage_{150.0};

  // SSU thresholds (hysteresis)
  double ssu_charge_enter_v_{160.0};     // enter CHARGE when SSU >= this
  double ssu_discharge_enter_v_{152.0};  // enter DISCHARGE when SSU <= this
  double ssu_nominal_v_{160.0};          // what we feed to MBSU when charging

  int num_channels_{12};
  int orus_per_channel_{2};

  // State
  std::mutex mtx_;
  std::map<int, BatteryInfo> battery_state_;
  std::map<int, rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
  std::map<int, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> charge_clients_;
  std::map<int, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> discharge_clients_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ssu_sub_;
  double ssu_voltage_{0.0};
  rclcpp::Time ssu_last_update_;
  std::string mode_{"idle"};  // "charge" | "discharge" | "idle"
  bool transition_in_progress_{false};

  // I/O
  rclcpp::Publisher<demo_eps::msg::BCDUStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mbsu_voltage_pub_;

  // Control loop
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Duration stale_timeout_{rclcpp::Duration::from_seconds(5.0)};

  // Callbacks / helpers
  void ssuVoltageCb(const std_msgs::msg::Float64::SharedPtr msg);
  void controlTick();
  void enterCharge();
  void enterDischarge();
  void publishToMbsu(double volts);
  void publishDiag(uint8_t level, const std::string &name, const std::string &msg);
  void batteryCallback(int id, const sensor_msgs::msg::BatteryState::SharedPtr msg);

  void publishStatus(const std::string &mode, bool fault = false, const std::string &fault_msg = "");
  std::vector<int> healthyBatteryIdsLocked() const;
};

}  // namespace demo_eps
