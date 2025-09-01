#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_srvs/srv/trigger.hpp>  // <-- REQUIRED

#include "demo_eps/action/bcdu_operation.hpp"
#include "demo_eps/msg/bcdu_status.hpp"
#include "demo_eps/srv/set_bcdu_state.hpp"

#include <mutex>
#include <unordered_map>

namespace demo_eps
{

class BcduNode : public rclcpp::Node
{
public:
  using BcduOperation = demo_eps::action::BCDUOperation;
  using GoalHandle = rclcpp_action::ServerGoalHandle<BcduOperation>;

  explicit BcduNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  // Action server
  rclcpp_action::Server<BcduOperation>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const BcduOperation::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // Subscriptions and Publications
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bus_voltage_sub_;
  rclcpp::Publisher<demo_eps::msg::BCDUStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Service<demo_eps::srv::SetBCDUState>::SharedPtr mode_srv_;

  // Charge/discharge clients for each battery ORU
  std::unordered_map<int, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> charge_clients_;
  std::unordered_map<int, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> discharge_clients_;

  // Battery state tracking
  struct BatteryStateSnapshot
  {
    double voltage = 0.0;
    bool discharging = false;
    rclcpp::Time last_update;
  };

  std::unordered_map<int, rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr> battery_subs_;
  std::unordered_map<int, BatteryStateSnapshot> battery_state_;

  // Internal state
  std::mutex mtx_;
  double bus_voltage_{0.0};
  double regulation_voltage_{120.0};
  double current_draw_{0.0};
  std::string mode_{"idle"};
  bool fault_{false};
  std::string fault_msg_;

  // Parameters
  double max_discharge_current_A_{127.0};  // Fault Isolator (FI) limit
  double max_charge_current_A_{65.0};
  double regulation_min_V_{130.0};
  double regulation_max_V_{150.0};
  double bus_low_threshold_V_{140.0};
  double bus_high_threshold_V_{165.0};
  int num_channels_{12};
  int orus_per_channel_{2};

  // Callbacks
  void busVoltageCb(const std_msgs::msg::Float64::SharedPtr msg);
  void batteryCb(int id, const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void setModeSrvCb(
    const std::shared_ptr<demo_eps::srv::SetBCDUState::Request> req,
    std::shared_ptr<demo_eps::srv::SetBCDUState::Response> res);

  // Helper methods
  void publishStatus(const std::string &mode, bool fault, const std::string &fault_msg);
  void publishDiag(uint8_t level, const std::string &name, const std::string &msg);
};

} // namespace demo_eps
