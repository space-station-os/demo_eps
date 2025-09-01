#ifndef DEMO_EPS__EPS_POWER_CONTROLLER_HPP_
#define DEMO_EPS__EPS_POWER_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <demo_eps/msg/bcdu_status.hpp>
#include <demo_eps/action/bcdu_operation.hpp>

namespace demo_eps
{

class EpsPowerController : public rclcpp::Node
{
public:
  using BCDUOperation = demo_eps::action::BCDUOperation;
  using GoalHandleBcdu = rclcpp_action::ClientGoalHandle<BCDUOperation>;

  explicit EpsPowerController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void controlLoop();
  void enterSafeMode(const std::string &reason);
  void maybeSendGoal(const std::string &mode);  // new unified goal sender

private:
  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr solar_voltage_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr solar_power_sub_;
  rclcpp::Subscription<demo_eps::msg::BCDUStatus>::SharedPtr bcdu_status_sub_;
  
  // Publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;

  // Action client
  rclcpp_action::Client<BCDUOperation>::SharedPtr bcdu_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parameters
  double charge_threshold_;
  double discharge_threshold_;
  double latest_solar_voltage_;

  // Internal state
  double solar_power_kw_;
  std::string current_mode_ = "idle";
  std::string desired_mode_ = "idle";
  bool fault_ = false;
  bool goal_active_ = false;

  // Callbacks
  void solarPowerCb(const std_msgs::msg::Float64::SharedPtr msg);
  void solarVoltageCb(const std_msgs::msg::Float64::SharedPtr msg);
  void bcduStatusCb(const demo_eps::msg::BCDUStatus::SharedPtr msg);
  void bcduFeedbackCb(GoalHandleBcdu::SharedPtr, const std::shared_ptr<const BCDUOperation::Feedback> feedback);
  void bcduResultCb(const GoalHandleBcdu::WrappedResult &result);
};

}  // namespace demo_eps

#endif  // DEMO_EPS__EPS_POWER_CONTROLLER_HPP_
