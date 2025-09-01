#include "demo_eps/eps_power_controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace demo_eps;
using namespace std::chrono_literals;

EpsPowerController::EpsPowerController(const rclcpp::NodeOptions &options)
: Node("eps_power_controller", options)
{
  this->declare_parameter("charge_threshold", 2.0);  // in kW
  this->declare_parameter("discharge_threshold", 0.5);  // in kW
  this->get_parameter("charge_threshold", charge_threshold_);
  this->get_parameter("discharge_threshold", discharge_threshold_);

  bcdu_client_ = rclcpp_action::create_client<BCDUOperation>(this, "bcdu/operate");

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

  solar_voltage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/solar_controller/ssu_voltage_v", 10,
    std::bind(&EpsPowerController::solarVoltageCb, this, std::placeholders::_1));

  solar_power_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/solar_controller/ssu_power_w", 10,
    std::bind(&EpsPowerController::solarPowerCb, this, std::placeholders::_1));

  control_timer_ = this->create_wall_timer(2s, std::bind(&EpsPowerController::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "EPS Power Controller initialized");
}

void EpsPowerController::solarPowerCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  solar_power_kw_ = msg->data / 1000.0;  // W to kW
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Updated solar power: %.2f kW", solar_power_kw_);
}

void EpsPowerController::solarVoltageCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  latest_solar_voltage_ = msg->data;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Updated solar voltage: %.2f V", latest_solar_voltage_);
}

void EpsPowerController::controlLoop()
{
  if (fault_) return;

  // Retrigger logic if current mode doesn't match requirement
  if (!goal_active_ || current_mode_ != desired_mode_) {
    if (solar_power_kw_ > charge_threshold_ && current_mode_ != "charging") {
      RCLCPP_INFO(this->get_logger(), "[EPS]CHARGING...");
      maybeSendGoal("charge");
    } else if (solar_power_kw_ < discharge_threshold_ && current_mode_ != "discharging") {
      RCLCPP_INFO(this->get_logger(), "[EPS]DISCHARGING...");
      maybeSendGoal("discharge");
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Solar power %.2f kW in idle range", solar_power_kw_);
    }
  }
}

void EpsPowerController::maybeSendGoal(const std::string &mode)
{
  if (!bcdu_client_->wait_for_action_server(1s)) {
    RCLCPP_WARN(this->get_logger(), "BCDU action server not available.");
    goal_active_ = false;
    return;
  }

  auto goal_msg = BCDUOperation::Goal();
  goal_msg.operation_type = mode;
  // goal_msg.current_voltage = std::clamp(latest_solar_voltage_, 130.0, 180.0);

  desired_mode_ = mode;
  goal_active_ = true;

  RCLCPP_INFO(this->get_logger(), "Sending %s goal to BCDU", mode.c_str());

  auto send_goal_options = rclcpp_action::Client<BCDUOperation>::SendGoalOptions();
  send_goal_options.feedback_callback =
    std::bind(&EpsPowerController::bcduFeedbackCb, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&EpsPowerController::bcduResultCb, this, std::placeholders::_1);

  bcdu_client_->async_send_goal(goal_msg, send_goal_options);
}

void EpsPowerController::bcduFeedbackCb(
  GoalHandleBcdu::SharedPtr,
  const std::shared_ptr<const BCDUOperation::Feedback> feedback)
{
  current_mode_ = feedback->status;
  RCLCPP_INFO(this->get_logger(), "[Feedback] Mode: %s, Bus V: %.2f",
              feedback->status.c_str(), feedback->current_voltage);
}

void EpsPowerController::bcduResultCb(const GoalHandleBcdu::WrappedResult &result)
{
  goal_active_ = false;

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
    RCLCPP_INFO(this->get_logger(), "BCDU goal succeeded: %s", result.result->message.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "BCDU goal failed: %s", result.result->message.c_str());
    enterSafeMode(result.result->message);
  }
}

void EpsPowerController::enterSafeMode(const std::string &reason)
{
  RCLCPP_FATAL(this->get_logger(), "Entering SAFE MODE: %s", reason.c_str());
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.level = diag.ERROR;
  diag.name = "EPS Power Controller";
  diag.message = "Entered Safe Mode: " + reason;
  diag_pub_->publish(diag);
  fault_ = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EpsPowerController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
