#include "demo_eps/bcdu_device.hpp"
#include <chrono>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

namespace demo_eps
{

BcduNode::BcduNode(const rclcpp::NodeOptions &options)
: Node("bcdu_node", options)
{
  max_discharge_current_A_ = this->declare_parameter("max_discharge_current_A", 127.0);
  max_charge_current_A_    = this->declare_parameter("max_charge_current_A", 65.0);
  regulation_min_V_        = this->declare_parameter("regulation_min_V", 130.0);
  regulation_max_V_        = this->declare_parameter("regulation_max_V", 180.0);
  regulation_voltage_      = this->declare_parameter("regulation_voltage", 150.0);
  bus_low_threshold_V_     = this->declare_parameter("bus_low_threshold_V", 140.0);
  bus_high_threshold_V_    = this->declare_parameter("bus_high_threshold_V", 165.0);
  num_channels_            = this->declare_parameter("num_channels", 12);
  orus_per_channel_        = this->declare_parameter("orus_per_channel", 2);

  int total_orus = num_channels_ * orus_per_channel_;
  for (int i = 0; i < total_orus; ++i) {
    std::string health_topic = "/battery/battery_bms_" + std::to_string(i) + "/health";
    std::string charge_srv = "/battery/battery_bms_" + std::to_string(i) + "/charge";
    std::string discharge_srv = "/battery/battery_bms_" + std::to_string(i) + "/discharge";

    battery_subs_[i] = this->create_subscription<sensor_msgs::msg::BatteryState>(
      health_topic, 10, [this, i](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        battery_state_[i] = {
          .voltage = msg->voltage,
          .discharging = (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING),
          .last_update = msg->header.stamp
        };
      });

    charge_clients_[i] = this->create_client<std_srvs::srv::Trigger>(charge_srv);
    discharge_clients_[i] = this->create_client<std_srvs::srv::Trigger>(discharge_srv);
  }

  action_server_ = rclcpp_action::create_server<BcduOperation>(
    this, "bcdu/operate",
    std::bind(&BcduNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&BcduNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&BcduNode::handle_accepted, this, std::placeholders::_1));

  bus_voltage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/bus_voltage", 10,
    std::bind(&BcduNode::busVoltageCb, this, std::placeholders::_1));

  status_pub_ = this->create_publisher<demo_eps::msg::BCDUStatus>("/bcdu/status", 10);
  diag_pub_   = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(), "BCDU node ready with parallel battery ops");
}

rclcpp_action::GoalResponse BcduNode::handle_goal(const rclcpp_action::GoalUUID &,
                                                  std::shared_ptr<const BcduOperation::Goal> goal)
{
  if (goal->operation_type != "charge" && goal->operation_type != "discharge"){
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BcduNode::handle_cancel(const std::shared_ptr<GoalHandle> /*gh*/)
{
  RCLCPP_INFO(this->get_logger(), "Canceling BCDU goal on request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BcduNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&BcduNode::execute, this, goal_handle)}.detach();
}

void BcduNode::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  std::string op_type = goal->operation_type;
  publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", "Executing " + op_type);
  RCLCPP_INFO(this->get_logger(), "Executing %s", op_type.c_str());
  auto feedback = std::make_shared<BcduOperation::Feedback>();
  auto result   = std::make_shared<BcduOperation::Result>();

  std::vector<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> futures;
  for (const auto& [id, state] : battery_state_) {
    if (state.voltage <= 50.0) continue;

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> client = nullptr;

    if (op_type == "charge") {
      client = charge_clients_[id];
    } else if (op_type == "discharge") {
      client = discharge_clients_[id];
    }

    if (client && client->service_is_ready()) {
      futures.push_back(client->async_send_request(req));
    }
  }

  for (auto &fut : futures) fut.wait();

  result->success = true;
  result->message = op_type + " completed on healthy batteries";
  goal_handle->succeed(result);

  feedback->status = op_type;
  feedback->current_voltage = bus_voltage_;
  goal_handle->publish_feedback(feedback);

  publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", result->message);

  // Publish extended BCDUStatus
  demo_eps::msg::BCDUStatus status;
  status.header.stamp = this->now();
  status.mode = op_type;
  status.fault = false;
  status.fault_message = "";
  status.bus_voltage = bus_voltage_;
  status.regulation_voltage = regulation_voltage_;
  status.current_draw = (op_type == "discharge") ? max_discharge_current_A_ : -max_charge_current_A_;
  status_pub_->publish(status);
}

void BcduNode::busVoltageCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  bus_voltage_ = msg->data;
}

void BcduNode::publishDiag(uint8_t level, const std::string &name, const std::string &msg)
{
  diagnostic_msgs::msg::DiagnosticStatus d;
  d.level = level;
  d.name = name;
  d.message = msg;
  diag_pub_->publish(d);
}

void BcduNode::publishStatus(const std::string &mode, bool fault, const std::string &fault_msg)
{
  demo_eps::msg::BCDUStatus status;
  status.header.stamp = this->now();
  status.mode = mode;
  status.fault = fault;
  status.fault_message = fault_msg;
  status.bus_voltage = bus_voltage_;
  status.regulation_voltage = regulation_voltage_;
  status.current_draw = (mode == "discharge") ? max_discharge_current_A_ : (mode == "charge" ? -max_charge_current_A_ : 0.0);
  status_pub_->publish(status);
}

} // namespace demo_eps

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<demo_eps::BcduNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}