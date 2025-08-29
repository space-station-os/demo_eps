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

  action_server_ = rclcpp_action::create_server<BcduOperation>(
    this,
    "bcdu/operate",
    std::bind(&BcduNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&BcduNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&BcduNode::handle_accepted, this, std::placeholders::_1));

  bus_voltage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/bus_voltage", 10,
      std::bind(&BcduNode::busVoltageCb, this, std::placeholders::_1));

  status_pub_ = this->create_publisher<demo_eps::msg::BCDUStatus>("/bcdu/status", 10);
  diag_pub_   = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

  int total_orus = num_channels_ * orus_per_channel_;
  for (int i = 0; i < total_orus; ++i) {
    std::string topic = "/battery/battery_bms_" + std::to_string(i) + "/health";
    battery_subs_[i] = this->create_subscription<sensor_msgs::msg::BatteryState>(
        topic, 10,
        [this, i](sensor_msgs::msg::BatteryState::SharedPtr msg) {
          this->batteryCb(i, msg);
        });
  }

  mode_srv_ = this->create_service<demo_eps::srv::SetBCDUState>(
    "/bcdu/set_mode",
    std::bind(&BcduNode::setModeSrvCb, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "BCDU node ready. Reg window: [%.1f, %.1f] V; FI limit: %.1f A",
              regulation_min_V_, regulation_max_V_, max_discharge_current_A_);
}

rclcpp_action::GoalResponse BcduNode::handle_goal(const rclcpp_action::GoalUUID &,
                      std::shared_ptr<const BcduOperation::Goal> goal)
{
  if (goal->operation_type != "charge" && goal->operation_type != "discharge") return rclcpp_action::GoalResponse::REJECT;
  if (goal->target_voltage < regulation_min_V_ || goal->target_voltage > regulation_max_V_) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BcduNode::handle_cancel(const std::shared_ptr<GoalHandle> /*gh*/)
{
  RCLCPP_INFO(get_logger(), "Canceling BCDU goal on request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BcduNode::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&BcduNode::execute, this, goal_handle)}.detach();
}

void BcduNode::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  {
    std::lock_guard<std::mutex> lk(mtx_);
    regulation_voltage_ = goal->target_voltage;
    mode_ = (goal->operation_type == "charge") ? "charging" : "discharging";
    fault_ = false;
    fault_msg_.clear();
  }
  publishStatus(mode_, false, "");
  publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", "Operation started");

  rclcpp::Rate rate(10.0);
  auto feedback = std::make_shared<BcduOperation::Feedback>();
  auto result   = std::make_shared<BcduOperation::Result>();
  static int dwell = 0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      std::lock_guard<std::mutex> lk(mtx_);
      mode_ = "idle"; current_draw_ = 0.0;
      publishStatus(mode_, false, "");
      result->success = false; result->message = "Operation canceled";
      goal_handle->canceled(result);
      publishDiag(diagnostic_msgs::msg::DiagnosticStatus::WARN, "BCDU", result->message);
      return;
    }

    double busV, regV, drawA; std::string mode_local;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      busV = bus_voltage_; regV = regulation_voltage_; mode_local = mode_;
    }

    fault_ = false; fault_msg_.clear(); drawA = 0.0;

    if (mode_local == "discharging") {
      if (busV >= bus_high_threshold_V_) mode_local = "idle";
      else {
        bool can_discharge = false;
        for (const auto& [id, state] : battery_state_) {
          if (state.voltage > 50.0 && !state.discharging) {
            can_discharge = true; break;
          }
        }
        if (!can_discharge) {
          fault_ = true; fault_msg_ = "No healthy battery available for discharge";
        } else {
          double error = regV - busV;
          drawA = std::clamp(error * 1.0, 0.0, max_discharge_current_A_);
          if (drawA >= max_discharge_current_A_ - 1e-6) {
            fault_ = true; fault_msg_ = "Overcurrent near FI limit";
          }
        }
      }
    } else if (mode_local == "charging") {
      if (busV <= bus_low_threshold_V_) {
        mode_local = "safe"; fault_ = true; fault_msg_ = "Bus low during charge";
      } else {
        double error = busV - regV;
        drawA = -std::clamp(error * 0.8, 0.0, max_charge_current_A_);
      }
    }

    {
      std::lock_guard<std::mutex> lk(mtx_);
      current_draw_ = drawA; mode_ = mode_local;
    }

    if (fault_) {
      publishStatus("fault", true, fault_msg_);
      publishDiag(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "BCDU", fault_msg_);
      result->success = false; result->message = fault_msg_;
      goal_handle->abort(result);
      return;
    }

    feedback->current_voltage = busV;
    feedback->status = mode_local;
    goal_handle->publish_feedback(feedback);
    publishStatus(mode_local, false, "");

    if (std::abs(busV - regV) < 1.0 && (mode_local == "charging" || mode_local == "discharging")) {
      if (++dwell > 30) {
        result->success = true; result->message = "Reached regulation window";
        goal_handle->succeed(result);
        publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", result->message);
        {
          std::lock_guard<std::mutex> lk(mtx_);
          mode_ = "idle"; current_draw_ = 0.0;
        }
        publishStatus("idle", false, "");
        return;
      }
    } else dwell = 0;

    rate.sleep();
  }

  if (rclcpp::ok()) {
    result->success = false; result->message = "Node stopped";
    goal_handle->abort(result);
  }
}

void BcduNode::busVoltageCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  bus_voltage_ = msg->data;
}

void BcduNode::batteryCb(int id, const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  battery_state_[id] = {
    .voltage = msg->voltage,
    .discharging = (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING),
    .last_update = msg->header.stamp
  };
}

void BcduNode::publishStatus(const std::string &mode, bool fault, const std::string &fault_msg)
{
  auto m = demo_eps::msg::BCDUStatus();
  m.header.stamp = now(); m.mode = mode;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    m.bus_voltage = bus_voltage_;
    m.regulation_voltage = regulation_voltage_;
    m.current_draw = current_draw_;
  }
  m.fault = fault;
  m.fault_message = fault_msg;
  status_pub_->publish(m);
}

void BcduNode::publishDiag(uint8_t level, const std::string &name, const std::string &msg)
{
  diagnostic_msgs::msg::DiagnosticStatus d;
  d.level = level; d.name = name; d.message = msg;
  diag_pub_->publish(d);
}

void BcduNode::setModeSrvCb(
  const std::shared_ptr<demo_eps::srv::SetBCDUState::Request> req,
  std::shared_ptr<demo_eps::srv::SetBCDUState::Response> res)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (req->desired_mode == "idle" || req->desired_mode == "charge" ||
      req->desired_mode == "discharge" || req->desired_mode == "safe") {
    mode_ = req->desired_mode;
    res->accepted = true;
    res->message = "Mode set to " + mode_;
    publishStatus(mode_, false, "");
  } else {
    res->accepted = false;
    res->message = "Invalid mode";
  }
}

} // namespace demo_eps

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node= std::make_shared<demo_eps::BcduNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
