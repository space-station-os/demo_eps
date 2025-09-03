#include "demo_eps/bcdu_device.hpp"

#include <chrono>
#include <thread>
#include <utility>

using namespace std::chrono_literals;

namespace demo_eps
{

BcduNode::BcduNode(const rclcpp::NodeOptions &options)
: Node("bcdu_node", options)
{
  // Core limits / regulation
  max_discharge_current_A_ = this->declare_parameter("max_discharge_current_A", 127.0);
  max_charge_current_A_    = this->declare_parameter("max_charge_current_A", 65.0);
  regulation_min_V_        = this->declare_parameter("regulation_min_V", 130.0);
  regulation_max_V_        = this->declare_parameter("regulation_max_V", 180.0);
  regulation_voltage_      = this->declare_parameter("regulation_voltage", 150.0);

  // SSU thresholds & nominal feed
  ssu_charge_enter_v_      = this->declare_parameter("ssu_charge_enter_v", 160.0);
  ssu_discharge_enter_v_   = this->declare_parameter("ssu_discharge_enter_v", 152.0);
  ssu_nominal_v_           = this->declare_parameter("ssu_nominal_v", 160.0);

  // Fleet geometry
  num_channels_            = this->declare_parameter("num_channels", 12);
  orus_per_channel_        = this->declare_parameter("orus_per_channel", 2);

  const int total_orus = num_channels_ * orus_per_channel_;
  for (int i = 0; i < total_orus; ++i) {
    const std::string base = "/battery/battery_bms_" + std::to_string(i);
    const std::string health_topic = base + "/health";
    const std::string charge_srv   = base + "/charge";
    const std::string discharge_srv= base + "/discharge";

    battery_subs_[i] = this->create_subscription<sensor_msgs::msg::BatteryState>(
      health_topic, rclcpp::SensorDataQoS(),
      [this, i](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        this->batteryCallback(i, msg);
      });


    charge_clients_[i]    = this->create_client<std_srvs::srv::Trigger>(charge_srv);
    discharge_clients_[i] = this->create_client<std_srvs::srv::Trigger>(discharge_srv);
  }


  ssu_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/solar_controller/ssu_voltage_v", 10,
    std::bind(&BcduNode::ssuVoltageCb, this, std::placeholders::_1));


  status_pub_      = this->create_publisher<demo_eps::msg::BCDUStatus>("/bcdu/status", 10);
  diag_pub_        = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);
  mbsu_voltage_pub_= this->create_publisher<std_msgs::msg::Float64>("/mbsu/input_voltage", 10);

  control_timer_ = this->create_wall_timer(500ms, std::bind(&BcduNode::controlTick, this));

  RCLCPP_INFO(this->get_logger(),
              "BCDU ready. Hysteresis: charge>=%.1fV, discharge<=%.1fV, regulation=%.1fV, SSU nominal=%.1fV",
              ssu_charge_enter_v_, ssu_discharge_enter_v_, regulation_voltage_, ssu_nominal_v_);
}

void BcduNode::batteryCallback(int id, const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  battery_state_[id] = BatteryInfo{
    msg->voltage,
    (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING),
    this->now()  
  };
}

void BcduNode::ssuVoltageCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ssu_voltage_ = msg->data;
  ssu_last_update_ = this->now();
}

std::vector<int> BcduNode::healthyBatteryIdsLocked() const
{
  std::vector<int> ids;
  const auto now = this->now();

  for (const auto &kv : battery_state_) {
    const int id = kv.first;
    const auto &info = kv.second;
   
    if ((now - info.last_update) < stale_timeout_ && info.voltage > 50.0) {
      ids.push_back(id);
    }
  }
  return ids;
}

void BcduNode::controlTick()
{
 \
  double ssu_v = 0.0;
  rclcpp::Time ssu_ts;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ssu_v = ssu_voltage_;
    ssu_ts = ssu_last_update_;
  }

  
  if (!ssu_ts.nanoseconds() || (this->now() - ssu_ts) > stale_timeout_) {
    return;
  }

  if (transition_in_progress_) {
   
    return;
  }

  
  std::string desired = mode_;
  if (mode_ == "charge" || mode_ == "idle") {
    if (ssu_v <= ssu_discharge_enter_v_) desired = "discharge";
    else if (ssu_v >= ssu_charge_enter_v_) desired = "charge";
  } else if (mode_ == "discharge") {
    if (ssu_v >= ssu_charge_enter_v_) desired = "charge";
    else if (ssu_v <= ssu_discharge_enter_v_) desired = "discharge";
  }

  if (desired == mode_) {
    
    publishToMbsu(mode_ == "charge" ? ssu_nominal_v_ : regulation_voltage_);
    publishStatus(mode_);
    return;
  }

  // Transition
  transition_in_progress_ = true;
  if (desired == "charge") {
    enterCharge();
  } else if (desired == "discharge") {
    enterDischarge();
  } else {
    transition_in_progress_ = false;
  }
}

void BcduNode::enterCharge()
{
  publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", "Entering CHARGE");
  RCLCPP_INFO(this->get_logger(), "[BCDU] Enter CHARGE: commanding all healthy BMS /charge");

  std::vector<int> ids;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ids = healthyBatteryIdsLocked();
  }

  // Parallel Trigger calls
  std::vector<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> futures;
  for (int id : ids) {
    auto client = charge_clients_[id];
    if (!client) continue;
    client->wait_for_service(0s);
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    futures.push_back(client->async_send_request(req));
  }
  for (auto &f : futures) (void)f.wait();

  mode_ = "charge";
  publishToMbsu(ssu_nominal_v_);  // divert SSU 160Vdc to MBSU
  publishStatus(mode_);
  transition_in_progress_ = false;
}

void BcduNode::enterDischarge()
{
  publishDiag(diagnostic_msgs::msg::DiagnosticStatus::OK, "BCDU", "Entering DISCHARGE");
  RCLCPP_INFO(this->get_logger(), "[BCDU] Enter DISCHARGE: commanding all healthy BMS /discharge");

  std::vector<int> ids;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ids = healthyBatteryIdsLocked();
  }

  std::vector<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> futures;
  for (int id : ids) {
    auto client = discharge_clients_[id];
    if (!client) continue;
    client->wait_for_service(0s);
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    futures.push_back(client->async_send_request(req));
  }
  for (auto &f : futures) (void)f.wait();

  mode_ = "discharge";
  publishToMbsu(regulation_voltage_);  
  publishStatus(mode_);
  transition_in_progress_ = false;
}

void BcduNode::publishToMbsu(double volts)
{
  std_msgs::msg::Float64 v;
  v.data = volts;
  mbsu_voltage_pub_->publish(v);
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
  demo_eps::msg::BCDUStatus st;
  st.header.stamp = this->now();
  st.mode = mode;
  st.fault = fault;
  st.fault_message = fault_msg;
  st.bus_voltage = (mode == "charge") ? ssu_nominal_v_ : regulation_voltage_;
  st.regulation_voltage = regulation_voltage_;
  st.current_draw = (mode == "discharge") ? max_discharge_current_A_
                                          : (mode == "charge" ? -max_charge_current_A_ : 0.0);
  status_pub_->publish(st);
}

}  // namespace demo_eps

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<demo_eps::BcduNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
