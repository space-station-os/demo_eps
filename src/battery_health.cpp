#include "demo_eps/battery_health.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

BatteryManager::BatteryManager()
: Node("battery_manager_node")
{
  this->declare_parameter<int>("num_channels", 12);
  this->declare_parameter<std::string>("battery_config", "config/battery_config.yaml");

  this->get_parameter("num_channels", num_channels_);
  std::string config_file;
  this->get_parameter("battery_config", config_file);

  std::string config_path = ament_index_cpp::get_package_share_directory("demo_eps") + "/" + config_file;
  YAML::Node config = YAML::LoadFile(config_path);

  int total_orus = num_channels_ * 2;
  RCLCPP_INFO(this->get_logger(), "Initializing BatteryManager with %d ORUs", total_orus);

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

  for (int i = 0; i < total_orus; ++i)
  {
    auto unit = std::make_shared<BatteryUnit>();
    unit->id = "battery_bms_" + std::to_string(i);
    unit->voltage = BATTERY_MAX_VOLTAGE;
    unit->discharging = false;

    
    if (config[unit->id]) {
      unit->location = config[unit->id].as<std::string>();
    } else {
      unit->location = "unknown";
    }

    unit->publisher = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery/" + unit->id + "/health", 10);

    unit->service = this->create_service<std_srvs::srv::Trigger>(
      "/battery/" + unit->id + "/discharge",
      [this, unit](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "Discharge request received for %s", unit->id.c_str());
        unit->discharging = true;
        response->success = true;
        response->message = "Battery " + unit->id + " is discharging.";
      });

    batteries_.push_back(*unit);
  }

  publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      for (auto& unit : batteries_)
        update_and_publish(std::make_shared<BatteryUnit>(unit));
    });
}

void BatteryManager::update_and_publish(const std::shared_ptr<BatteryUnit>& unit)
{
  float max_voltage = BATTERY_MAX_VOLTAGE;
  float min_voltage = BATTERY_MIN_VOLTAGE;
  float warning_voltage = 90.0f;
  float critical_voltage = 85.0f;

  if (unit->discharging)
  {
    unit->voltage -= 0.5;
    if (unit->voltage <= min_voltage)
    {
      unit->voltage = min_voltage;
      unit->discharging = false;
      RCLCPP_WARN(this->get_logger(), "%s has reached minimum voltage.", unit->id.c_str());
    }
  }
  else
  {
    unit->voltage += 0.3;
    if (unit->voltage > max_voltage)
      unit->voltage = max_voltage;
  }

  sensor_msgs::msg::BatteryState b;
  b.header.stamp = this->now();
  b.header.frame_id = unit->id;
  b.voltage = unit->voltage;
  b.current = unit->discharging ? -5.0f : 4.0f;
  b.charge = unit->voltage / max_voltage * 100.0f;
  b.capacity = 100.0f;
  b.percentage = unit->voltage / max_voltage;
  b.power_supply_status = unit->discharging ? b.POWER_SUPPLY_STATUS_DISCHARGING : b.POWER_SUPPLY_STATUS_CHARGING;
  b.power_supply_health = (unit->voltage < warning_voltage) ?
    ((unit->voltage < critical_voltage) ? b.POWER_SUPPLY_HEALTH_DEAD : b.POWER_SUPPLY_HEALTH_OVERHEAT) : b.POWER_SUPPLY_HEALTH_GOOD;
  b.power_supply_technology = b.POWER_SUPPLY_TECHNOLOGY_LION;
  b.present = true;
  b.location = unit->location;
  b.serial_number = unit->id;

  b.cell_voltage.resize(38);
  b.cell_temperature.resize(38);

  float avg_cell_voltage = unit->voltage / 38.0f;
  for (int i = 0; i < 38; ++i)
  {
    b.cell_voltage[i] = avg_cell_voltage + static_cast<float>(rand() % 100 - 50) / 1000.0f;
    b.cell_temperature[i] = 25.0f + static_cast<float>(rand() % 100 - 50) / 10.0f;
  }

  unit->publisher->publish(b);

  if (unit->discharging && unit->voltage < warning_voltage)
  {
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = unit->id;
    diag.level = (unit->voltage < critical_voltage) ? diag.ERROR : diag.WARN;
    diag.message = (unit->voltage < critical_voltage) ? "Critical voltage level" : "Low voltage during eclipse";
    diag.hardware_id = unit->location;
    diag_pub_->publish(diag);
  }
}

int main (int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BatteryManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
