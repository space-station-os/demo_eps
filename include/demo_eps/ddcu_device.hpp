#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "space_station_thermal_control/srv/node_heat_flow.hpp"

namespace demo_eps
{

class DdcuNode : public rclcpp::Node
{
public:
  explicit DdcuNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void primaryVoltageCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void publishOutputVoltage(double voltage);
  void publishDiagnostics(double input_voltage, double output_voltage);
  void callInternalCooling(double heat_j);

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr primary_voltage_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_voltage_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::Client<space_station_thermal_control::srv::NodeHeatFlow>::SharedPtr coolant_client_;

  std::string ddcu_type_;
  double nominal_voltage_;
  double regulation_tolerance_;
  double input_voltage_;
};

} // namespace demo_eps
