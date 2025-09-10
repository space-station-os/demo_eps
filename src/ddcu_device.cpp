#include "demo_eps/ddcu_device.hpp"

using namespace demo_eps;

DdcuNode::DdcuNode(const rclcpp::NodeOptions &options)
: Node("ddcu_node", options)
{
  this->declare_parameter<std::string>("ddcu_type", "DDCU-I");
  this->declare_parameter<double>("regulation_nominal", 124.5);
  this->declare_parameter<double>("regulation_tolerance", 1.5);

  this->get_parameter("ddcu_type", ddcu_type_);
  this->get_parameter("regulation_nominal", nominal_voltage_);
  this->get_parameter("regulation_tolerance", regulation_tolerance_);

  primary_voltage_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/ddcu/input_voltage", 10,
    std::bind(&DdcuNode::primaryVoltageCallback, this, std::placeholders::_1)
  );

  output_voltage_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ddcu/output_voltage", 10);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/eps/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(), "DDCU (%s) Node initialized", ddcu_type_.c_str());
}

void DdcuNode::primaryVoltageCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  input_voltage_ = msg->data;
  double output_voltage = nominal_voltage_;

  // Fault conditions for input voltage
  if (input_voltage_ < 115.0 || input_voltage_ > 173.0)
  {
    output_voltage = 0.0;
    publishOutputVoltage(output_voltage);
    publishDiagnostics(input_voltage_, output_voltage);
    return;
  }

  // Add small fluctuations to simulate regulation behavior
  output_voltage += ((rand() % 300) - 150) / 100.0; // ±1.5V

  publishOutputVoltage(output_voltage);
  publishDiagnostics(input_voltage_, output_voltage);
}

void DdcuNode::publishOutputVoltage(double voltage)
{
  std_msgs::msg::Float64 msg;
  msg.data = voltage;
  output_voltage_pub_->publish(msg);
}

void DdcuNode::publishDiagnostics(double input_voltage, double output_voltage)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "DDCU_" + ddcu_type_;
  status.hardware_id = "DDCU_Generic";

  if (input_voltage < 115.0 || input_voltage > 173.0)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Primary input voltage out of range!";
  }
  else if (output_voltage < (nominal_voltage_ - regulation_tolerance_) ||
           output_voltage > (nominal_voltage_ + regulation_tolerance_))
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Secondary voltage outside regulation range!";
  }
  else
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "DDCU operating within limits.";
  }

  diag_pub_->publish(status);
}



int main (int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DdcuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}