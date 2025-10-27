#include "rclcpp/rclcpp.hpp"
#include "ethercat_msgs/msg/error_code.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

class EthercatErrorDiagnosticsNode : public rclcpp::Node
{
public:
  EthercatErrorDiagnosticsNode()
  : Node("ethercat_error_diagnostics")
  {
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10));   // 默认 volatile, reliable
    sub_qos.best_effort();   

    sub_ = this->create_subscription<ethercat_msgs::msg::ErrorCode>(
      "/ethercat/drive_error", sub_qos,
      std::bind(&EthercatErrorDiagnosticsNode::onError, this, std::placeholders::_1));

    // /diagnostics 通常用默认 QoS
    pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Started: EtherCAT -> /diagnostics bridge");
  }

private:
  static std::string hex4(uint16_t v)
  {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%04X", v);
    return std::string(buf);
  }

  void onError(const ethercat_msgs::msg::ErrorCode::SharedPtr msg)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;

    // 每个从站一条 status
    status.name = "EtherCAT Drive m" + std::to_string(msg->master_id) +
                  "_a" + std::to_string(msg->alias) +
                  "_p" + std::to_string(msg->position);
    status.hardware_id = std::to_string(msg->alias);

    // level：0=OK, 1=WARN, 2=ERROR, 3=STALE
    if (msg->error_code == 0x0000) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "No error";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = std::string("Drive error: ") + hex4(msg->error_code);
    }

    // 附加字段
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "error_code";
      kv.value = hex4(msg->error_code);
      status.values.push_back(kv);
    }
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "master_id";
      kv.value = std::to_string(msg->master_id);
      status.values.push_back(kv);
    }
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "alias";
      kv.value = std::to_string(msg->alias);
      status.values.push_back(kv);
    }
    {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "position";
      kv.value = std::to_string(msg->position);
      status.values.push_back(kv);
    }

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->get_clock()->now();
    arr.status.push_back(status);

    pub_->publish(arr);
  }

  rclcpp::Subscription<ethercat_msgs::msg::ErrorCode>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EthercatErrorDiagnosticsNode>());
  rclcpp::shutdown();
  return 0;
}
