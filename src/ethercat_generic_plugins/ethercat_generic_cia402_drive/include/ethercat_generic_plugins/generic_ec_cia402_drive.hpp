// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>

#include <fstream>
#include <chrono>
#include <mutex>
#include <cstdint>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "ethercat_generic_plugins/cia402_common_defs.hpp"
#include "ethercat_msgs/msg/error_code.hpp"

namespace ethercat_generic_plugins
{

struct ErrorInfo
{
  const char * name;
};

inline constexpr ErrorInfo kUnknownError{"其他错误"};

inline const std::unordered_map<uint16_t, ErrorInfo> & error_info_map()
{
  static const std::unordered_map<uint16_t, ErrorInfo> map = {
    {0x0000, {"无错误"}},
    {0x3210, {"过压保护"}},
    {0x3220, {"欠压保护"}},
    {0x3230, {"过载保护"}},
    {0x4210, {"温度过高"}},
    {0x7121, {"电机堵转"}},
    {0x7310, {"电机超速"}},
    {0x8130, {"心跳掉线"}},
    {0x8500, {"速度误差过大"}},
    {0x8611, {"位置误差过大"}}
  };
  return map;
}

class EcCiA402Drive : public GenericEcSlave
{
public:
  EcCiA402Drive();
  virtual ~EcCiA402Drive();
  /** Returns true if drive has reached "operation enabled" state.
   *  The transition through the state machine is handled automatically. */
  bool initialized() const;

  virtual void processData(size_t index, uint8_t * domain_address);

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);

  int8_t mode_of_operation_display_ = 0;
  int8_t mode_of_operation_ = -1;

protected:
  uint32_t counter_ = 0;
  uint16_t last_status_word_ = -1;
  uint16_t status_word_ = 0;
  uint16_t control_word_ = 0;
  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;
  bool initialized_ = false;
  bool auto_fault_reset_ = false;
  bool auto_state_transitions_ = true;
  bool fault_reset_ = false;
  int fault_reset_command_interface_index_ = -1;
  bool last_fault_reset_command_ = false;
  double last_position_ = std::numeric_limits<double>::quiet_NaN();

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word);
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word);
  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node drive_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);

  private:
  std::ofstream json_log_;
  std::mutex json_log_mtx_;
  bool json_log_enabled_{false};
  std::string json_log_path_{"./ecat_error_snapshot.jsonl"};
  size_t json_log_every_n_{1};  // 0.5 秒（1 kHz 假设）
  // 辅助：把当前所有 ERROR_CODE 收集写成一行 JSON
  inline void write_error_json_snapshot_();

  // 标识当前从站/电机的基础信息，便于错误日志区分来源
  std::string slave_name_{"unknown"};
  int master_id_{-1};
  int alias_{0};
  int position_{-1};
  rclcpp::Node::SharedPtr error_node_;
  rclcpp::Publisher<ethercat_msgs::msg::ErrorCode>::SharedPtr error_pub_;
  std::string error_topic_{"/ethercat/drive_error"};
  uint16_t last_error_code_{0};
  void init_error_publisher_();
  void publish_error_code_(uint16_t code, bool force = false);
  void maybe_publish_status_snapshot_();
  bool periodic_status_enabled_{true};
  double status_publish_period_sec_{5.0};
  rclcpp::Time last_status_pub_;
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
