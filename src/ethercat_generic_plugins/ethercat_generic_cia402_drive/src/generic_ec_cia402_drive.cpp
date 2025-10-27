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

#include <numeric>
#include <exception>
#include <unordered_map>
#include <string>
#include <rcutils/logging_macros.h>

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins
{

EcCiA402Drive::EcCiA402Drive()
: GenericEcSlave() {}
EcCiA402Drive::~EcCiA402Drive() {}

bool EcCiA402Drive::initialized() const {return initialized_;}

void EcCiA402Drive::processData(size_t index, uint8_t * domain_address)
{
  // Special case: ControlWord
  if (pdo_channels_info_[index].index == CiA402D_RPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
          !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      if (auto_state_transitions_) {
        pdo_channels_info_[index].default_value = transition(
          state_,
          pdo_channels_info_[index].ec_read(domain_address));
      }
    }
  }

  // setup current position as default position
  if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION) {
    if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE) {
      pdo_channels_info_[index].default_value =
        pdo_channels_info_[index].factor * last_position_ +
        pdo_channels_info_[index].offset;
    }
    pdo_channels_info_[index].override_command =
      (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;
  }

  // setup mode of operation
  if (pdo_channels_info_[index].index == CiA402D_RPDO_MODE_OF_OPERATION) {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
      pdo_channels_info_[index].default_value = mode_of_operation_;
    }
  }

  pdo_channels_info_[index].ec_update(domain_address);

  // get mode_of_operation_display_
  if (pdo_channels_info_[index].index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = pdo_channels_info_[index].last_value;
  }

  if (pdo_channels_info_[index].index == CiA402D_TPDO_POSITION) {
    last_position_ = pdo_channels_info_[index].last_value;
    
  }

  // Special case: StatusWord
  if (pdo_channels_info_[index].index == CiA402D_TPDO_STATUSWORD) {
    status_word_ = pdo_channels_info_[index].last_value;

  }

  if (pdo_channels_info_[index].index == CiA402D_ERROR_CODE) {
    const uint16_t code = static_cast<uint16_t>(pdo_channels_info_[index].last_value);
    publish_error_code_(code);
  }


  // CHECK FOR STATE CHANGE
  if (index == all_channels_.size() - 1) {  // if last entry  in domain
    if (status_word_ != last_status_word_) {
      state_ = deviceState(status_word_);
      if (state_ != last_state_) {
        std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                  << " with status word :" << status_word_ << std::endl;
      }
    }
    initialized_ = ((state_ == STATE_OPERATION_ENABLED) &&
      (last_state_ == STATE_OPERATION_ENABLED)) ? true : false;

    last_status_word_ = status_word_;
    last_state_ = state_;
    counter_++;
    maybe_publish_status_snapshot_();
  }
}

bool EcCiA402Drive::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  // 记录基础标识信息，方便日志区分电机
  if (paramters_.find("position") != paramters_.end()) {
    try {
      position_ = std::stoi(paramters_.at("position"));
    } catch (const std::exception &) {
      position_ = -1;
    }
  }
  if (paramters_.find("alias") != paramters_.end()) {
    try {
      alias_ = std::stoi(paramters_.at("alias"));
    } catch (const std::exception &) {
      alias_ = 0;
    }
  }
  if (paramters_.find("master_id") != paramters_.end()) {
    try {
      master_id_ = std::stoi(paramters_.at("master_id"));
    } catch (const std::exception &) {
      master_id_ = -1;
    }
  }
  if (paramters_.find("name") != paramters_.end()) {
    slave_name_ = paramters_.at("name");
  }

  if (paramters_.find("slave_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "EcCiA402Drive: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  setup_interface_mapping();
  setup_syncs();

  if (paramters_.find("mode_of_operation") != paramters_.end()) {
    mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
  }

  init_error_publisher_();

  return true;
}

// bool EcCiA402Drive::setup_from_config(YAML::Node drive_config)
// {
//   if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
//   // additional configuration parameters for CiA402 Drives
//   if (drive_config["auto_fault_reset"]) {
//     auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
//   }
//   if (drive_config["auto_state_transitions"]) {
//     auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
//   }
//   return true;
// }
bool EcCiA402Drive::setup_from_config(YAML::Node drive_config)
{
  if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
  if (drive_config["auto_fault_reset"]) {
    auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }

  // ---- 新增：错误日志 JSONL 开关 & 路径 & 周期 ----
  if (drive_config["error_log"]) {
    auto node = drive_config["error_log"];
    if (node["enabled"]) {
      json_log_enabled_ = node["enabled"].as<bool>();
    }
    if (node["path"]) {
      json_log_path_ = node["path"].as<std::string>();
    }
    if (node["every_n_cycles"]) { // 默认 500 => 0.5s
      json_log_every_n_ = node["every_n_cycles"].as<size_t>();
      if (json_log_every_n_ == 0) json_log_every_n_ = 1;
    }
  }

  if (json_log_enabled_) {
    json_log_.open(json_log_path_, std::ios::out | std::ios::app);
    if (!json_log_.is_open()) {
      RCUTILS_LOG_WARN("EcCiA402Drive: cannot open error log file: %s", json_log_path_.c_str());
      json_log_enabled_ = false;
    }
  }
  // -----------------------------------------------

  if (drive_config["periodic_status_enabled"]) {
    periodic_status_enabled_ = drive_config["periodic_status_enabled"].as<bool>();
  }
  if (drive_config["status_publish_period_sec"]) {
    try {
      status_publish_period_sec_ = drive_config["status_publish_period_sec"].as<double>();
    } catch (const std::exception &) {
      status_publish_period_sec_ = 5.0;
    }
    if (status_publish_period_sec_ <= 0.0) {
      status_publish_period_sec_ = 5.0;
    }
  }

  return true;
}

void EcCiA402Drive::init_error_publisher_()
{
  if (error_node_) {
    if (!error_pub_) {
      error_pub_ = error_node_->create_publisher<ethercat_msgs::msg::ErrorCode>(
        error_topic_, rclcpp::QoS(10).best_effort());
    }
    return;
  }
  std::string node_name = "ec_drive_error_pub_m" + std::to_string(master_id_) +
    "_a" + std::to_string(alias_) +
    "_p" + std::to_string(position_);
  error_node_ = rclcpp::Node::make_shared(node_name);
  error_pub_ = error_node_->create_publisher<ethercat_msgs::msg::ErrorCode>(
    error_topic_, rclcpp::QoS(10).best_effort());
  last_status_pub_ = error_node_->now();
}

void EcCiA402Drive::publish_error_code_(uint16_t code, bool force)
{
  if (!force && code == last_error_code_) {
    return;
  }
  last_error_code_ = code;

  if (!error_pub_) {
    init_error_publisher_();
  }
  if (!error_pub_ || !error_node_) {
    return;
  }

  ethercat_msgs::msg::ErrorCode msg;
  const auto now = error_node_->now();
  msg.stamp = now;
  last_status_pub_ = now;
  msg.master_id = static_cast<int32_t>(master_id_);
  msg.alias = static_cast<uint16_t>(alias_ < 0 ? 0 : alias_);
  msg.position = position_;
  msg.joint_name = slave_name_;
  msg.status_word = status_word_;
  msg.error_code = code;

  const auto & map = error_info_map();
  auto it = map.find(code);
  msg.error_name = (it != map.end()) ? it->second.name : kUnknownError.name;

  error_pub_->publish(msg);
}

void EcCiA402Drive::maybe_publish_status_snapshot_()
{
  if (!periodic_status_enabled_) {
    return;
  }
  if (!error_node_) {
    init_error_publisher_();
  }
  if (!error_node_) {
    return;
  }
  const auto now = error_node_->now();
  if (last_status_pub_.nanoseconds() == 0) {
    last_status_pub_ = now;
    return;
  }
  const auto elapsed = (now - last_status_pub_).seconds();
  if (elapsed < status_publish_period_sec_) {
    return;
  }
  last_status_pub_ = now;

  if (last_error_code_ == 0u) {
    publish_error_code_(0u, true);
  }
}

void EcCiA402Drive::write_error_json_snapshot_()
{
  if (!json_log_enabled_ || !json_log_.is_open()) return;

  // 时间戳（ms since epoch）
  const auto now = std::chrono::system_clock::now();
  const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

  // 预先抓取本次 snapshot 的数据，避免持锁时间过长
  struct Item { size_t idx; int32_t val; };
  std::vector<Item> items;
  items.reserve(8);
  for (size_t i = 0; i < pdo_channels_info_.size(); ++i) {
    if (pdo_channels_info_[i].index == CiA402D_ERROR_CODE) {
      items.push_back({i, static_cast<int32_t>(pdo_channels_info_[i].last_value)});
    }
  }
  const uint16_t sw = status_word_;
  const DeviceState st = state_;
  const char* st_str = DEVICE_STATE_STR.at(st).c_str();

  // 组 JSON 文本（JSON Lines）：一行一个对象
  // 形如：
  // {"ts_ms": 175...,"status_word": 12345,"state":"OPERATION_ENABLED","errors":[{"index":3,"code":4660}, ...]}
  std::string line;
  line.reserve(384 + items.size() * 48);  // 适当增大
  line += "{\"ts_ms\":";
  line += std::to_string(ms);

  // 新增：标识字段
  line += ",\"master_id\":";
  line += std::to_string(master_id_);
  line += ",\"alias\":";
  line += std::to_string(alias_);
  line += ",\"position\":";
  line += std::to_string(position_);


  // 原有字段
  line += ",\"status_word\":";
  line += std::to_string(static_cast<uint32_t>(sw));
  line += ",\"state\":\"";
  line += st_str;
  line += "\",\"errors\":[";
  // ...

  const auto &error_map = error_info_map();
  for (size_t k = 0; k < items.size(); ++k) {
    const auto &it = items[k];
    const uint16_t error_code = static_cast<uint16_t>(it.val);
    const auto info_it = error_map.find(error_code);
    const auto &info = (info_it != error_map.end()) ? info_it->second : kUnknownError;

    line += "{\"index\":";
    line += std::to_string(it.idx);
    line += ",\"code\":";
    line += std::to_string(static_cast<uint32_t>(error_code));
    line += ",\"name\":\"";
    line += info.name;
    line += "\"}";
    if (k + 1 < items.size()) line += ",";
  }
  line += "]}";
  line += "\n";

  // 轻量写入（短锁）
  {
    std::lock_guard<std::mutex> lk(json_log_mtx_);
    json_log_ << line;       // 不强制 flush，减少 IO 开销
    // 如需更稳妥，可每 N 次 flush 一次：由你决定是否加入计数器
  }
}

bool EcCiA402Drive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

/** returns device state based upon the status_word */
DeviceState EcCiA402Drive::deviceState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return STATE_NOT_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return STATE_SWITCH_ON_DISABLED;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return STATE_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return STATE_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return STATE_OPERATION_ENABLED;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return STATE_QUICK_STOP_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return STATE_FAULT_REACTION_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  }
  return STATE_UNDEFINED;
}

/** returns the control word that will take device from state to next desired state */
uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word)
{
  switch (state) {
    case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
      return (control_word & 0b01111110) | 0b00000110;
    case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
      return (control_word & 0b01110111) | 0b00000111;
    case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_OPERATION_ENABLED:         // -> GOOD
      return control_word;
    case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
