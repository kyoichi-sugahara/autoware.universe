// Copyright 2025 Tier IV, Inc.
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

#include "autoware_node_death_monitor/autoware_node_death_monitor.hpp"

#include <regex>
#include <string>
#include <vector>

namespace autoware::node_death_monitor
{

NodeDeathMonitor::NodeDeathMonitor(const rclcpp::NodeOptions & options)
: Node("autoware_node_death_monitor", options)
{
  ignore_node_names_ =
    declare_parameter<std::vector<std::string>>("ignore_node_names", std::vector<std::string>{});
  ignore_exit_codes_ = declare_parameter<std::vector<int64_t>>("ignore_exit_codes");
  check_interval_ = declare_parameter<double>("check_interval");
  enable_debug_ = declare_parameter<bool>("enable_debug");

  RCLCPP_INFO(get_logger(), "ignore_node_names: %zu entries", ignore_node_names_.size());
  RCLCPP_INFO(get_logger(), "ignore_exit_codes: %zu entries", ignore_exit_codes_.size());
  RCLCPP_INFO(get_logger(), "check_interval: %.2f", check_interval_);
  RCLCPP_INFO(get_logger(), "enable_debug: %s", enable_debug_ ? "true" : "false");

  sub_rosout_ = create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", 100, std::bind(&NodeDeathMonitor::on_log, this, std::placeholders::_1));

  auto interval_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(check_interval_));
  timer_ = create_wall_timer(interval_ns, std::bind(&NodeDeathMonitor::on_timer, this));

  if (enable_debug_) {
    RCLCPP_INFO(get_logger(), "[DEBUG] NodeDeathMonitor initialized, now subscribing to /rosout.");
  }
}

void NodeDeathMonitor::on_log(const rcl_interfaces::msg::Log::SharedPtr msg)
{
  if (enable_debug_) {
    RCLCPP_INFO(
      get_logger(), "[DEBUG] Received /rosout log: level=%d name=%s msg=%s", msg->level,
      msg->name.c_str(), msg->msg.c_str());
  }

  const std::string & text = msg->msg;
  const std::string target_str = "process has died";

  // "process has died" を含むかチェック
  if (text.find(target_str) == std::string::npos) {
    if (enable_debug_) {
      RCLCPP_INFO(get_logger(), "[DEBUG] The log does not contain '%s': skip", target_str.c_str());
    }
    return;
  }

  // exit code のパース (例: "exit code 139" を取得)
  // 参考: "[my_node-1] process has died [pid 12345, exit code 139, cmd '...']"
  int exit_code = -1;
  {
    static const std::regex exit_code_pattern("exit code\\s+([0-9]+)");
    std::smatch match_exit;
    if (std::regex_search(text, match_exit, exit_code_pattern)) {
      try {
        exit_code = std::stoi(match_exit[1]);
      } catch (...) {
        exit_code = -1;
      }
      if (enable_debug_) {
        RCLCPP_INFO(get_logger(), "[DEBUG] Parsed exit_code=%d from the log.", exit_code);
      }
    } else {
      if (enable_debug_) {
        RCLCPP_INFO(get_logger(), "[DEBUG] Could not parse exit_code from the log.");
      }
    }
  }

  // ---------------------------
  // 除外するexit codeの場合は無視
  // ---------------------------
  if (
    std::find(ignore_exit_codes_.begin(), ignore_exit_codes_.end(), exit_code) !=
    ignore_exit_codes_.end()) {
    if (enable_debug_) {
      RCLCPP_INFO(
        get_logger(),
        "[DEBUG] Ignoring process died log (exit_code=%d is in ignore_exit_codes_). Original: %s",
        exit_code, text.c_str());
    }
    return;
  }

  // ---------------------------
  // "[node_name-#]" を抽出
  // ---------------------------
  static const std::regex node_name_pattern("\\[([^\\]]+)\\] process has died");
  std::smatch match_node;
  if (std::regex_search(text, match_node, node_name_pattern)) {
    const std::string node_id = match_node[1];  // 例: "my_node-1"

    // (任意) ノードIDのデバッグ
    if (enable_debug_) {
      RCLCPP_INFO(get_logger(), "[DEBUG] Extracted node_id='%s'", node_id.c_str());
    }

    // -------------------------
    // ignore_node_names_ に含まれるなら無視
    // -------------------------
    for (const auto & ignore : ignore_node_names_) {
      if (node_id.find(ignore) != std::string::npos) {
        if (enable_debug_) {
          RCLCPP_INFO(
            get_logger(), "[DEBUG] Ignoring node death: node_id='%s' matched ignore='%s'",
            node_id.c_str(), ignore.c_str());
        }
        return;
      }
    }

    // -------------------------
    // 死亡ノードとして記録
    // -------------------------
    dead_nodes_[node_id] = true;

    RCLCPP_WARN(
      get_logger(), "Detected node death: %s (exit_code=%d, full_log='%s')", node_id.c_str(),
      exit_code, text.c_str());
  } else {
    // 正規表現で node_id を抽出できなかった場合
    if (enable_debug_) {
      RCLCPP_INFO(
        get_logger(), "[DEBUG] Could not extract [node_name-#] from the log, text='%s'",
        text.c_str());
    }
  }
}

void NodeDeathMonitor::on_timer()
{
  // 死んだノードがあれば定期的に一覧を出力
  if (!dead_nodes_.empty()) {
    std::string report = "Dead nodes detected: ";
    for (const auto & kv : dead_nodes_) {
      if (kv.second) {
        report += kv.first + " ";
      }
    }
    RCLCPP_INFO(get_logger(), "%s", report.c_str());
  } else if (enable_debug_) {
    // デバッグ時に "no dead nodes" を報告
    RCLCPP_INFO(get_logger(), "[DEBUG] on_timer: No dead nodes so far.");
  }
}

}  // namespace autoware::node_death_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::node_death_monitor::NodeDeathMonitor)
