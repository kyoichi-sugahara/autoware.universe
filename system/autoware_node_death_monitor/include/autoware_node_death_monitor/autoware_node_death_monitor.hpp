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

#ifndef AUTOWARE_NODE_DEATH_MONITOR__AUTOWARE_NODE_DEATH_MONITOR_HPP_
#define AUTOWARE_NODE_DEATH_MONITOR__AUTOWARE_NODE_DEATH_MONITOR_HPP_

#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::node_death_monitor
{

class NodeDeathMonitor : public rclcpp::Node
{
public:
  explicit NodeDeathMonitor(const rclcpp::NodeOptions & options);

private:
  // ログ購読コールバック
  void on_log(const rcl_interfaces::msg::Log::SharedPtr msg);
  // 定期処理（死んだノード一覧の報告やクリアなど）
  void on_timer();

  // 死んだノードを記録: [node_name-#] -> true
  std::unordered_map<std::string, bool> dead_nodes_;

  // ROS通信 (購読/タイマー)
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_rosout_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- 以下、declare_parameter で取得するパラメータ例 ---
  // 監視から除外したいノード名
  std::vector<std::string> ignore_node_names_;
  // 監視から除外したい終了コード (正常終了など)
  std::vector<int64_t> ignore_exit_codes_;
  // 定期チェック周期 (秒)
  double check_interval_{1.0};
  // デバッグ出力を有効にするか
  bool enable_debug_{false};
};

}  // namespace autoware::node_death_monitor

#endif  // AUTOWARE_NODE_DEATH_MONITOR__AUTOWARE_NODE_DEATH_MONITOR_HPP_
