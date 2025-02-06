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

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace autoware::node_death_monitor
{

// ヘルパー関数: ~/.ros/log または ROS_LOG_DIR を探し、
// その中で最新のセッションディレクトリを特定し、launch.log のパスを返す
static fs::path find_latest_launch_log()
{
  // 1) ROS_LOG_DIR 環境変数を確認
  const char * ros_log_dir_env = std::getenv("ROS_LOG_DIR");
  fs::path base_path;
  if (ros_log_dir_env) {
    base_path = fs::path(ros_log_dir_env);
  } else {
    // なければ ~/.ros/log にする
    const char * home_env = std::getenv("HOME");
    if (!home_env) {
      // HOME が取れなければ仕方ないのでカレントディレクトリを使う例
      base_path = fs::current_path();
    } else {
      base_path = fs::path(home_env) / ".ros" / "log";
    }
  }

  if (!fs::exists(base_path) || !fs::is_directory(base_path)) {
    // ログディレクトリが存在しない場合は空パス返す
    return fs::path();
  }

  // 2) base_path 以下を走査し、最新(更新時刻が最大)のディレクトリを探す
  fs::path latest_dir;
  auto latest_time = fs::file_time_type::min();

  for (auto & entry : fs::directory_iterator(base_path)) {
    if (entry.is_directory()) {
      // ディレクトリの更新時刻を取得
      auto ftime = fs::last_write_time(entry.path());
      if (ftime > latest_time) {
        latest_time = ftime;
        latest_dir = entry.path();
      }
    }
  }

  if (latest_dir.empty()) {
    return fs::path();  // ディレクトリが無い場合
  }

  // 3) latest_dir/launch.log
  fs::path log_file = latest_dir / "launch.log";
  if (fs::exists(log_file) && fs::is_regular_file(log_file)) {
    return log_file;
  }
  return fs::path();  // launch.log が無い場合
}

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

  // ---- ここで最新の launch.log を特定 ----
  launch_log_path_ = find_latest_launch_log();
  if (launch_log_path_.empty()) {
    RCLCPP_WARN(get_logger(), "Could not find latest launch.log. Monitoring disabled.");
  } else {
    RCLCPP_WARN(get_logger(), "Monitoring launch.log at: %s", launch_log_path_.c_str());
  }

  // この時点でファイルサイズを取得して、そこから読み始めるようにする(差分読み)
  last_file_pos_ = 0;
  if (!launch_log_path_.empty() && fs::exists(launch_log_path_)) {
    auto raw_size = fs::file_size(launch_log_path_);
    last_file_pos_ = raw_size;

    if (enable_debug_) {
      RCLCPP_WARN(
        get_logger(),
        "File size details - Raw size (uintmax_t): %ju, Stored position (size_t): %zu", raw_size,
        last_file_pos_);
    }
  }

  // sub_rosout_ = create_subscription<rcl_interfaces::msg::Log>(
  //   "/rosout", 100, std::bind(&NodeDeathMonitor::on_log, this, std::placeholders::_1));

  // ------ タイマー ----
  auto interval_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(check_interval_));
  timer_ = create_wall_timer(interval_ns, std::bind(&NodeDeathMonitor::on_timer, this));
}

// on_log() は不要になったので削除してもOK
// (下記の parseLine() 的な関数にする方法も)

//---------------------------------------------------------------------------
// launch.log から新規追記分を読み込み、
// "process has died" を含む行を解析して処理する
//---------------------------------------------------------------------------
void NodeDeathMonitor::read_launch_log_diff()
{
  if (launch_log_path_.empty()) {
    return;
  }

  std::ifstream ifs(launch_log_path_, std::ios::binary);
  if (!ifs.good()) {
    RCLCPP_WARN(get_logger(), "Failed to open launch.log: %s", launch_log_path_.c_str());
    return;
  }

  // ファイル全体をシークしてサイズを取得
  ifs.seekg(0, std::ios::end);
  const std::streampos file_end = ifs.tellg();

  // 前回の読み取り位置がファイルサイズを超えていたら(ログローテ等) 先頭から読む
  if (last_file_pos_ > static_cast<size_t>(file_end)) {
    RCLCPP_WARN(
      get_logger(),
      "File size is reset. Possibly new session? Reading from top. last_file_pos_: %zu, file_end: "
      "%zu",
      last_file_pos_, static_cast<size_t>(file_end));
    last_file_pos_ = 0;
  }

  // 前回の位置までシーク
  ifs.seekg(last_file_pos_, std::ios::beg);

  if (enable_debug_) {
    RCLCPP_WARN(
      get_logger(), "[DEBUG] Reading launch.log from pos=%zu to end=%zu",
      static_cast<size_t>(last_file_pos_), static_cast<size_t>(file_end));
  }

  std::streampos last_valid_pos = static_cast<std::streampos>(last_file_pos_);

  size_t iteration = 0;
  while (true) {
    // 1) 現在位置チェック
    std::streampos current_pos_start = ifs.tellg();
    if (current_pos_start == std::streampos(-1)) {
      // すでにEOF or エラーかもしれない
      if (ifs.eof()) {
        RCLCPP_DEBUG(get_logger(), "EOF reached at iteration=%zu", iteration);
      } else {
        RCLCPP_WARN(
          get_logger(), "tellg() failed at iteration=%zu. Possibly file closed?", iteration);
      }
      break;  // ループ抜ける
    }

    // 2) 一行読み込み
    std::string line;
    if (!std::getline(ifs, line)) {
      if (ifs.eof()) {
        RCLCPP_DEBUG(get_logger(), "Reached EOF at iteration=%zu", iteration);
      } else {
        RCLCPP_WARN(get_logger(), "Error reading line at iteration=%zu", iteration);
      }
      break;
    }

    // 3) 行をパース
    parse_log_line(line);

    // 4) 行読み込み後にファイル位置を取得
    std::streampos current_pos_end = ifs.tellg();
    if (current_pos_end == std::streampos(-1)) {
      // EOFかもしれないし、エラーかもしれない
      if (ifs.eof()) {
        // 「最後の行は読めたが、次の読み込みでEOFになった」ケースが多い
        RCLCPP_DEBUG(get_logger(), "EOF after iteration=%zu", iteration);
      } else {
        RCLCPP_WARN(get_logger(), "tellg() failed after reading line at iteration=%zu", iteration);
      }
      // ただし、「最後の行」はすでに読み込めているので、ここでブレークする
      break;
    }

    // ここに到達したということは「行の読み込み成功」+「tellg() != -1」で有効
    last_valid_pos = current_pos_end;
    ++iteration;
  }

  // ループが終了したら、last_valid_pos が「直近の有効位置」
  if (last_valid_pos != std::streampos(-1)) {
    last_file_pos_ = static_cast<size_t>(last_valid_pos);
    RCLCPP_DEBUG(get_logger(), "Set last_file_pos_=%zu after reading", last_file_pos_);
  } else {
    RCLCPP_WARN(get_logger(), "No valid position found at the end");
  }
}

//---------------------------------------------------------------------------
// 1行分の "process has died" ログ解析
//---------------------------------------------------------------------------
void NodeDeathMonitor::parse_log_line(const std::string & line)
{
  const std::string target_str = "process has died";
  if (line.find(target_str) == std::string::npos) {
    if (enable_debug_) {
      RCLCPP_INFO(
        get_logger(), "[DEBUG] The log line does not contain '%s': skip\nline='%s'",
        target_str.c_str(), line.c_str());
    }
    return;
  }

  // exit code のパース
  int exit_code = -1;
  {
    static const std::regex exit_code_pattern("exit code\\s+([0-9]+)");
    std::smatch match_exit;
    if (std::regex_search(line, match_exit, exit_code_pattern)) {
      try {
        exit_code = std::stoi(match_exit[1]);
      } catch (...) {
        exit_code = -1;
      }
      if (enable_debug_) {
        RCLCPP_INFO(get_logger(), "[DEBUG] Parsed exit_code=%d from log line.", exit_code);
      }
    } else {
      if (enable_debug_) {
        RCLCPP_INFO(get_logger(), "[DEBUG] Could not parse exit_code from log line.");
      }
    }
  }

  // exit_code フィルタ
  if (
    std::find(ignore_exit_codes_.begin(), ignore_exit_codes_.end(), exit_code) !=
    ignore_exit_codes_.end()) {
    if (enable_debug_) {
      RCLCPP_INFO(
        get_logger(),
        "[DEBUG] Ignoring process died log (exit_code=%d is in ignore_exit_codes_). line='%s'",
        exit_code, line.c_str());
    }
    return;
  }

  // "[component_container_mt-56]: process has died" 部分を抽出
  static const std::regex node_name_pattern("\\[([^\\]]+)\\]\\:\\s*process has died");
  std::smatch match_node;
  if (std::regex_search(line, match_node, node_name_pattern)) {
    const std::string node_id = match_node[1];
    if (enable_debug_) {
      RCLCPP_INFO(get_logger(), "[DEBUG] Extracted node_id='%s'", node_id.c_str());
    }

    // ignore_node_names_ に含まれるなら無視
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

    // dead_nodes_ に登録
    dead_nodes_[node_id] = true;

    // ログ出力
    RCLCPP_WARN(
      get_logger(), "Detected node death from launch.log: node_id='%s' (exit_code=%d)\n  line='%s'",
      node_id.c_str(), exit_code, line.c_str());
  } else {
    if (enable_debug_) {
      RCLCPP_INFO(
        get_logger(),
        "[DEBUG] Could not extract [node_name-#] (with ': process has died') from log line='%s'",
        line.c_str());
    }
  }
}

//---------------------------------------------------------------------------
// タイマーコールバック
//---------------------------------------------------------------------------
void NodeDeathMonitor::on_timer()
{
  // 1) launch.log の差分を読み取り
  read_launch_log_diff();

  // 2) 死んだノード一覧を出力
  if (!dead_nodes_.empty()) {
    std::string report = "Dead nodes detected: ";
    for (const auto & kv : dead_nodes_) {
      if (kv.second) {
        report += kv.first + " ";
      }
    }
    RCLCPP_INFO(get_logger(), "%s", report.c_str());
  } else if (enable_debug_) {
    RCLCPP_INFO(get_logger(), "[DEBUG] on_timer: No dead nodes so far.");
  }
}

}  // namespace autoware::node_death_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::node_death_monitor::NodeDeathMonitor)
