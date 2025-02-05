# autoware_node_death_monitor

This package provides a simple monitoring node to detect other ROS 2 node processes dying by observing `/rosout` log messages containing `process has died`.

---

## Overview

- **Node name**: `autoware_node_death_monitor`
- **Subscribed topic**: `/rosout` (type: `rcl_interfaces/msg/Log`)
- **Detected event**: Looks for log lines containing the substring `"process has died"` and extracts the node/command name from the log text.

When a crash or unexpected shutdown occurs, `ros2 launch` typically outputs a line such as:

```
[node_name-1] process has died [pid 12345, exit code 139, cmd '...']
```

The `autoware_node_death_monitor` node parses these messages and logs a warning or marks the node as "dead."

---

## How it Works

1. **Any node** launched via `ros2 launch` that dies unexpectedly produces a log line in `/rosout` that includes `"process has died"`.
2. **`autoware_node_death_monitor`** subscribes to `/rosout` and scans each incoming log message for `"process has died"`.
3. If found, it extracts the bracketed text `[node_name-#]` to identify the failing node.
4. It also tries to parse the **exit code** from the message (e.g., `exit code 139`) to differentiate between abnormal (e.g., segfault) and user-intended exits (e.g., 0, 130).
5. The node then checks for additional filters, such as:
   - **Ignored node names**: If `rviz2` is in the ignore list, it won’t flag it as an error.
   - **Ignored exit codes**: If `0` or `130` is in the ignore list, those will be skipped (often indicating normal or Ctrl+C exits).
6. A simple timer callback periodically prints out a list of dead nodes. In a real system, this could be extended to publish diagnostics or trigger an alert.

---

## Parameters

The node uses `declare_parameter` for several settings, which can be supplied via a YAML file (e.g., `config/topics.yaml`). Below are the main parameters:

| Parameter Name      | Type       | Default           | Description                                               |
| ------------------- | ---------- | ----------------- | --------------------------------------------------------- |
| `ignore_node_names` | `string[]` | `[]` (empty list) | Node name patterns to ignore. E.g., `["rviz2"]`.          |
| `ignore_exit_codes` | `int[]`    | `[0, 130]`        | Exit codes to ignore (normal or user-intended).           |
| `check_interval`    | `double`   | `1.0`             | Timer interval (in seconds) to print detected dead nodes. |
| `enable_debug`      | `bool`     | `false`           | If `true`, debug logs are printed for ignored items, etc. |

Example **`topics.yaml`**:

```yaml
autoware_node_death_monitor:
  ros__parameters:
    ignore_node_names:
      - rviz2
      - teleop_twist_joy
    ignore_exit_codes:
      - 0
      - 130
    check_interval: 1.0
    enable_debug: false
```

---

## Launch

### 1) Standalone Node Execution

If your `CMakeLists.txt` creates a standalone executable named `autoware_node_death_monitor_exe`, you can launch it via an XML file such as:

**`launch/autoware_node_death_monitor.launch.xml`**

```xml
<launch>
  <node
    pkg="autoware_node_death_monitor"
    exec="autoware_node_death_monitor_exe"
    name="autoware_node_death_monitor"
    output="screen"
  >
    <!-- Load parameters from YAML -->
    <param from="$(find-pkg-share autoware_node_death_monitor)/config/topics.yaml" />
  </node>
</launch>
```

Then run:

```bash
ros2 launch autoware_node_death_monitor autoware_node_death_monitor.launch.xml
```

By default, it will:

- Create a node called **`autoware_node_death_monitor`**
- Subscribe to **`/rosout`**
- Parse any message containing `"process has died"`
- Print out or log the names of dead nodes at the specified interval

### 2) Composable Node Execution

If you prefer to run this node as a component in a container, you can use `component_container`:

```xml
<launch>
  <node
    pkg="rclcpp_components"
    exec="component_container"
    name="autoware_node_death_monitor_container"
    output="screen"
  >
    <composition>
      <plugin>autoware::node_death_monitor::NodeDeathMonitor</plugin>
      <name>autoware_node_death_monitor</name>
      <param from="$(find-pkg-share autoware_node_death_monitor)/config/topics.yaml" />
    </composition>
  </node>
</launch>
```

Then:

```bash
ros2 launch autoware_node_death_monitor autoware_node_death_monitor.launch.xml
```

---

## Advanced Usage

- **Additional Filtering**: The code can be extended to parse node name patterns or more complex logic (e.g., partial matches).
- **Reporting**: Instead of only logging, you can publish diagnostic messages or trigger fault recovery actions in a larger Autoware system.
- **Exit Code Analysis**: If needed, you can parse specific non-zero codes differently (e.g., `139` as segfault).
- **Integration**: This node can run in parallel with system-level monitors (e.g., systemd or supervisord) for more robust fault detection.

---
