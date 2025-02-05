# node_death_monitor

This package provides a simple monitoring node to detect other ROS 2 node processes dying by observing `rosout` log messages containing `process has died`.

## Overview

- **Node name**: `node_death_monitor`
- **Message**: Subscribes to `/rosout` (type: `rcl_interfaces/msg/Log`)
- **Detected event**: Looks for log lines containing the substring `"process has died"` and extracts the node/command name from the log text.

## How it Works

1. Any node launched via `ros2 launch` will output a log line like:
   [node_name-1] process has died [pid 1234, exit code 139, cmd '...']
   when it crashes or exits unexpectedly.
2. `node_death_monitor` subscribes to `/rosout` and checks each incoming log message for the substring `"process has died"`.
3. When a match is found, it extracts the bracketed text `[node_name-1]` as the identifier of the dead node, and logs a warning message or stores it in an internal map.

## Launch

### XML launch example

See `launch/node_death_monitor.launch.xml` for a sample of how to run this node.

```bash
ros2 launch node_death_monitor node_death_monitor.launch.xml
```

By default, it will:

Create a node called node_death_monitor
Subscribe to /rosout
Print any detected node death in its own logs

---

# 4. launch.xml

以下は XML 形式のサンプルランチファイルです。  
ここでは「単独ノード実行ファイル」で起動する例を示します。

> **ポイント**
>
> - `exec="node_death_monitor_exe"` は、CMakeLists にて `ament_auto_add_executable(${PROJECT_NAME}_exe ...)` でビルドしたバイナリ名を想定。
> - もしコンポーネントとして動かす場合は、`component_container` を使った構成に書き換えてください。

```xml
<launch>
  <!--
      Example: node_death_monitor を単独ノードとして起動
      (CMakeListsで ament_auto_add_executable(node_death_monitor_exe ...) とした場合)

      もしコンポーネントで起動するなら:
        <node pkg="rclcpp_components" exec="component_container" name="node_death_monitor_container" output="screen">
          <param from="$(find-pkg-share node_death_monitor)/config/param.yaml" />
          <composition>
            <plugin>autoware::node_death_monitor::NodeDeathMonitor</plugin>
            <name>node_death_monitor</name>
          </composition>
        </node>
  -->

  <node
    pkg="node_death_monitor"
    exec="node_death_monitor_exe"
    name="node_death_monitor"
    output="screen"
  >
    <!-- 任意のパラメータファイルがある場合 -->
    <param from="$(find-pkg-share node_death_monitor)/config/node_death_monitor.param.yaml"/>
  </node>
</launch>

```

- find-pkg-share は ament_index からパッケージのshareディレクトリを探す際に使われる表記です。（$(find node_death_monitor) など互換の記法もありますが、ROS 2 ではPython launchファイルでの get_package_share_directory() が一般的です。）
- <composition> タグなどを使えば、ComposableNodeContainer形式で起動可能です。
