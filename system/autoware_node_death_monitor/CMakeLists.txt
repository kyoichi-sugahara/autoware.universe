cmake_minimum_required(VERSION 3.14)
project(autoware_node_death_monitor)

###
# 1) Autoware/ROS関連のfind_package
###
find_package(autoware_cmake REQUIRED)  # もしAutowareのマクロを使うなら
autoware_package()

# あるいはAutowareなしの場合は、下記に置き換える
# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()


###
# 2) ライブラリ(コンポーネント)のビルド
###
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/autoware_node_death_monitor.cpp
)

# コンポーネント登録
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::node_death_monitor::NodeDeathMonitor"
  EXECUTABLE ${PROJECT_NAME}_node)

###
# 3) 単独実行ファイルの作成 (オプショナル)
#    => コンポーネントを直接実行するためのラッパノード
###
# target_link_libraries(${PROJECT_NAME}
# )


###
# 4) インストールおよびパッケージ化
###
# launchファイルや設定ファイルをインストール
ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
