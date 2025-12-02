// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_HPP_
#define AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_HPP_

#include "autoware/lidar_frnet/point_type.hpp"
#include "autoware/lidar_frnet/postprocess_kernel.hpp"
#include "autoware/lidar_frnet/preprocess_kernel.hpp"
#include "autoware/lidar_frnet/utils.hpp"
#include "visibility_control.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/logger.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::lidar_frnet
{
using cuda_utils::CudaUniquePtr;
using tensorrt_common::NetworkIO;
using tensorrt_common::ProfileDims;
using tensorrt_common::TrtCommon;
using tensorrt_common::TrtCommonConfig;

class LIDAR_FRNET_PUBLIC LidarFRNet
{
public:
  LidarFRNet(
    const tensorrt_common::TrtCommonConfig & trt_config,
    const utils::NetworkParams & network_params,
    const utils::PreprocessingParams & preprocessing_params,
    const utils::PostprocessingParams & postprocessing_params, const rclcpp::Logger & logger);
  ~LidarFRNet() = default;

  bool process(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & cloud_in,
    const utils::ActiveComm & active_comm, std::unordered_map<std::string, double> & proc_timing);

  void setPublishSegmentedPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
  {
    publish_segmented_pointcloud_ = std::move(func);
  }

  void setPublishVisualizationPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
  {
    publish_visualization_pointcloud_ = std::move(func);
  }

  void setPublishFilteredPointcloud(
    std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
  {
    publish_filtered_pointcloud_ = std::move(func);
  }

private:
  bool preprocess(const uint32_t input_num_points);
  bool inference();
  bool postprocess(
    const uint32_t input_num_points, const std_msgs::msg::Header & header,
    const utils::ActiveComm & active_comm);
  void initTensors();

  /**
   * @brief Initialize and store point cloud layouts, then allocate output messages
   *
   * This method stores the point cloud layouts for segmentation, visualization, and filtered
   * outputs, and allocates CUDA memory for the output messages. This should be called once
   * during initialization.
   *
   * @param cloud_seg_layout Layout configuration for segmentation point cloud
   * @param cloud_viz_layout Layout configuration for visualization point cloud
   * @param cloud_filtered_layout Layout configuration for filtered point cloud
   */
  void allocateMessages(
    const ros_utils::PointCloudLayout & cloud_seg_layout,
    const ros_utils::PointCloudLayout & cloud_viz_layout,
    const ros_utils::PointCloudLayout & cloud_filtered_layout);

  /**
   * @brief Allocate output messages using stored point cloud layouts
   *
   * This method reallocates CUDA memory for output messages that were previously published
   * and set to nullptr. It uses the layouts stored by the parameterized version of
   * allocateMessages(). This is called automatically at the end of postprocess() to prepare
   * messages for the next processing cycle.
   */
  void allocateMessages();

  std::once_flag init_cloud_;

  const rclcpp::Logger logger_;

  std::unique_ptr<TrtCommon> network_ptr_{nullptr};
  std::unique_ptr<PreprocessCuda> preprocess_ptr_{nullptr};
  std::unique_ptr<PostprocessCuda> postprocess_ptr_{nullptr};
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  utils::NetworkParams network_params_;
  utils::PreprocessingParams preprocessing_params_;
  utils::PostprocessingParams postprocessing_params_;

  // Callback functions for publishing
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_segmented_pointcloud_;
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_visualization_pointcloud_;
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)>
    publish_filtered_pointcloud_;

  // Output messages
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> cloud_seg_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> cloud_viz_msg_ptr_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> cloud_filtered_msg_ptr_{nullptr};

  // Point cloud layouts
  ros_utils::PointCloudLayout cloud_seg_layout_;
  ros_utils::PointCloudLayout cloud_viz_layout_;
  ros_utils::PointCloudLayout cloud_filtered_layout_;

  cudaStream_t stream_;
  // Inference
  CudaUniquePtr<float[]> points_d_{nullptr};         // N x 4 (x, y, z, intensity)
  CudaUniquePtr<int64_t[]> coors_d_{nullptr};        // N x 3 (0, y, x)
  CudaUniquePtr<int64_t[]> voxel_coors_d_{nullptr};  // M x 3 (0, y, x)
  CudaUniquePtr<int64_t[]> inverse_map_d_{nullptr};  // N
  CudaUniquePtr<float[]> seg_logit_d_{nullptr};      // NUM_CLASSES x 1
  // Preprocess & Postprocess
  CudaUniquePtr<InputPointType[]> cloud_in_d_{nullptr};
  CudaUniquePtr<int64_t[]> coors_keys_d_{nullptr};
  CudaUniquePtr<uint32_t[]> num_points_d_{nullptr};
  CudaUniquePtr<uint32_t[]> proj_idxs_d_{nullptr};
  CudaUniquePtr<uint64_t[]> proj_2d_d_{nullptr};
  CudaUniquePtr<OutputSegmentationPointType[]> seg_data_d_{nullptr};
  CudaUniquePtr<OutputVisualizationPointType[]> viz_data_d_{nullptr};
  CudaUniquePtr<InputPointType[]> cloud_filtered_d_{nullptr};
  CudaUniquePtr<uint32_t[]> num_points_filtered_d_{nullptr};
};

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_HPP_
