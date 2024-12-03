// Copyright 2024 TIER IV, Inc.
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

#include "autoware/lane_departure_checker/utils.hpp"

#include <Eigen/Core>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <gtest/gtest.h>

using autoware::lane_departure_checker::utils::createVehiclePassingAreas;
using autoware::universe_utils::LinearRing2d;

class CreateVehiclePassingAreasTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    // 時計回りで点を定義
    square1_.reserve(5);
    square1_.push_back({0.0, 0.0});  // 左下
    square1_.push_back({0.0, 1.0});  // 左上
    square1_.push_back({1.0, 1.0});  // 右上
    square1_.push_back({1.0, 0.0});  // 右下
    square1_.push_back({0.0, 0.0});  // 閉じる

    square2_.reserve(5);
    square2_.push_back({1.0, 0.0});  // 左下
    square2_.push_back({1.0, 1.0});  // 左上
    square2_.push_back({2.0, 1.0});  // 右上
    square2_.push_back({2.0, 0.0});  // 右下
    square2_.push_back({1.0, 0.0});  // 閉じる

    // 多角形の方向を正しく修正
    boost::geometry::correct(square1_);
    boost::geometry::correct(square2_);

    // // デバッグ情報の出力
    // std::cout << "Square1 points after correction:" << std::endl;
    // for (const auto & p : square1_) {
    //   std::cout << "  (" << p.x() << ", " << p.y() << ")" << std::endl;
    // }

    // std::string reason;
    // bool valid = boost::geometry::is_valid(square1_, reason);
    // if (!valid) {
    //   std::cout << "Square1 is invalid: " << reason << std::endl;
    // }
  }

  LinearRing2d square1_;  // 1x1の正方形
  LinearRing2d square2_;  // 隣接する1x1の正方形
};

TEST_F(CreateVehiclePassingAreasTest, EmptyInput)
{
  const std::vector<LinearRing2d> empty_footprints;
  const auto areas = createVehiclePassingAreas(empty_footprints);
  EXPECT_TRUE(areas.empty());
}

TEST_F(CreateVehiclePassingAreasTest, SingleFootprint)
{
  const std::vector<LinearRing2d> single_footprint = {square1_};
  const auto areas = createVehiclePassingAreas(single_footprint);

  ASSERT_EQ(areas.size(), 1);

  // 結果の多角形の方向を修正
  auto result = areas.front();
  boost::geometry::correct(result);

  EXPECT_EQ(result, square1_);
}

TEST_F(CreateVehiclePassingAreasTest, MultipleFootprints)
{
  const std::vector<LinearRing2d> footprints = {square1_, square2_};
  auto areas = createVehiclePassingAreas(footprints);

  ASSERT_EQ(areas.size(), 1);
  auto & hull = areas.front();

  // 結果の多角形の方向を修正
  boost::geometry::correct(hull);

  // 凸包は少なくとも四角形になるはず
  EXPECT_GE(hull.size(), 5);  // 4つの点 + 閉じるための重複点

  // デバッグ用に座標を出力
  // std::cout << "Hull points after correction:" << std::endl;
  for (const auto & p : hull) {
    std::cout << "  (" << p.x() << ", " << p.y() << ")" << std::endl;
  }

  // 元の頂点が全て凸包内に含まれることを確認
  for (const auto & footprint : footprints) {
    for (size_t j = 0; j < footprint.size() - 1; ++j) {  // 最後の点は重複なのでスキップ
      const auto & point = footprint[j];

      // within または covered_by を使用して点が領域内/境界上にあるかチェック
      bool is_inside =
        boost::geometry::within(point, hull) || boost::geometry::covered_by(point, hull);

      if (!is_inside) {
        std::cout << "Point not inside hull: (" << point.x() << ", " << point.y() << ")"
                  << std::endl;
      }

      EXPECT_TRUE(is_inside) << "Point (" << point.x() << ", " << point.y()
                             << ") is not inside the hull";
    }
  }
}
