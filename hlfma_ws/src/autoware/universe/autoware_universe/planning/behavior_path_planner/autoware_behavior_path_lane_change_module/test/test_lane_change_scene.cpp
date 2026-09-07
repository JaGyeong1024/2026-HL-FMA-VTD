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

#include "autoware/behavior_path_lane_change_module/manager.hpp"
#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_lane_change_module/structs/data.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>

using autoware::behavior_path_planner::FilteredLanesObjects;
using autoware::behavior_path_planner::LaneChangeModuleManager;
using autoware::behavior_path_planner::LaneChangeModuleType;
using autoware::behavior_path_planner::NormalLaneChange;
using autoware::behavior_path_planner::PlannerData;
using autoware::behavior_path_planner::lane_change::CommonDataPtr;
using autoware::behavior_path_planner::lane_change::LCParamPtr;
using autoware::behavior_path_planner::lane_change::RouteHandlerPtr;
using autoware::route_handler::Direction;
using autoware::route_handler::RouteHandler;
using autoware::test_utils::get_absolute_path_to_config;
using autoware::test_utils::get_absolute_path_to_lanelet_map;
using autoware::test_utils::get_absolute_path_to_route;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::Pose;

class TestNormalLaneChange : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    init_param();
    init_module();
  }

  void init_param()
  {
    auto node_options = get_node_options();
    auto node = rclcpp::Node(name_, node_options);
    planner_data_->init_parameters(node);
    lc_param_ptr_ = LaneChangeModuleManager::set_params(&node, node.get_name());
    planner_data_->route_handler = init_route_handler();

    ego_pose_ = autoware::test_utils::createPose(-50.0, 1.75, 0.0, 0.0, 0.0, 0.0);
    planner_data_->self_odometry = set_odometry(ego_pose_);
    const auto objects_file =
      ament_index_cpp::get_package_share_directory("autoware_behavior_path_lane_change_module") +
      "/test_data/test_object_filter.yaml";

    YAML::Node yaml_node = YAML::LoadFile(objects_file);
    const auto objects = autoware::test_utils::parse<PredictedObjects>(yaml_node);
    planner_data_->dynamic_object = std::make_shared<PredictedObjects>(objects);
  }

  void init_module()
  {
    normal_lane_change_ =
      std::make_shared<NormalLaneChange>(lc_param_ptr_, lc_type_, lc_direction_);
    normal_lane_change_->setData(planner_data_);
    set_previous_approved_path();
  }

  [[nodiscard]] const CommonDataPtr & get_common_data_ptr() const
  {
    return normal_lane_change_->common_data_ptr_;
  }

  [[nodiscard]] rclcpp::NodeOptions get_node_options() const
  {
    auto node_options = rclcpp::NodeOptions{};

    const auto common_param =
      get_absolute_path_to_config(test_utils_dir_, "test_common.param.yaml");
    const auto nearest_search_param =
      get_absolute_path_to_config(test_utils_dir_, "test_nearest_search.param.yaml");
    const auto vehicle_info_param =
      get_absolute_path_to_config(test_utils_dir_, "test_vehicle_info.param.yaml");

    std::string bpp_dir{"autoware_behavior_path_planner"};
    const auto bpp_param = get_absolute_path_to_config(bpp_dir, "behavior_path_planner.param.yaml");
    const auto drivable_area_expansion_param =
      get_absolute_path_to_config(bpp_dir, "drivable_area_expansion.param.yaml");
    const auto scene_module_manager_param =
      get_absolute_path_to_config(bpp_dir, "scene_module_manager.param.yaml");

    std::string lc_dir{"autoware_behavior_path_lane_change_module"};
    const auto lc_param = get_absolute_path_to_config(lc_dir, "lane_change.param.yaml");

    autoware::test_utils::updateNodeOptions(
      node_options, {common_param, nearest_search_param, vehicle_info_param, bpp_param,
                     drivable_area_expansion_param, scene_module_manager_param, lc_param});
    return node_options;
  }

  [[nodiscard]] RouteHandlerPtr init_route_handler() const
  {
    std::string autoware_route_handler_dir{"autoware_route_handler"};
    std::string lane_change_right_test_route_filename{"lane_change_test_route.yaml"};
    std::string lanelet_map_filename{"2km_test.osm"};
    const auto lanelet2_path =
      get_absolute_path_to_lanelet_map(test_utils_dir_, lanelet_map_filename);
    const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(lanelet2_path, 5.0);
    auto route_handler_ptr = std::make_shared<RouteHandler>(map_bin_msg);
    const auto rh_test_route =
      get_absolute_path_to_route(autoware_route_handler_dir, lane_change_right_test_route_filename);
    if (
      const auto route_opt =
        autoware::test_utils::parse<std::optional<LaneletRoute>>(rh_test_route)) {
      route_handler_ptr->setRoute(*route_opt);
    }

    return route_handler_ptr;
  }

  static std::shared_ptr<nav_msgs::msg::Odometry> set_odometry(const Pose & pose)
  {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose = pose;
    return std::make_shared<nav_msgs::msg::Odometry>(odom);
  }

  [[nodiscard]] FilteredLanesObjects & get_filtered_objects() const
  {
    return normal_lane_change_->filtered_objects_;
  }

  void set_previous_approved_path()
  {
    normal_lane_change_->prev_module_output_.path = create_previous_approved_path();
  }

  [[nodiscard]] PathWithLaneId create_previous_approved_path() const
  {
    const auto & common_data_ptr = get_common_data_ptr();
    const auto & route_handler_ptr = common_data_ptr->route_handler_ptr;
    lanelet::ConstLanelet closest_lane;
    const auto current_pose = planner_data_->self_odometry->pose.pose;
    route_handler_ptr->getClosestLaneletWithinRoute(current_pose, &closest_lane);
    const auto backward_distance = common_data_ptr->bpp_param_ptr->backward_path_length;
    const auto forward_distance = common_data_ptr->bpp_param_ptr->forward_path_length;
    const auto current_lanes = route_handler_ptr->getLaneletSequence(
      closest_lane, current_pose, backward_distance, forward_distance);

    return route_handler_ptr->getCenterLinePath(
      current_lanes, 0.0, std::numeric_limits<double>::max());
  }

  void TearDown() override
  {
    normal_lane_change_ = nullptr;
    lc_param_ptr_ = nullptr;
    planner_data_ = nullptr;
    rclcpp::shutdown();
  }

  LCParamPtr lc_param_ptr_;
  std::shared_ptr<NormalLaneChange> normal_lane_change_;
  std::shared_ptr<PlannerData> planner_data_ = std::make_shared<PlannerData>();
  LaneChangeModuleType lc_type_{LaneChangeModuleType::NORMAL};
  Direction lc_direction_{Direction::RIGHT};
  std::string name_{"test_lane_change_scene"};
  std::string test_utils_dir_{"autoware_test_utils"};
  Pose ego_pose_;
};

TEST_F(TestNormalLaneChange, testBaseClassInitialize)
{
  const auto type = normal_lane_change_->getModuleType();
  const auto type_str = normal_lane_change_->getModuleTypeStr();

  ASSERT_EQ(type, LaneChangeModuleType::NORMAL);
  const auto is_type_str = type_str == "NORMAL";
  ASSERT_TRUE(is_type_str);

  ASSERT_EQ(normal_lane_change_->getDirection(), Direction::RIGHT);

  ASSERT_TRUE(get_common_data_ptr());

  ASSERT_TRUE(get_common_data_ptr()->is_data_available());
  ASSERT_FALSE(get_common_data_ptr()->is_lanes_available());
}

TEST_F(TestNormalLaneChange, testUpdateLanes)
{
  constexpr auto is_approved = true;

  normal_lane_change_->update_lanes(is_approved);

  ASSERT_FALSE(get_common_data_ptr()->is_lanes_available());

  normal_lane_change_->update_lanes(!is_approved);

  ASSERT_TRUE(get_common_data_ptr()->is_lanes_available());
}

TEST_F(TestNormalLaneChange, testGetPathWhenInvalid)
{
  constexpr auto is_approved = true;
  normal_lane_change_->update_lanes(!is_approved);
  normal_lane_change_->update_filtered_objects();
  normal_lane_change_->update_transient_data(!is_approved);
  normal_lane_change_->updateLaneChangeStatus();
  const auto & lc_status = normal_lane_change_->getLaneChangeStatus();

  ASSERT_FALSE(lc_status.is_valid_path);
}

// TODO(Azu, Quda): Fix this test
TEST_F(TestNormalLaneChange, DISABLED_testFilteredObjects)
{
  constexpr auto is_approved = true;
  ego_pose_ = autoware::test_utils::createPose(1.0, 1.75, 0.0, 0.0, 0.0, 0.0);
  planner_data_->self_odometry = set_odometry(ego_pose_);
  set_previous_approved_path();

  normal_lane_change_->update_lanes(!is_approved);
  normal_lane_change_->update_filtered_objects();

  const auto & filtered_objects = get_filtered_objects();

  const auto filtered_size =
    filtered_objects.current_lane.size() + filtered_objects.target_lane_leading.size() +
    filtered_objects.target_lane_trailing.size() + filtered_objects.others.size();
  EXPECT_EQ(filtered_size, planner_data_->dynamic_object->objects.size());
  EXPECT_EQ(filtered_objects.current_lane.size(), 1);
  EXPECT_EQ(filtered_objects.target_lane_leading.size(), 2);
  EXPECT_EQ(filtered_objects.target_lane_trailing.size(), 0);
  EXPECT_EQ(filtered_objects.others.size(), 1);
}

TEST_F(TestNormalLaneChange, testGetPathWhenValid)
{
  constexpr auto is_approved = true;
  ego_pose_ = autoware::test_utils::createPose(1.0, 1.75, 0.0, 0.0, 0.0, 0.0);
  planner_data_->self_odometry = set_odometry(ego_pose_);
  normal_lane_change_->setData(planner_data_);
  set_previous_approved_path();
  normal_lane_change_->update_lanes(!is_approved);
  normal_lane_change_->update_filtered_objects();
  normal_lane_change_->update_transient_data(!is_approved);
  const auto err = normal_lane_change_->isLaneChangeRequired();

  ASSERT_FALSE(err);

  normal_lane_change_->updateLaneChangeStatus();
  const auto & lc_status = normal_lane_change_->getLaneChangeStatus();

  ASSERT_TRUE(lc_status.is_valid_path);
}

// ---------------------------------------------------------------------------
// HL FMA 하네스: external_request 차선변경 패치 P0/P1/P2 (수정 전 red, 수정 후 green)
// 근거: docs 스크래치 adv_bundle2.md D1/D4, NG 68aeeb5
// ---------------------------------------------------------------------------
#include "autoware/behavior_path_lane_change_module/utils/calculation.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>

#include <set>

namespace
{
constexpr std::array<int64_t, 6> kLeftLaneIds{4765, 4770, 4775, 4424, 4780, 4785};
constexpr std::array<int64_t, 3> kLeftNonPreferredIds{4424, 4780, 4785};
}  // namespace

// P0 (NG 68aeeb5): 자차가 우선차선이 아닌 차선 열에 있어도 EXTERNAL_REQUEST 는
// target_neighbor 가 비지 않아야 한다 (비면 is_lanes_available()=false → 후보 0).
TEST_F(TestNormalLaneChange, HlfmaP0_ExternalRequestNeighborLanesFromNonPreferredLane)
{
  const auto & rh = *planner_data_->route_handler;
  lanelet::ConstLanelets current_lanes;
  for (const auto id : kLeftNonPreferredIds) {
    current_lanes.push_back(rh.getLaneletsFromId(id));
    ASSERT_NE(rh.getNumLaneToPreferredLane(current_lanes.back()), 0) << "fixture: lane " << id;
  }

  const auto normal = autoware::behavior_path_planner::utils::lane_change::get_target_neighbor_lanes(
    rh, current_lanes, LaneChangeModuleType::NORMAL);
  EXPECT_EQ(normal.size(), current_lanes.size()) << "NORMAL(필수) 은 기존 동작 유지";

  const auto external =
    autoware::behavior_path_planner::utils::lane_change::get_target_neighbor_lanes(
      rh, current_lanes, LaneChangeModuleType::EXTERNAL_REQUEST);
  EXPECT_EQ(external.size(), current_lanes.size())
    << "P0: EXTERNAL_REQUEST 는 비우선차선에서도 현재 차선 열을 target_neighbor 로 써야 함";
}

// P1: 자차가 우선차선(좌회전 차선 등)에 있을 때 EXTERNAL_REQUEST(우측 우회)의
// 최소 차선변경 길이가 DBL_MAX 가 되면 안 된다 (calc_shift_intervals 빈 벡터 → 후보 0).
TEST_F(TestNormalLaneChange, HlfmaP1_ExternalRequestFromPreferredLaneHasFiniteLength)
{
  // 경로: 좌측 차선(y>0)을 전 구간 우선차선으로 → 자차(-50,1.75) 는 우선차선
  std::string autoware_route_handler_dir{"autoware_route_handler"};
  const auto rh_test_route =
    get_absolute_path_to_route(autoware_route_handler_dir, "lane_change_test_route.yaml");
  auto route_opt = autoware::test_utils::parse<std::optional<LaneletRoute>>(rh_test_route);
  ASSERT_TRUE(route_opt.has_value());
  auto route = *route_opt;
  const std::set<int64_t> left_ids(kLeftLaneIds.begin(), kLeftLaneIds.end());
  for (auto & seg : route.segments) {
    for (const auto & p : seg.primitives) {
      if (left_ids.count(p.id)) seg.preferred_primitive = p;
    }
  }
  planner_data_->route_handler->setRoute(route);

  normal_lane_change_ = std::make_shared<NormalLaneChange>(
    lc_param_ptr_, LaneChangeModuleType::EXTERNAL_REQUEST, Direction::RIGHT);
  normal_lane_change_->setData(planner_data_);
  set_previous_approved_path();

  constexpr auto is_approved = true;
  normal_lane_change_->update_lanes(!is_approved);
  const auto & common = get_common_data_ptr();
  ASSERT_TRUE(common->is_lanes_available()) << "fixture: 우선차선에서 external 후보 차선은 있어야 함";
  ASSERT_EQ(
    planner_data_->route_handler->getNumLaneToPreferredLane(common->lanes_ptr->current.back()), 0)
    << "fixture: 현재 차선 열의 끝이 우선차선이어야 P1 조건";

  const auto [lc_length, dist_buffer] =
    autoware::behavior_path_planner::utils::lane_change::calculation::
      calc_lc_length_and_dist_buffer(common, common->lanes_ptr->current);
  EXPECT_LT(lc_length.min, 1.0e6)
    << "P1: 우선차선에서 external_request 최소 차선변경 길이가 무한대 (후보 생성 불가)";
  EXPECT_LT(dist_buffer.min, 1.0e6);
}

// P2: 준비구간 상한(규제요소 근접 금지 거리)은 외부 속도제한이 걸려 있으면
// min(max_vel, 외부 제한) × max_prepare_duration 이어야 한다.
TEST_F(TestNormalLaneChange, HlfmaP2_MaxPrepareLengthRespectsExternalVelocityLimit)
{
  using autoware::behavior_path_planner::utils::lane_change::calculation::
    calc_maximum_prepare_length;
  const auto max_prepare_duration = lc_param_ptr_->trajectory.max_prepare_duration;
  const auto max_vel = planner_data_->parameters.max_vel;

  // 제한 없음: 기존 동작 (회귀)
  EXPECT_NEAR(
    calc_maximum_prepare_length(get_common_data_ptr()), max_prepare_duration * max_vel, 1e-6);

  // 외부 속도제한 4.0 m/s (판단 노드의 hold 접근)
  auto limit = std::make_shared<autoware_internal_planning_msgs::msg::VelocityLimit>();
  limit->max_velocity = 4.0;
  planner_data_->external_limit_max_velocity = limit;
  normal_lane_change_->setData(planner_data_);
  EXPECT_NEAR(calc_maximum_prepare_length(get_common_data_ptr()), max_prepare_duration * 4.0, 1e-6)
    << "P2: 외부 속도제한 시 준비구간 상한이 줄어야 규제요소 근접 금지구역이 실제 속도에 맞음";

  // 외부 제한이 max_vel 보다 크면 max_vel 유지
  limit->max_velocity = max_vel + 10.0;
  planner_data_->external_limit_max_velocity = limit;
  normal_lane_change_->setData(planner_data_);
  EXPECT_NEAR(
    calc_maximum_prepare_length(get_common_data_ptr()), max_prepare_duration * max_vel, 1e-6);
}
