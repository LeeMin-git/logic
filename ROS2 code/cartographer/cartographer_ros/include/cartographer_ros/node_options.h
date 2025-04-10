/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer_ros/trajectory_options.h"

namespace cartographer_ros {

// Top-level options of Cartographer's ROS integration.
struct NodeOptions {
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options; //LM mapbuilder의 옵션, .lua 설정 파일에서 설정됨.
  std::string map_frame;
  double lookup_transform_timeout_sec; //LM TF에서 좌표 변환 정보를 가져올 때 얼마나 오래 기다릴 것인지, 설정 시간 지나면 타임아웃 발생
  double submap_publish_period_sec; //LM 부분 지도를 얼마나 자주 pub 할것인지 
  double pose_publish_period_sec; //LM 현재 추정 위치를 얼마나 자주 퍼블리시할지 설정
  double trajectory_publish_period_sec; // 로봇이 지나온 경로를 얼마나 자주 pub할것인지
  bool publish_to_tf = true; //LM SLAM으로 계산한 위치 정보를 TF로의 pub 여부, turtlebot3_lds_2d.lua에 해당 값 없음
  bool publish_tracked_pose = false; //LM SLAM이 계산한 로봇의 추적 위치의 pub 여부, turtlebot3_lds_2d.lua에 해당 값 없음
  bool use_pose_extrapolator = true; //LM 센서 간 시간 차이를 보정하고, 더 부드럽고 정활한 포즈 추정을 위해 extrapolatior의 사용 여부, turtlebot3_lds_2d.lua에 해당 값 없음
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_OPTIONS_H
