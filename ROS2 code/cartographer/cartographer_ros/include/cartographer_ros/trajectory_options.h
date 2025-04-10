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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer_ros {

struct TrajectoryOptions {
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options; //LM SLAM 내부 알고리즘을 위한 세부 설정, .lua 설정 파일로부터 파싱된 내용, 2D/3D모드도 설정할 수 있다고함.
  std::string tracking_frame; //LM SLAM 계산 시 기준이 되는 센서 프레임 (imu_link)
  std::string published_frame; //LM 로봇의 위치를 pub할 때 사용할 프레임 (base_link)
  std::string odom_frame; //LM 외부 오도메트리 정보의 기준 프레임 (odom)
  bool provide_odom_frame; //LM cartographer가 map->odom 변환을 pub할지 여부
  bool use_odometry; //LM 외부 odom을 SLAM에 사용 할지 여부
  bool use_nav_sat; //LM GPS 등의 위성 데이터를 사용할지 여부
  bool use_landmarks; //LM 시각/레이버 렌드 마크 를 사용할지 여부
  bool publish_frame_projected_to_2d; //LM 로봇 위치를 퍼블리시할 때 z출 제거하고 2D 평면으로 투영할지
  int num_laser_scans; //LM 사용하려는 일반 라이다 센서 수 (velodyne 3D LiDAR 사용시 num_laser_scans값 1)
  int num_multi_echo_laser_scans; //LM 멀티 에코 라이다 수
  int num_subdivisions_per_laser_scan; //LM 하나의 레이저 스캔을 몇 개로 나눠 SLAM에 사용할지
  int num_point_clouds; //LM 포인트 클라우드 센서 수 (velodyne 3D LiDAR 사용시 num_point_clouds값 1)
  double rangefinder_sampling_ratio; //LM 라이다 데이터를 몇 % 사용할 것인지 (0.0~0.1)
  double odometry_sampling_ratio;//LM odomatry 데이터 샘플링 비율
  double fixed_frame_pose_sampling_ratio; //LM GPS 등 fixed frame 데이터 샘플링 비율
  double imu_sampling_ratio; //LM IMU 데이터 샘플링 비율
  double landmarks_sampling_ratio;//LM 랜드마크 데이터 샘플링 비율
}; //LM turtlebot3_lds_2d.lua에 모든 값이 들어있음

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
