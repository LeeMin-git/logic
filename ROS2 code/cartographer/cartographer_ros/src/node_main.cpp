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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");//LM turtelbot3_cartographer/config
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file."); //LM turtlebot3_lds_2d.lua를 가지고옴. 
              //LM 두 가지를 활용하여 turtlebot3_cartographer/config/turtlebot3_lds_2d.lua를 가져옴.
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  rclcpp::Node::SharedPtr cartographer_node = rclcpp::Node::make_shared("cartographer_node"); //LM node 생성 -> Node.__init__('노드이름')
  constexpr double kTfBufferCacheTimeInSeconds = 10.;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node); //*** LM 10초 동안 좌표변환 정보를 유지하는 버퍼 생성, 3번째 인자에 debugging 정보를 노출한다.

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
// *** LM 해당 포인터의 역할이 뭔지 잘 이해가 안됨.

//LM 두 구조체 모두 turtlebo3_lds_2d.lua에서 값을 가져옴
  NodeOptions node_options; //LM 구조체 할당
  TrajectoryOptions trajectory_options;  //LM 구조체 할당 
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
      //LM FLAGS_ 변수에 저장된 .lua 파일을 불러와 구조체에 값을 할당한다.

  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
    //LM 핵심역할,map_Builder.h의 createbuilder 사용. map_builder를 생성함.
  auto node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics); // LM 핵심역할, 인터페이스 생성, 여기서 자동으로 publisher, service_server가 선언됨, map_builder의 소유권 node에 이전
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  } //LM load_state_filename에 해당 하는 argv 값을 전달하지 않기때문에 해당 코드 타지 않음.

  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  } //LM 함수 안에서 SensorBridge가 생성되고, 등록된 센서 토픽(scan,imu,odom 등)을 구독함.

  rclcpp::spin(cartographer_node);

  node->FinishAllTrajectories(); //LM 모든 trajectory를 더 이상 업데이트하지 않도록 종료하는 함수.
  node->RunFinalOptimization(); //LM PoseGraph 전체를 최적화해서 SLAM 결과를 정밀하게 만듦.

  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing(); //LM 파싱을 여러번 할 수 있도록 해주는 함수
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false); //LM 코드 상단의 DEFINE_ 과 함께 사용하며 FLAGS_변수명으로 호출가능한것 같음

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink; //LM glog 형식의 로그를 ROS2 콘솔에서 확인 가능하게 해줌
  cartographer_ros::Run(); //LM 노드에 선언한 run 함수 실행
  ::rclcpp::shutdown(); //LM 전역 네임스페이스에 있는 rclcpp 종료
}
