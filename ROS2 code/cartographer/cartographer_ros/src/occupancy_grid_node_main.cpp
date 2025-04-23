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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "gflags/gflags.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node : public rclcpp::Node
{
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::msg::SubmapList::ConstSharedPtr& msg);
  void DrawAndPublish();

  const double resolution_;

  absl::Mutex mutex_; //스레드 동기화 도구, 뮤텍스 생성 -> 버퍼 접근을 통제하는 역할 
  rclcpp::CallbackGroup::SharedPtr callback_group_; //callback 그룹을 가리키는 포인터 생성
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_; //싱글스레드실행를 가리키는 포인터 생성
  ::rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client_ GUARDED_BY(mutex_); // 전역 클라이언트를 가리키는 포인터 생성, client_는 mutex 없이 사용하면 안된다는 것을 명시
  ::rclcpp::Subscription<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_subscriber_ GUARDED_BY(mutex_); // 서브스크립션을 가리키는 포인터 생성, 해당 포인터는 mutex 없이 사용하면 안된다는 것을 명시
  ::rclcpp::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_ GUARDED_BY(mutex_); // 퍼블리셔를 가리키는 포인터 생성, 해당 포인터는 mutex 없이 사용하면 안된다.
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  rclcpp::TimerBase::SharedPtr occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  rclcpp::Time last_timestamp_;
};

Node::Node(const double resolution, const double publish_period_sec)
    : rclcpp::Node("cartographer_occupancy_grid_node"), 
      resolution_(resolution) //LM rclcpp::Node 생성자를 Node::Node에 상속, resolution_ 멤버 변수 resolution 값으로 초기화 
{
  callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  callback_group_executor_->add_callback_group(callback_group_, this->get_node_base_interface());
  client_ = this->create_client<cartographer_ros_msgs::srv::SubmapQuery>(
        kSubmapQueryServiceName,
#if RCLCPP_VERSION_GTE(17, 0, 0)
        rclcpp::ServicesQoS(),
#else
        rmw_qos_profile_services_default,
#endif
        callback_group_
        ); //LM submap_query의 메세지 명으로 client 선언 -> submap의 텍스처 데이터를 요청하기 위한 클라이언트임.

  occupancy_grid_publisher_ = this->create_publisher<::nav_msgs::msg::OccupancyGrid>(
      kOccupancyGridTopic, rclcpp::QoS(10).transient_local()); //LM map의 메세지 명으로 publisher 생성

  occupancy_grid_publisher_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(int(publish_period_sec * 1000)),
    [this]() {
      DrawAndPublish();
    }); //LM timer 함수를 통해 주어진 주기 마다 람다 함수 실행 (이때 람다 함수는 DrawAndPublish 임.)

  auto handleSubmapList =
    [this, publish_period_sec](const typename cartographer_ros_msgs::msg::SubmapList::SharedPtr msg) -> void 
    {
    absl::MutexLock locker(&mutex_); //LM mutex 잠금 {} 부분을 벗어나면 자동으로 unlock 상태가 된다.

    // We do not do any work if nobody listens.
    if (this->count_publishers(kSubmapListTopic) == 0){
      return;
    } //LM 'summap_list'의 토픽을 구독하는 publisher의 개수

    // Keep track of submap IDs that don't appear in the message anymore.
    std::set<SubmapId> submap_ids_to_delete;
    for (const auto& pair : submap_slices_) {
      submap_ids_to_delete.insert(pair.first); //LM (SubmapId, SubmapSlice)를 갖는 변수에서 submap_ids_to_delete에 pair.first의 키 값(submapId)을 넣는다.
    } //LM 우선은 지워질 후보에 추가함. 추후에 현재 msg와 비교하여 실제 존재하는 곳이면 지워질 후보에서 제외시킴.
    
    //LM submap_list 메세지 명으로 받아오는 값 = msg
    for (const auto& submap_msg : msg->submap) {
      const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index}; //LM 구조체를 구성하는 코드
      submap_ids_to_delete.erase(id); //LM 현재 위치한 곳이기 때문에 해당 id를 삭제하면 안되므로
      if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
          (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
        continue;
      }
      SubmapSlice& submap_slice = submap_slices_[id]; //LM submap_slice_[id]에 submap_slice라는 이름 부여
      submap_slice.pose = ToRigid3d(submap_msg.pose); //LM 메세지를 통해 얻어 온 pose 값 저장
      submap_slice.metadata_version = submap_msg.submap_version; //LM 메세지에 표시되는 submap_version 저장
      if (submap_slice.surface != nullptr &&
          submap_slice.version == submap_msg.submap_version) {
        continue;
      }

      auto fetched_textures = cartographer_ros::FetchSubmapTextures(
            id, client_, callback_group_executor_,
            std::chrono::milliseconds(int(publish_period_sec * 1000))); //LM client_를 통해 submap의 이미지 형태의 맵 데이터(texture)를 요청하고 받은 reponse 값을 통해 실제 픽셀읠 intencity와 넓이 높이 해상도 데이터를 가져옴
      if (fetched_textures == nullptr) {
        continue;
      }
      CHECK(!fetched_textures->textures.empty());
      submap_slice.version = fetched_textures->version;

      // We use the first texture only. By convention this is the highest
      // resolution texture and that is the one we want to use to construct the
      // map for ROS.
      const auto fetched_texture = fetched_textures->textures.begin();
      submap_slice.width = fetched_texture->width;
      submap_slice.height = fetched_texture->height;
      submap_slice.slice_pose = fetched_texture->slice_pose;
      submap_slice.resolution = fetched_texture->resolution;
      submap_slice.cairo_data.clear(); //LM 이전에 저장된 이미지 데이터 초기화
      submap_slice.surface = ::cartographer::io::DrawTexture(
          fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
          fetched_texture->width, fetched_texture->height,
          &submap_slice.cairo_data); //LM 해당 부분이 2D 이미지를 생성하는 부분임. -> submap_slice.surface를 주석 처리 하니 맵을 못그림.
    }

    // Delete all submaps that didn't appear in the message.
    for (const auto& id : submap_ids_to_delete) {
      submap_slices_.erase(id);
    }//LM(생각) submap_ids_to_delete에 현재는 안쓰는 애들이 저장되어 있음.

    last_timestamp_ = msg->header.stamp;
    last_frame_id_ = msg->header.frame_id;
    };

  submap_list_subscriber_ = create_subscription<cartographer_ros_msgs::msg::SubmapList>(
    kSubmapListTopic, rclcpp::QoS(10), handleSubmapList); //LM submap_list라는 메세지 명으로 subscription 생성 호출 시 handleSubmapList 함수 실행
}

void Node::DrawAndPublish() {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_); //LM 여러개의 submap들을 겹쳐 하나의 map으로 만들어 주는 함수
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_); //LM 2D map 이미지를 ROS2 occupancyGrid 메세지로 변환
  occupancy_grid_publisher_->publish(*msg_ptr); //LM 변환한 메세지 일정 주기마다 pub
}//LM 여러 서브맵들을 하나의 큰 맵으로 합쳐서 ROS 표준 메시지 형태로 만들어주는 것.

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto node = std::make_shared<cartographer_ros::Node>(FLAGS_resolution, FLAGS_publish_period_sec);

  rclcpp::spin(node);
  ::rclcpp::shutdown();
  return 0;
}
