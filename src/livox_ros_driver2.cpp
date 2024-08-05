//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"  // Livox ROS 드라이버 헤더 파일 포함
#include "include/ros_headers.h"  // ROS 헤더 파일 포함
#include "driver_node.h"  // 드라이버 노드 헤더 파일 포함
#include "lddc.h"  // Lddc 헤더 파일 포함
#include "lds_lidar.h"  // LdsLidar 헤더 파일 포함

using namespace livox_ros;  // livox_ros 네임스페이스 사용

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** ROS 관련 초기화 */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();  // ROS 콘솔 로그 레벨 설정
  }

  ros::init(argc, argv, "livox_lidar_publisher");  // ROS 초기화

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;  // Livox ROS 드라이버 노드 생성

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);  // 드라이버 버전 정보 출력

  /** 기본 시스템 매개변수 초기화 */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);  // 매개변수 가져오기
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);  // 데이터 소스 출력

  if (publish_freq > 100.0) {
    publish_freq = 100.0;  // 최대 주파수 제한
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;  // 최소 주파수 제한
  } else {
    publish_freq = publish_freq;  // 설정된 주파수 사용
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();  // 종료 신호 설정

  /** 라이다 데이터 분배 제어 및 라이다 데이터 소스 설정 */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);  // Lddc 인스턴스 생성
  livox_node.lddc_ptr_->SetRosNode(&livox_node);  // ROS 노드 설정

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");  // 데이터 소스 정보 출력

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);  // 사용자 설정 파일 경로 가져오기
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);  // LdsLidar 인스턴스 가져오기
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));  // Lds 등록

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");  // 초기화 성공 메시지 출력
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");  // 초기화 실패 메시지 출력
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);  // 유효하지 않은 데이터 소스 에러 메시지 출력
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);  // 포인트 클라우드 데이터 폴링 스레드 생성
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);  // IMU 데이터 폴링 스레드 생성
  while (ros::ok()) { usleep(10000); }  // ROS가 실행 중인 동안 대기

  return 0;  // 프로그램 종료
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** 기본 시스템 매개변수 초기화 */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** 라이다 데이터 분배 제어 및 라이다 데이터 소스 설정 */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()  // 포인트 클라우드 데이터 폴링 스레드
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));  // 3초 대기
  do {
    lddc_ptr_->DistributePointCloudData();  // 포인트 클라우드 데이터 분배
    status = future_.wait_for(std::chrono::microseconds(0));  // 종료 신호 확인
  } while (status == std::future_status::timeout);  // 타임아웃 상태일 경우 계속 실행
}

void DriverNode::ImuDataPollThread()  // IMU 데이터 폴링 스레드
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));  // 3초 대기
  do {
    lddc_ptr_->DistributeImuData();  // IMU 데이터 분배
    status = future_.wait_for(std::chrono::microseconds(0));  // 종료 신호 확인
  } while (status == std::future_status::timeout);  // 타임아웃 상태일 경우 계속 실행
}












