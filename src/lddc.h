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

#ifndef LIVOX_ROS_DRIVER2_LDDC_H_
#define LIVOX_ROS_DRIVER2_LDDC_H_

#include "include/livox_ros_driver2.h"

#include "driver_node.h"
#include "lds.h"

namespace livox_ros {

/** ROS 구독자에게 포인트클라우드 메시지 데이터를 보내거나 rosbag 파일에 저장합니다 */
typedef enum {
  kOutputToRos = 0, // ROS로 출력
  kOutputToRosBagFile = 1, // rosbag 파일로 출력
} DestinationOfMessageOutput;

/** 전송 메시지의 유형 */
typedef enum {
  kPointCloud2Msg = 0, // PointCloud2 메시지
  kLivoxCustomMsg = 1, // Livox 커스텀 메시지
  kPclPxyziMsg = 2, // PCL PXYZI 메시지
  kLivoxImuMsg = 3, // Livox IMU 메시지
} TransferType;

/** ROS 버전에 따른 타입 정의 */
#ifdef BUILDING_ROS1
using Publisher = ros::Publisher;
using PublisherPtr = ros::Publisher*;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using CustomMsg = livox_ros_driver2::CustomMsg;
using CustomPoint = livox_ros_driver2::CustomPoint;
using ImuMsg = sensor_msgs::Imu;
#elif defined BUILDING_ROS2
template <typename MessageT> using Publisher = rclcpp::Publisher<MessageT>;
using PublisherPtr = std::shared_ptr<rclcpp::PublisherBase>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;
using ImuMsg = sensor_msgs::msg::Imu;
#endif

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class DriverNode;

class Lddc final {
 public:
#ifdef BUILDING_ROS1
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id, bool lidar_bag, bool imu_bag);
#elif defined BUILDING_ROS2
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id);
#endif
  ~Lddc();

  int RegisterLds(Lds *lds); // Lds 등록
  void DistributePointCloudData(void); // 포인트클라우드 데이터 분배
  void DistributeImuData(void); // IMU 데이터 분배
  void CreateBagFile(const std::string &file_name); // Bag 파일 생성
  void PrepareExit(void); // 종료 준비

  uint8_t GetTransferFormat(void) { return transfer_format_; } // 전송 형식 가져오기
  uint8_t IsMultiTopic(void) { return use_multi_topic_; } // 멀티 토픽 여부 확인
  void SetRosNode(livox_ros::DriverNode *node) { cur_node_ = node; } // ROS 노드 설정

  // void SetRosPub(ros::Publisher *pub) { global_pub_ = pub; };  // 사용되지 않음
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; } // 출판 빈도 설정

 public:
  Lds *lds_; // Lds 포인터

 private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar); // LiDAR 포인트클라우드 데이터 폴링
  void PollingLidarImuData(uint8_t index, LidarDevice *lidar); // LiDAR IMU 데이터 폴링

  void PublishPointcloud2(LidarDataQueue *queue, uint8_t index); // PointCloud2 메시지 발행
  void PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index); // 커스텀 포인트클라우드 발행
  void PublishPclMsg(LidarDataQueue *queue, uint8_t index); // PCL 메시지 발행

  void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index); // IMU 데이터 발행

  void InitPointcloud2MsgHeader(PointCloud2& cloud); // PointCloud2 메시지 헤더 초기화
  void InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp); // PointCloud2 메시지 초기화
  void PublishPointcloud2Data(const uint8_t index, uint64_t timestamp, const PointCloud2& cloud); // PointCloud2 데이터 발행

  void InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index); // 커스텀 메시지 초기화
  void FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg); // 커스텀 메시지에 포인트 채우기
  void PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index); // 커스텀 포인트 데이터 발행

  void InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp); // PCL 메시지 초기화
  void FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg); // PCL 메시지에 포인트 채우기
  void PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud); // PCL 데이터 발행

  void InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp); // IMU 메시지 초기화

  void FillPointsToPclMsg(PointCloud& pcl_msg, LivoxPointXyzrtlt* src_point, uint32_t num); // PCL 메시지에 포인트 채우기
  void FillPointsToCustomMsg(CustomMsg& livox_msg, LivoxPointXyzrtlt* src_point, uint32_t num,
      uint32_t offset_time, uint32_t point_interval, uint32_t echo_num); // 커스텀 메시지에 포인트 채우기

#ifdef BUILDING_ROS2
  PublisherPtr CreatePublisher(uint8_t msg_type, std::string &topic_name, uint32_t queue_size); // 퍼블리셔 생성
#endif

  PublisherPtr GetCurrentPublisher(uint8_t index); // 현재 퍼블리셔 가져오기
  PublisherPtr GetCurrentImuPublisher(uint8_t index); // 현재 IMU 퍼블리셔 가져오기

 private:
  uint8_t transfer_format_; // 전송 형식
  uint8_t use_multi_topic_; // 멀티 토픽 사용 여부
  uint8_t data_src_; // 데이터 소스
  uint8_t output_type_; // 출력 유형
  double publish_frq_; // 출판 빈도
  uint32_t publish_period_ns_; // 출판 주기 (나노초)
  std::string frame_id_; // 프레임 ID

#ifdef BUILDING_ROS1
  bool enable_lidar_bag_; // LiDAR bag 사용 여부
  bool enable_imu_bag_; // IMU bag 사용 여부
  PublisherPtr private_pub_[kMaxSourceLidar]; // 개인 퍼블리셔
  PublisherPtr global_pub_; // 전역 퍼블리셔
  PublisherPtr private_imu_pub_[kMaxSourceLidar]; // 개인 IMU 퍼블리셔
  PublisherPtr global_imu_pub_; // 전역 IMU 퍼블리셔
  rosbag::Bag *bag_; // rosbag 포인터
#elif defined BUILDING_ROS2
  PublisherPtr private_pub_[kMaxSourceLidar]; // 개인 퍼블리셔
  PublisherPtr global_pub_; // 전역 퍼블리셔
  PublisherPtr private_imu_pub_[kMaxSourceLidar]; // 개인 IMU 퍼블리셔
  PublisherPtr global_imu_pub_; // 전역 IMU 퍼블리셔
#endif

  livox_ros::DriverNode *cur_node_; // 현재 노드
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER2_LDDC_H_
