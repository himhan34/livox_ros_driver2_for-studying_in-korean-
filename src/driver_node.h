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

#ifndef LIVOX_DRIVER_NODE_H // LIVOX_DRIVER_NODE_H가 정의되지 않았을 때
#define LIVOX_DRIVER_NODE_H // LIVOX_DRIVER_NODE_H를 정의

#include "include/ros_headers.h" // "ros_headers.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

class Lddc; // Lddc 클래스 선언

#ifdef BUILDING_ROS1 // ROS1을 빌드할 때
class DriverNode final : public ros::NodeHandle { // DriverNode 클래스 정의, ros::NodeHandle 상속
 public:
  DriverNode() = default; // 기본 생성자
  DriverNode(const DriverNode &) = delete; // 복사 생성자 삭제
  ~DriverNode(); // 소멸자
  DriverNode &operator=(const DriverNode &) = delete; // 복사 할당 연산자 삭제

  DriverNode& GetNode() noexcept; // 노드 객체 반환 함수

  void PointCloudDataPollThread(); // 포인트 클라우드 데이터 폴링 스레드 함수
  void ImuDataPollThread(); // IMU 데이터 폴링 스레드 함수

  std::unique_ptr<Lddc> lddc_ptr_; // Lddc 포인터
  std::shared_ptr<std::thread> pointclouddata_poll_thread_; // 포인트 클라우드 데이터 폴링 스레드
  std::shared_ptr<std::thread> imudata_poll_thread_; // IMU 데이터 폴링 스레드
  std::shared_future<void> future_; // 종료 신호를 위한 미래 객체
  std::promise<void> exit_signal_; // 종료 신호를 위한 약속 객체
};

#elif defined BUILDING_ROS2 // ROS2를 빌드할 때
class DriverNode final : public rclcpp::Node { // DriverNode 클래스 정의, rclcpp::Node 상속
 public:
  explicit DriverNode(const rclcpp::NodeOptions& options); // 생성자
  DriverNode(const DriverNode &) = delete; // 복사 생성자 삭제
  ~DriverNode(); // 소멸자
  DriverNode &operator=(const DriverNode &) = delete; // 복사 할당 연산자 삭제

  DriverNode& GetNode() noexcept; // 노드 객체 반환 함수

 private:
  void PointCloudDataPollThread(); // 포인트 클라우드 데이터 폴링 스레드 함수
  void ImuDataPollThread(); // IMU 데이터 폴링 스레드 함수

  std::unique_ptr<Lddc> lddc_ptr_; // Lddc 포인터
  std::shared_ptr<std::thread> pointclouddata_poll_thread_; // 포인트 클라우드 데이터 폴링 스레드
  std::shared_ptr<std::thread> imudata_poll_thread_; // IMU 데이터 폴링 스레드
  std::shared_future<void> future_; // 종료 신호를 위한 미래 객체
  std::promise<void> exit_signal_; // 종료 신호를 위한 약속 객체
};
#endif

} // namespace livox_ros // livox_ros 네임스페이스 끝

#endif // LIVOX_DRIVER_NODE_H // LIVOX_DRIVER_NODE_H 종료
