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

// Livox Lidar 데이터 소스

#ifndef LIVOX_ROS_DRIVER_LDS_H_
#define LIVOX_ROS_DRIVER_LDS_H_

#include <map>

#include "comm/semaphore.h"
#include "comm/comm.h"
#include "comm/cache_index.h"

namespace livox_ros {
/**
 * Lidar 데이터 소스 추상화.
 */
class Lds {
 public:
  // 생성자
  Lds(const double publish_freq, const uint8_t data_src);
  // 소멸자
  virtual ~Lds();

  // IMU 데이터를 저장하는 함수
  void StorageImuData(ImuData* imu_data);
  // 포인트 데이터를 저장하는 함수
  void StoragePointData(PointFrame* frame);
  // LVX 포인트 데이터를 저장하는 함수
  void StorageLvxPointData(PointFrame* frame);

  // 라이다 핸들을 가져오는 함수
  int8_t GetHandle(const uint8_t lidar_type, const PointPacket* lidar_point);
  // 라이다 데이터를 큐에 추가하는 함수
  void PushLidarData(PointPacket* lidar_data, const uint8_t index, const uint64_t base_time);

  // 라이다 장치를 초기화하는 정적 함수
  static void ResetLidar(LidarDevice *lidar, uint8_t data_src);
  // 라이다 장치의 데이터 소스를 설정하는 정적 함수
  static void SetLidarDataSrc(LidarDevice *lidar, uint8_t data_src);
  // Lds를 초기화하는 함수
  void ResetLds(uint8_t data_src);

  // 종료 요청을 설정하는 함수
  void RequestExit();

  // 모든 큐가 비어있는지 확인하는 함수
  bool IsAllQueueEmpty();
  // 모든 큐의 읽기가 중지되었는지 확인하는 함수
  bool IsAllQueueReadStop();

  // 종료 요청을 초기화하는 함수
  void CleanRequestExit() { request_exit_ = false; }
  // 종료 요청 상태를 확인하는 함수
  bool IsRequestExit() { return request_exit_; }
  // 종료를 준비하는 가상 함수
  virtual void PrepareExit(void);

  // 출판 주기를 가져오는 함수
  double GetLdsFrequency() { return publish_freq_; }

 public:
  uint8_t lidar_count_;                 /**< 라이다 접근 핸들 */
  LidarDevice lidars_[kMaxSourceLidar]; /**< 인덱스는 핸들입니다 */
  Semaphore pcd_semaphore_;
  Semaphore imu_semaphore_;
  static CacheIndex cache_index_;
 protected:
  double publish_freq_;
  uint8_t data_src_;
 private:
  volatile bool request_exit_;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_H_
