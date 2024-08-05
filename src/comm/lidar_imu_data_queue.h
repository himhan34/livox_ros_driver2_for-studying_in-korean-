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


#ifndef LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_ // LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_ // LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_를 정의

#include <list> // 리스트 사용을 위한 헤더 파일 포함
#include <mutex> // 뮤텍스 사용을 위한 헤더 파일 포함
#include <cstdint> // 정수형 데이터 타입 사용을 위한 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// Livox 통신 프로토콜의 IMU 데이터 타입 기반
// TODO: 프로토콜에 대한 링크 추가
typedef struct {
  float gyro_x;        /**< 자이로스코프 X 축, 단위: rad/s */
  float gyro_y;        /**< 자이로스코프 Y 축, 단위: rad/s */
  float gyro_z;        /**< 자이로스코프 Z 축, 단위: rad/s */
  float acc_x;         /**< 가속도계 X 축, 단위: g */
  float acc_y;         /**< 가속도계 Y 축, 단위: g */
  float acc_z;         /**< 가속도계 Z 축, 단위: g */
} RawImuPoint;

// IMU 데이터 구조체
typedef struct {
  uint8_t lidar_type;  // Lidar 타입
  uint32_t handle;     // 핸들
  uint8_t slot;        // 슬롯
  uint64_t time_stamp; // 타임스탬프
  float gyro_x;        /**< 자이로스코프 X 축, 단위: rad/s */
  float gyro_y;        /**< 자이로스코프 Y 축, 단위: rad/s */
  float gyro_z;        /**< 자이로스코프 Z 축, 단위: rad/s */
  float acc_x;         /**< 가속도계 X 축, 단위: g */
  float acc_y;         /**< 가속도계 Y 축, 단위: g */
  float acc_z;         /**< 가속도계 Z 축, 단위: g */
} ImuData;

// Lidar IMU 데이터 큐 클래스
class LidarImuDataQueue {
 public:
  void Push(ImuData* imu_data); // IMU 데이터를 큐에 푸시하는 함수
  bool Pop(ImuData& imu_data); // IMU 데이터를 큐에서 팝하는 함수
  bool Empty(); // 큐가 비어있는지 확인하는 함수
  void Clear(); // 큐를 비우는 함수

 private:
  std::mutex mutex_; // 뮤텍스
  std::list<ImuData> imu_data_queue_; // IMU 데이터 큐 리스트
};

} // namespace livox_ros // livox_ros 네임스페이스 끝

#endif // LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_ // LIVOX_ROS_DRIVER_LIDAR_IMU_DATA_QUEUE_H_ 종료
