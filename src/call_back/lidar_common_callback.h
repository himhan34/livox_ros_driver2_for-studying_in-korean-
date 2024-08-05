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

#ifndef LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_ // LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_ // LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_를 정의

#include "comm/comm.h" // "comm/comm.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

class LidarCommonCallback { // LidarCommonCallback 클래스 정의
 public:
  // Lidar 포인트 클라우드 콜백 함수 (정적 함수)
  static void OnLidarPointClounCb(PointFrame* frame, void* client_data);
  // Lidar IMU 데이터 콜백 함수 (정적 함수)
  static void LidarImuDataCallback(ImuData* imu_data, void *client_data);
};

} // namespace livox_ros // livox_ros 네임스페이스 끝

#endif // LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_ // LIVOX_ROS_DRIVER_LIDAR_COMMON_CALLBACK_H_ 종료
