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

#ifndef LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_ // LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_ // LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_를 정의

#include "../lds.h" // 상위 디렉토리의 "lds.h" 헤더 파일 포함
#include "../lds_lidar.h" // 상위 디렉토리의 "lds_lidar.h" 헤더 파일 포함
#include "../comm/comm.h" // 상위 디렉토리의 "comm/comm.h" 헤더 파일 포함

#include "livox_lidar_api.h" // "livox_lidar_api.h" 헤더 파일 포함
#include "livox_lidar_def.h" // "livox_lidar_def.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

class LivoxLidarCallback { // LivoxLidarCallback 클래스 정의
 public:
  // Lidar 정보 변경 콜백 함수 (정적 함수)
  static void LidarInfoChangeCallback(const uint32_t handle,
                                      const LivoxLidarInfo* info,
                                      void* client_data);
  // 작업 모드 변경 콜백 함수 (정적 함수)
  static void WorkModeChangedCallback(livox_status status,
                                      uint32_t handle,
                                      LivoxLidarAsyncControlResponse *response,
                                      void *client_data);
  // 데이터 타입 설정 콜백 함수 (정적 함수)
  static void SetDataTypeCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  // 패턴 모드 설정 콜백 함수 (정적 함수)
  static void SetPatternModeCallback(livox_status status, uint32_t handle,
                                     LivoxLidarAsyncControlResponse *response,
                                     void *client_data);
  // 블라인드 스팟 설정 콜백 함수 (정적 함수)
  static void SetBlindSpotCallback(livox_status status, uint32_t handle,
                                   LivoxLidarAsyncControlResponse *response,
                                   void *client_data);
  // 듀얼 방출 설정 콜백 함수 (정적 함수)
  static void SetDualEmitCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  // 태도 설정 콜백 함수 (정적 함수)
  static void SetAttitudeCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);
  // Livox Lidar IMU 데이터 활성화 콜백 함수 (정적 함수)
  static void EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                  LivoxLidarAsyncControlResponse *response,
                                  void *client_data);

 private:
  // Lidar 장치 가져오기 함수 (정적 함수)
  static LidarDevice* GetLidarDevice(const uint32_t handle, void* client_data);
};

} // namespace livox_ros // livox_ros 네임스페이스 끝

#endif  // LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_ // LIVOX_ROS_DRIVER_LIVOX_LIDAR_CALLBACK_H_ 종료
