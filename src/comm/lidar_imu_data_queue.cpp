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
#include "lidar_imu_data_queue.h" // "lidar_imu_data_queue.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// IMU 데이터를 큐에 푸시하는 함수
void LidarImuDataQueue::Push(ImuData* imu_data) {
  ImuData data; // 임시 ImuData 객체 생성
  data.lidar_type = imu_data->lidar_type; // lidar_type 설정
  data.handle = imu_data->handle; // handle 설정
  data.time_stamp = imu_data->time_stamp; // time_stamp 설정

  data.gyro_x = imu_data->gyro_x; // gyro_x 설정
  data.gyro_y = imu_data->gyro_y; // gyro_y 설정
  data.gyro_z = imu_data->gyro_z; // gyro_z 설정

  data.acc_x = imu_data->acc_x; // acc_x 설정
  data.acc_y = imu_data->acc_y; // acc_y 설정
  data.acc_z = imu_data->acc_z; // acc_z 설정

  std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
  imu_data_queue_.push_back(std::move(data)); // 데이터 큐에 추가
}

// IMU 데이터를 큐에서 팝하는 함수
bool LidarImuDataQueue::Pop(ImuData& imu_data) {
  std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
  if (imu_data_queue_.empty()) { // 큐가 비어있는지 확인
    return false; // 실패
  }
  imu_data = imu_data_queue_.front(); // 큐의 첫 번째 데이터를 imu_data에 복사
  imu_data_queue_.pop_front(); // 첫 번째 데이터 제거
  return true; // 성공
}

// 큐가 비어있는지 확인하는 함수
bool LidarImuDataQueue::Empty() {
  std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
  return imu_data_queue_.empty(); // 큐가 비어있는지 여부 반환
}

// 큐를 비우는 함수
void LidarImuDataQueue::Clear() {
  std::list<ImuData> tmp_imu_data_queue; // 임시 큐 생성
  {
    std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
    imu_data_queue_.swap(tmp_imu_data_queue); // 기존 큐와 임시 큐를 교체
  }
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
