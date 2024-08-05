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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <chrono>
#include <algorithm>

#include "lds.h"
#include "comm/ldq.h"

namespace livox_ros {

// CacheIndex 클래스의 정적 멤버 초기화
CacheIndex Lds::cache_index_;

/* 멤버 함수 --------------------------------------------------------- */
// Lds 클래스의 생성자
Lds::Lds(const double publish_freq, const uint8_t data_src)
    : lidar_count_(kMaxSourceLidar),       // 라이다의 최대 개수로 초기화
      pcd_semaphore_(0),                   // PCD 세마포어 초기화
      imu_semaphore_(0),                   // IMU 세마포어 초기화
      publish_freq_(publish_freq),         // 주기 설정
      data_src_(data_src),                 // 데이터 소스 설정
      request_exit_(false) {               // 종료 요청 상태 초기화
  ResetLds(data_src_);                     // Lds 초기화 함수 호출
}

// Lds 클래스의 소멸자
Lds::~Lds() {
  lidar_count_ = 0;                       // 라이다 개수 0으로 설정
  ResetLds(0);                            // Lds 초기화 함수 호출
  printf("lds destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

// 라이다 장치를 초기화하는 함수
void Lds::ResetLidar(LidarDevice *lidar, uint8_t data_src) {
  // cache_index_.ResetIndex(lidar);  // 캐시 인덱스 초기화 (주석 처리됨)
  DeInitQueue(&lidar->data);          // 데이터 큐 초기화 해제
  lidar->imu_data.Clear();            // IMU 데이터 초기화

  lidar->data_src = data_src;         // 데이터 소스 설정
  lidar->connect_state = kConnectStateOff;  // 연결 상태를 꺼짐으로 설정
}

// 라이다 장치의 데이터 소스를 설정하는 함수
void Lds::SetLidarDataSrc(LidarDevice *lidar, uint8_t data_src) {
  lidar->data_src = data_src;         // 데이터 소스 설정
}

// Lds를 초기화하는 함수
void Lds::ResetLds(uint8_t data_src) {
  lidar_count_ = kMaxSourceLidar;     // 라이다 개수를 최대 값으로 설정
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    ResetLidar(&lidars_[i], data_src);  // 각 라이다를 초기화
  }
}

// 종료 요청을 설정하는 함수
void Lds::RequestExit() {
  request_exit_ = true;               // 종료 요청 상태를 true로 설정
}

// 모든 큐가 비어있는지 확인하는 함수
bool Lds::IsAllQueueEmpty() {
  for (int i = 0; i < lidar_count_; i++) {
    if (!QueueIsEmpty(&lidars_[i].data)) {  // 하나라도 비어있지 않으면 false 반환
      return false;
    }
  }
  return true;                        // 모두 비어있으면 true 반환
}

// 모든 큐의 읽기가 중지되었는지 확인하는 함수
bool Lds::IsAllQueueReadStop() {
  for (int i = 0; i < lidar_count_; i++) {
    uint32_t data_size = QueueUsedSize(&lidars_[i].data);  // 큐에 사용된 크기 확인
    if (data_size) {
      return false;                   // 데이터가 남아있으면 false 반환
    }
  }
  return true;                        // 모두 중지되었으면 true 반환
}

// IMU 데이터를 저장하는 함수
void Lds::StorageImuData(ImuData* imu_data) {
  uint32_t device_num = 0;
  if (imu_data->lidar_type == kLivoxLidarType) {
    device_num = imu_data->handle;    // 라이다 핸들 설정
  } else {
    printf("Storage imu data failed, unknown lidar type:%u.\n", imu_data->lidar_type);
    return;
  }

  uint8_t index = 0;
  int ret = cache_index_.GetIndex(imu_data->lidar_type, device_num, index);
  if (ret != 0) {
    printf("Storage point data failed, can not get index, lidar type:%u, device_num:%u.\n", imu_data->lidar_type, device_num);
    return;
  }

  LidarDevice *p_lidar = &lidars_[index];
  LidarImuDataQueue* imu_queue = &p_lidar->imu_data;
  imu_queue->Push(imu_data);          // IMU 데이터를 큐에 추가
  if (!imu_queue->Empty()) {          // 큐가 비어있지 않으면
    if (imu_semaphore_.GetCount() <= 0) {
      imu_semaphore_.Signal();        // 세마포어 신호 발생
    }
  }
}

// LVX 포인트 데이터를 저장하는 함수
void Lds::StorageLvxPointData(PointFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  uint8_t lidar_number = frame->lidar_num;
  for (uint i = 0; i < lidar_number; ++i) {
    PointPacket& lidar_point = frame->lidar_point[i];

    uint64_t base_time = frame->base_time[i];
    uint8_t index = 0;
    int8_t ret = cache_index_.LvxGetIndex(lidar_point.lidar_type, lidar_point.handle, index);
    if (ret != 0) {
      printf("Storage lvx point data failed, lidar type:%u, device num:%u.\n", lidar_point.lidar_type, lidar_point.handle);
      continue;
    }

    lidars_[index].connect_state = kConnectStateSampling;  // 연결 상태를 샘플링으로 설정

    PushLidarData(&lidar_point, index, base_time);  // 라이다 데이터를 큐에 추가
  }
}

// 포인트 데이터를 저장하는 함수
void Lds::StoragePointData(PointFrame* frame) {
  if (frame == nullptr) {
    return;
  }

  uint8_t lidar_number = frame->lidar_num;
  for (uint i = 0; i < lidar_number; ++i) {
    PointPacket& lidar_point = frame->lidar_point[i];
    //printf("StoragePointData, lidar_type:%u, point_num:%lu.\n", lidar_point.lidar_type, lidar_point.points_num);

    uint64_t base_time = frame->base_time[i];

    uint8_t index = 0;
    int8_t ret = cache_index_.GetIndex(lidar_point.lidar_type, lidar_point.handle, index);
    if (ret != 0) {
      printf("Storage point data failed, lidar type:%u, handle:%u.\n", lidar_point.lidar_type, lidar_point.handle);
      continue;
    }
    PushLidarData(&lidar_point, index, base_time);  // 라이다 데이터를 큐에 추가
  }
}

// 라이다 데이터를 큐에 추가하는 함수
void Lds::PushLidarData(PointPacket* lidar_data, const uint8_t index, const uint64_t base_time) {
  if (lidar_data == nullptr) {
    return;
  }

  LidarDevice *p_lidar = &lidars_[index];
  LidarDataQueue *queue = &p_lidar->data;

  if (nullptr == queue->storage_packet) {
    uint32_t queue_size = CalculatePacketQueueSize(publish_freq_);
    InitQueue(queue, queue_size);     // 큐 초기화
    printf("Lidar[%u] storage queue size: %u\n", index, queue_size);
  }

  if (!QueueIsFull(queue)) {          // 큐가 가득 차지 않았으면
    QueuePushAny(queue, (uint8_t *)lidar_data, base_time);  // 큐에 데이터를 추가
    if (!QueueIsEmpty(queue)) {       // 큐가 비어있지 않으면
      if (pcd_semaphore_.GetCount() <= 0) {
        pcd_semaphore_.Signal();      // 세마포어 신호 발생
      }
    }
  } else {
    if (pcd_semaphore_.GetCount() <= 0) {
        pcd_semaphore_.Signal();      // 큐가 가득 차도 세마포어 신호 발생
    }
  }
}

// 종료를 준비하는 함수
void Lds::PrepareExit(void) {}

}  // namespace livox_ros
