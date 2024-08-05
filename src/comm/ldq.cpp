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

#include <stdio.h> // 표준 입출력 함수 사용을 위한 헤더 파일 포함
#include <string.h> // 문자열 처리 함수 사용을 위한 헤더 파일 포함

#include "ldq.h" // "ldq.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

/* 포인트 클라우드 큐 처리 함수 */
// 큐 초기화 함수
bool InitQueue(LidarDataQueue *queue, uint32_t queue_size) {
  if (queue == nullptr) { // 큐가 nullptr인지 확인
    // ROS_WARN("RosDriver Queue: Initialization failed - invalid queue."); // 경고 메시지 (주석 처리됨)
    return false; // 초기화 실패
  }

  if (!IsPowerOf2(queue_size)) { // 큐 크기가 2의 제곱인지 확인
    queue_size = RoundupPowerOf2(queue_size); // 큐 크기를 2의 제곱으로 반올림
    printf("Init queue, real query size:%u.\n", queue_size); // 실제 큐 크기 출력
  }

  if (queue->storage_packet) { // 기존 저장 패킷이 있는지 확인
    delete[] queue->storage_packet; // 기존 저장 패킷 삭제
    queue->storage_packet = nullptr; // 포인터 초기화
  }

  queue->storage_packet = new StoragePacket[queue_size]; // 새로운 저장 패킷 배열 할당
  if (queue->storage_packet == nullptr) { // 메모리 할당 실패 여부 확인
    // ROS_WARN("RosDriver Queue: Initialization failed - failed to allocate memory."); // 경고 메시지 (주석 처리됨)
    return false; // 초기화 실패
  }

  queue->rd_idx = 0; // 읽기 인덱스 초기화
  queue->wr_idx = 0; // 쓰기 인덱스 초기화
  queue->size = queue_size; // 큐 크기 설정
  queue->mask = queue_size - 1; // 마스크 설정

  return true; // 초기화 성공
}

// 큐 해제 함수
bool DeInitQueue(LidarDataQueue *queue) {
  if (queue == nullptr) { // 큐가 nullptr인지 확인
    // ROS_WARN("RosDriver Queue: Deinitialization failed - invalid queue."); // 경고 메시지 (주석 처리됨)
    return false; // 해제 실패
  }

  if (queue->storage_packet) { // 저장 패킷이 있는지 확인
    delete[] queue->storage_packet; // 저장 패킷 삭제
  }

  queue->rd_idx = 0; // 읽기 인덱스 초기화
  queue->wr_idx = 0; // 쓰기 인덱스 초기화
  queue->size = 0; // 큐 크기 초기화
  queue->mask = 0; // 마스크 초기화

  return true; // 해제 성공
}

// 큐 리셋 함수
void ResetQueue(LidarDataQueue *queue) {
  queue->rd_idx = 0; // 읽기 인덱스 초기화
  queue->wr_idx = 0; // 쓰기 인덱스 초기화
}

// 큐에서 팝할 데이터 미리 가져오기
bool QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
  if (queue == nullptr || storage_packet == nullptr) { // 큐나 저장 패킷이 nullptr인지 확인
    // ROS_WARN("RosDriver Queue: Invalid pointer parameters."); // 경고 메시지 (주석 처리됨)
    return false; // 실패
  }

  if (QueueIsEmpty(queue)) { // 큐가 비어있는지 확인
    // ROS_WARN("RosDriver Queue: Pop failed, since the queue is empty."); // 경고 메시지 (주석 처리됨)
    return false; // 실패
  }

  uint32_t rd_idx = queue->rd_idx & queue->mask; // 읽기 인덱스 계산

  storage_packet->base_time = queue->storage_packet[rd_idx].base_time; // base_time 설정
  storage_packet->points_num = queue->storage_packet[rd_idx].points_num; // points_num 설정
  storage_packet->points.resize(queue->storage_packet[rd_idx].points_num); // points 크기 설정

  memcpy(storage_packet->points.data(), queue->storage_packet[rd_idx].points.data(), (storage_packet->points_num) * sizeof(PointXyzlt)); // 데이터 복사
  return true; // 성공
}

// 큐 팝 업데이트 함수
void QueuePopUpdate(LidarDataQueue *queue) {
  queue->rd_idx++; // 읽기 인덱스 증가
}

// 큐에서 데이터 팝 함수
bool QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet) {
  if (!QueuePrePop(queue, storage_packet)) { // 데이터 미리 가져오기 실패 여부 확인
    return false; // 실패
  }
  QueuePopUpdate(queue); // 팝 업데이트

  return true; // 성공
}

// 큐에 사용된 크기 반환 함수
uint32_t QueueUsedSize(LidarDataQueue *queue) {
  return queue->wr_idx - queue->rd_idx; // 사용된 크기 계산
}

// 큐의 사용되지 않은 크기 반환 함수
uint32_t QueueUnusedSize(LidarDataQueue *queue) {
  return (queue->size - QueueUsedSize(queue)); // 사용되지 않은 크기 계산
}

// 큐가 가득 찼는지 확인하는 함수
bool QueueIsFull(LidarDataQueue *queue) {
  return ((queue->wr_idx - queue->rd_idx) > queue->mask); // 큐가 가득 찼는지 확인
}

// 큐가 비어있는지 확인하는 함수
bool QueueIsEmpty(LidarDataQueue *queue) {
  return (queue->rd_idx == queue->wr_idx); // 큐가 비어있는지 확인
}

// 큐에 데이터 푸시 함수
uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, const uint64_t base_time) {
  uint32_t wr_idx = queue->wr_idx & queue->mask; // 쓰기 인덱스 계산
  PointPacket* lidar_point_data = reinterpret_cast<PointPacket*>(data); // 데이터 포인터 캐스팅
  queue->storage_packet[wr_idx].base_time = base_time; // base_time 설정
  queue->storage_packet[wr_idx].points_num = lidar_point_data->points_num; // points_num 설정

  queue->storage_packet[wr_idx].points.clear(); // points 초기화
  queue->storage_packet[wr_idx].points.resize(lidar_point_data->points_num); // points 크기 설정
  memcpy(queue->storage_packet[wr_idx].points.data(), lidar_point_data->points, sizeof(PointXyzlt) * (lidar_point_data->points_num)); // 데이터 복사

  queue->wr_idx++; // 쓰기 인덱스 증가
  return 1; // 성공
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
