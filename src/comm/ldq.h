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

#ifndef LIVOX_ROS_DRIVER_LDQ_H_ // LIVOX_ROS_DRIVER_LDQ_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER_LDQ_H_ // LIVOX_ROS_DRIVER_LDQ_H_를 정의

#include <stdint.h> // 정수형 데이터 타입 사용을 위한 헤더 파일 포함
#include <vector> // 벡터 사용을 위한 헤더 파일 포함

#include "comm/comm.h" // "comm/comm.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// 2의 제곱 여부 확인 함수 (인라인 정적 함수)
inline static bool IsPowerOf2(uint32_t size) {
  return (size != 0) && ((size & (size - 1)) == 0);
}

// 2의 제곱으로 반올림하는 함수 (인라인 정적 함수)
inline static uint32_t RoundupPowerOf2(uint32_t size) {
  uint32_t power2_val = 0;
  for (int i = 0; i < 32; i++) {
    power2_val = ((uint32_t)1) << i; // 2의 i 제곱 값 계산
    if (size <= power2_val) {
      break; // 입력 크기보다 큰 2의 제곱 값 찾기
    }
  }

  return power2_val; // 2의 제곱 값 반환
}

/** 큐 작업 함수 */
// 큐 초기화 함수
bool InitQueue(LidarDataQueue *queue, uint32_t queue_size);
// 큐 해제 함수
bool DeInitQueue(LidarDataQueue *queue);
// 큐 리셋 함수
void ResetQueue(LidarDataQueue *queue);
// 큐에서 팝할 데이터 미리 가져오기
bool QueuePrePop(LidarDataQueue *queue, StoragePacket *storage_packet);
// 큐 팝 업데이트 함수
void QueuePopUpdate(LidarDataQueue *queue);
// 큐에서 데이터 팝 함수
bool QueuePop(LidarDataQueue *queue, StoragePacket *storage_packet);
// 큐에 사용된 크기 반환 함수
uint32_t QueueUsedSize(LidarDataQueue *queue);
// 큐의 사용되지 않은 크기 반환 함수
uint32_t QueueUnusedSize(LidarDataQueue *queue);
// 큐가 가득 찼는지 확인하는 함수
bool QueueIsFull(LidarDataQueue *queue);
// 큐가 비어있는지 확인하는 함수
bool QueueIsEmpty(LidarDataQueue *queue);
// 큐에 데이터 푸시 함수
uint32_t QueuePushAny(LidarDataQueue *queue, uint8_t *data, const uint64_t base_time);

}  // namespace livox_ros // livox_ros 네임스페이스 끝

#endif // LIVOX_ROS_DRIVER_LDQ_H_ // LIVOX_ROS_DRIVER_LDQ_H_ 종료
