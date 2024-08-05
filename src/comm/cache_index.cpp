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
#include "cache_index.h" // "cache_index.h" 헤더 파일 포함
#include "livox_lidar_def.h" // "livox_lidar_def.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

CacheIndex::CacheIndex() { // CacheIndex 클래스의 생성자 정의
  std::array<bool, kMaxLidarCount> index_cache = {0}; // index_cache 배열 초기화
  index_cache_.swap(index_cache); // index_cache_와 초기화된 배열을 교체
}

int8_t CacheIndex::GetFreeIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index) {
  std::string key; // 키 문자열 선언
  int8_t ret = GenerateIndexKey(livox_lidar_type, handle, key); // 인덱스 키 생성
  if (ret != 0) { // 키 생성 실패 여부 확인
    return -1; // 실패 시 -1 반환
  }
  {
    std::lock_guard<std::mutex> lock(index_mutex_); // index_mutex_ 잠금
    if (map_index_.find(key) != map_index_.end()) { // 키가 이미 존재하는지 확인
      index = map_index_[key]; // 존재하면 인덱스 설정
      return 0; // 성공 시 0 반환
    }
  }

  {
    printf("GetFreeIndex key:%s.\n", key.c_str()); // 키 출력
    std::lock_guard<std::mutex> lock(index_mutex_); // index_mutex_ 잠금
    for (size_t i = 0; i < kMaxSourceLidar; ++i) { // 최대 소스 Lidar 개수만큼 반복
      if (!index_cache_[i]) { // 비어있는 인덱스 찾기
        index_cache_[i] = 1; // 해당 인덱스를 사용 중으로 표시
        map_index_[key] = static_cast<uint8_t>(i); // 키와 인덱스 맵핑
        index = static_cast<uint8_t>(i); // 인덱스 설정
        return 0; // 성공 시 0 반환
      }
    }
  }
  return -1; // 실패 시 -1 반환
}

int8_t CacheIndex::GenerateIndexKey(const uint8_t livox_lidar_type, const uint32_t handle, std::string& key) {
  if (livox_lidar_type == kLivoxLidarType) { // livox_lidar_type이 올바른지 확인
    key = "livox_lidar_" + std::to_string(handle); // 키 생성
  } else {
    printf("Can not generate index, the livox lidar type is unknown, the livox lidar type:%u\n", livox_lidar_type); // 오류 메시지 출력
    return -1; // 실패 시 -1 반환
  }
  return 0; // 성공 시 0 반환
}

int8_t CacheIndex::GetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index) {
  std::string key; // 키 문자열 선언
  int8_t ret = GenerateIndexKey(livox_lidar_type, handle, key); // 인덱스 키 생성
  if (ret != 0) { // 키 생성 실패 여부 확인
    return -1; // 실패 시 -1 반환
  }

  if (map_index_.find(key) != map_index_.end()) { // 키가 존재하는지 확인
    std::lock_guard<std::mutex> lock(index_mutex_); // index_mutex_ 잠금
    index = map_index_[key]; // 인덱스 설정
    return 0; // 성공 시 0 반환
  }
  printf("Can not get index, the livox lidar type:%u, handle:%u\n", livox_lidar_type, handle); // 오류 메시지 출력
  return -1; // 실패 시 -1 반환
}

int8_t CacheIndex::LvxGetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index) {
  std::string key; // 키 문자열 선언
  int8_t ret = GenerateIndexKey(livox_lidar_type, handle, key); // 인덱스 키 생성
  if (ret != 0) { // 키 생성 실패 여부 확인
    return -1; // 실패 시 -1 반환
  }

  if (map_index_.find(key) != map_index_.end()) { // 키가 존재하는지 확인
    index = map_index_[key]; // 인덱스 설정
    return 0; // 성공 시 0 반환
  }

  return GetFreeIndex(livox_lidar_type, handle, index); // 비어있는 인덱스 가져오기
}

void CacheIndex::ResetIndex(LidarDevice *lidar) {
  std::string key; // 키 문자열 선언
  int8_t ret = GenerateIndexKey(lidar->lidar_type, lidar->handle, key); // 인덱스 키 생성
  if (ret != 0) { // 키 생성 실패 여부 확인
    printf("Reset index failed, can not generate index key, lidar type:%u, handle:%u.\n", lidar->lidar_type, lidar->handle); // 오류 메시지 출력
    return; // 함수 종료
  }

  if (map_index_.find(key) != map_index_.end()) { // 키가 존재하는지 확인
    uint8_t index = map_index_[key]; // 인덱스 설정
    std::lock_guard<std::mutex> lock(index_mutex_); // index_mutex_ 잠금
    map_index_.erase(key); // 키 삭제
    index_cache_[index] = 0; // 인덱스를 비어있음으로 표시
  }
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
