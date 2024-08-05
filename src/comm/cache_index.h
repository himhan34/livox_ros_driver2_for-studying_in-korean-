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

#ifndef LIVOX_ROS_DRIVER_CACHE_INDEX_H_ // LIVOX_ROS_DRIVER_CACHE_INDEX_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER_CACHE_INDEX_H_ // LIVOX_ROS_DRIVER_CACHE_INDEX_H_를 정의

#include <mutex> // 뮤텍스 사용을 위한 mutex 라이브러리 포함
#include <array> // 배열 사용을 위한 array 라이브러리 포함
#include <map> // 맵 사용을 위한 map 라이브러리 포함
#include <string> // 문자열 처리를 위한 string 라이브러리 포함

#include "comm/comm.h" // "comm/comm.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

class CacheIndex { // CacheIndex 클래스 정의
 public:
  CacheIndex(); // 생성자
  // 비어있는 인덱스를 가져오는 함수
  int8_t GetFreeIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  // 인덱스를 가져오는 함수
  int8_t GetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  // 인덱스 키를 생성하는 함수
  int8_t GenerateIndexKey(const uint8_t livox_lidar_type, const uint32_t handle, std::string& key);
  // Livox 인덱스를 가져오는 함수
  int8_t LvxGetIndex(const uint8_t livox_lidar_type, const uint32_t handle, uint8_t& index);
  // 인덱스를 초기화하는 함수
  void ResetIndex(LidarDevice *lidar);

 private:
  std::mutex index_mutex_; // 인덱스 뮤텍스
  std::map<std::string, uint8_t> map_index_; /* key:handle/slot, val:index */ // 인덱스 맵
  std::array<bool, kMaxSourceLidar> index_cache_; // 인덱스 캐시 배열
};

} // namespace livox_ros // livox_ros 네임스페이스 끝

#endif // LIVOX_ROS_DRIVER_CACHE_INDEX_H_ // LIVOX_ROS_DRIVER_CACHE_INDEX_H_ 종료
