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
#include "comm/comm.h" // "comm/comm.h" 헤더 파일 포함
#include <string.h> // 문자열 처리를 위한 string.h 라이브러리 포함
#include <arpa/inet.h> // 네트워크 주소 변환을 위한 arpa/inet.h 라이브러리 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

/** 공통 함수 --------------------------------------------------------- */
// 파일 경로 유효성 검사 함수
bool IsFilePathValid(const char *path_str) {
  int str_len = strlen(path_str); // 문자열 길이 계산

  if ((str_len > kPathStrMinSize) && (str_len < kPathStrMaxSize)) { // 경로 길이 범위 확인
    return true; // 유효한 경로이면 true 반환
  } else {
    return false; // 유효하지 않은 경로이면 false 반환
  }
}

// 패킷 큐 크기 계산 함수
uint32_t CalculatePacketQueueSize(const double publish_freq) {
  uint32_t queue_size = 10; // 기본 큐 크기 설정
  if (publish_freq > 10.0) { // 발행 빈도가 10.0보다 큰지 확인
    queue_size = static_cast<uint32_t>(publish_freq) + 1; // 큐 크기 설정
  }
  return queue_size; // 큐 크기 반환
}

// IP 번호를 문자열로 변환하는 함수
std::string IpNumToString(uint32_t ip_num) {
  struct in_addr ip; // in_addr 구조체 선언
  ip.s_addr = ip_num; // IP 주소 설정
  return std::string(inet_ntoa(ip)); // IP 주소를 문자열로 변환하여 반환
}

// IP 문자열을 숫자로 변환하는 함수
uint32_t IpStringToNum(std::string ip_string) {
  return static_cast<uint32_t>(inet_addr(ip_string.c_str())); // IP 문자열을 숫자로 변환하여 반환
}

// 마침표를 밑줄로 대체하는 함수
std::string ReplacePeriodByUnderline(std::string str) {
  std::size_t pos = str.find("."); // 문자열에서 마침표 위치 찾기
  while (pos != std::string::npos) { // 마침표가 문자열에 있는 동안
    str.replace(pos, 1, "_"); // 마침표를 밑줄로 대체
    pos = str.find("."); // 다음 마침표 위치 찾기
  }
  return str; // 변환된 문자열 반환
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
