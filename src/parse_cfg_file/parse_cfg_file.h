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

#ifndef LIVOX_ROS_DRIVER_PARSE_CFG_FILE_H_ // LIVOX_ROS_DRIVER_PARSE_CFG_FILE_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER_PARSE_CFG_FILE_H_ // LIVOX_ROS_DRIVER_PARSE_CFG_FILE_H_를 정의

#include "../comm/comm.h" // 공통 헤더 파일 포함

#include "rapidjson/document.h" // rapidjson 문서 포함
#include "rapidjson/filereadstream.h" // rapidjson 파일 읽기 스트림 포함
#include "rapidjson/stringbuffer.h" // rapidjson 문자열 버퍼 포함

#include <string> // 문자열 처리를 위한 헤더 파일 포함
#include <vector> // 벡터를 위한 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

class ParseCfgFile { // ParseCfgFile 클래스 정의
 public:
  explicit ParseCfgFile(const std::string& path); // 생성자 정의
  ~ParseCfgFile() {} // 소멸자 정의

  bool ParseSummaryInfo(LidarSummaryInfo& lidar_summary_info); // 요약 정보 파싱 함수 정의
  
 private:
  const std::string path_; // 경로를 저장할 멤버 변수
};

} // namespace livox_ros // livox_ros 네임스페이스 끝

#endif // LIVOX_ROS_DRIVER_PARSE_CFG_FILE_H_ // LIVOX_ROS_DRIVER_PARSE_CFG_FILE_H_ 종료
