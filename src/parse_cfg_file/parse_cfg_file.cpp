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

#include "parse_cfg_file.h" // "parse_cfg_file.h" 헤더 파일 포함

#include <iostream> // 입출력을 위한 헤더 파일 포함
#include <cstdio> // C 표준 입출력 함수를 위한 헤더 파일 포함
#include <arpa/inet.h> // 네트워크 주소 변환을 위한 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

ParseCfgFile::ParseCfgFile(const std::string& path) : path_(path) {} // 생성자 정의, 경로를 멤버 변수에 저장

bool ParseCfgFile::ParseSummaryInfo(LidarSummaryInfo& lidar_summary_info) { // 요약 정보 파싱 함수 정의
  FILE* raw_file = std::fopen(path_.c_str(), "rb"); // 파일을 읽기 모드로 열기
  if (!raw_file) { // 파일 열기 실패 시
    std::cout << "parse summary info failed, can not open file: " << path_ << std::endl; // 오류 메시지 출력
    return false; // false 반환
  }

  char read_buffer[kMaxBufferSize]; // 읽기 버퍼 정의
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer)); // 파일 읽기 스트림 생성
  rapidjson::Document doc; // JSON 문서 객체 생성
  do {
    if (doc.ParseStream(config_file).HasParseError()) { // JSON 파싱 중 오류 발생 시
      break; // 루프 탈출
    }
    if (!doc.HasMember("lidar_summary_info") || !doc["lidar_summary_info"].IsObject()) { // "lidar_summary_info" 멤버가 없거나 객체가 아닐 시
      break; // 루프 탈출
    }
    const rapidjson::Value &object = doc["lidar_summary_info"]; // "lidar_summary_info" 객체 가져오기
    if (!object.HasMember("lidar_type") || !object["lidar_type"].IsUint()) { // "lidar_type" 멤버가 없거나 unsigned int 타입이 아닐 시
      break; // 루프 탈출
    }
    lidar_summary_info.lidar_type = static_cast<uint8_t>(object["lidar_type"].GetUint()); // lidar_type 설정
    std::fclose(raw_file); // 파일 닫기
    return true; // true 반환
  } while (false); // do-while 루프 종료

  std::cout << "parse lidar type failed." << std::endl; // 오류 메시지 출력
  std::fclose(raw_file); // 파일 닫기
  return false; // false 반환
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
