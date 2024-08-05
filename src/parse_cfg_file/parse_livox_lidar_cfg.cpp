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

#include "parse_livox_lidar_cfg.h" // "parse_livox_lidar_cfg.h" 헤더 파일 포함
#include <iostream> // 입출력을 위한 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// LivoxLidarConfigParser 클래스의 Parse 함수 정의
bool LivoxLidarConfigParser::Parse(std::vector<UserLivoxLidarConfig> &lidar_configs) {
  FILE* raw_file = std::fopen(path_.c_str(), "rb"); // 설정 파일을 읽기 모드로 열기
  if (!raw_file) { // 파일 열기 실패 시
    std::cout << "failed to open config file: " << path_ << std::endl; // 오류 메시지 출력
    return false; // false 반환
  }

  lidar_configs.clear(); // Lidar 설정 벡터 초기화
  char read_buffer[kMaxBufferSize]; // 읽기 버퍼 정의
  rapidjson::FileReadStream config_file(raw_file, read_buffer, sizeof(read_buffer)); // 파일 읽기 스트림 생성
  rapidjson::Document doc; // JSON 문서 객체 생성

  do {
    if (doc.ParseStream(config_file).HasParseError()) { // JSON 파싱 중 오류 발생 시
      std::cout << "failed to parse config jason" << std::endl; // 오류 메시지 출력
      break; // 루프 탈출
    }
    if (!doc.HasMember("lidar_configs") || !doc["lidar_configs"].IsArray() || 0 == doc["lidar_configs"].Size()) { // "lidar_configs" 멤버가 없거나 배열이 아니거나 비어 있을 시
      std::cout << "there is no user-defined config" << std::endl; // 오류 메시지 출력
      break; // 루프 탈출
    }
    if (!ParseUserConfigs(doc, lidar_configs)) { // 사용자 설정 파싱 실패 시
      std::cout << "failed to parse basic configs" << std::endl; // 오류 메시지 출력
      break; // 루프 탈출
    }
    return true; // 성공 시 true 반환
  } while (false); // do-while 루프 종료

  std::fclose(raw_file); // 파일 닫기
  return false; // 실패 시 false 반환
}

// 사용자 설정 파싱 함수 정의
bool LivoxLidarConfigParser::ParseUserConfigs(const rapidjson::Document &doc, std::vector<UserLivoxLidarConfig> &user_configs) {
  const rapidjson::Value &lidar_configs = doc["lidar_configs"]; // "lidar_configs" 객체 가져오기
  for (auto &config : lidar_configs.GetArray()) { // 각 설정에 대해 반복
    if (!config.HasMember("ip")) { // "ip" 멤버가 없을 시
      continue; // 다음 설정으로 이동
    }
    UserLivoxLidarConfig user_config; // 사용자 설정 객체 생성

    // 사용자 설정 파싱
    user_config.handle = IpStringToNum(std::string(config["ip"].GetString())); // IP 주소를 숫자로 변환하여 핸들 설정
    if (!config.HasMember("pcl_data_type")) { // "pcl_data_type" 멤버가 없을 시
      user_config.pcl_data_type = -1; // 기본값 설정
    } else {
      user_config.pcl_data_type = static_cast<int8_t>(config["pcl_data_type"].GetInt()); // 값 설정
    }
    if (!config.HasMember("pattern_mode")) { // "pattern_mode" 멤버가 없을 시
      user_config.pattern_mode = -1; // 기본값 설정
    } else {
      user_config.pattern_mode = static_cast<int8_t>(config["pattern_mode"].GetInt()); // 값 설정
    }
    if (!config.HasMember("blind_spot_set")) { // "blind_spot_set" 멤버가 없을 시
      user_config.blind_spot_set = -1; // 기본값 설정
    } else {
      user_config.blind_spot_set = static_cast<int8_t>(config["blind_spot_set"].GetInt()); // 값 설정
    }
    if (!config.HasMember("dual_emit_en")) { // "dual_emit_en" 멤버가 없을 시
      user_config.dual_emit_en = -1; // 기본값 설정
    } else {
      user_config.dual_emit_en = static_cast<uint8_t>(config["dual_emit_en"].GetInt()); // 값 설정
    }
    if (!config.HasMember("extrinsic_parameter")) { // "extrinsic_parameter" 멤버가 없을 시
      memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param)); // 외부 매개변수 초기화
    } else {
      auto &value = config["extrinsic_parameter"]; // 외부 매개변수 값 가져오기
      if (!ParseExtrinsics(value, user_config.extrinsic_param)) { // 외부 매개변수 파싱 실패 시
        memset(&user_config.extrinsic_param, 0, sizeof(user_config.extrinsic_param)); // 외부 매개변수 초기화
        std::cout << "failed to parse extrinsic parameters, ip: " << IpNumToString(user_config.handle) << std::endl; // 오류 메시지 출력
      }
    }
    user_config.set_bits = 0; // 설정 비트 초기화
    user_config.get_bits = 0; // 가져오기 비트 초기화

    user_configs.push_back(user_config); // 사용자 설정 벡터에 추가
  }

  if (0 == user_configs.size()) { // 유효한 기본 설정이 없을 시
    std::cout << "no valid base configs" << std::endl; // 오류 메시지 출력
    return false; // false 반환
  }
  std::cout << "successfully parse base config, counts: " << user_configs.size() << std::endl; // 성공 메시지 출력
  return true; // true 반환
}

// 외부 매개변수 파싱 함수 정의
bool LivoxLidarConfigParser::ParseExtrinsics(const rapidjson::Value &value, ExtParameter &param) {
  if (!value.HasMember("roll")) { // "roll" 멤버가 없을 시
    param.roll = 0.0f; // 기본값 설정
  } else {
    param.roll = static_cast<float>(value["roll"].GetFloat()); // 값 설정
  }
  if (!value.HasMember("pitch")) { // "pitch" 멤버가 없을 시
    param.pitch = 0.0f; // 기본값 설정
  } else {
    param.pitch = static_cast<float>(value["pitch"].GetFloat()); // 값 설정
  }
  if (!value.HasMember("yaw")) { // "yaw" 멤버가 없을 시
    param.yaw = 0.0f; // 기본값 설정
  } else {
    param.yaw = static_cast<float>(value["yaw"].GetFloat()); // 값 설정
  }
  if (!value.HasMember("x")) { // "x" 멤버가 없을 시
    param.x = 0; // 기본값 설정
  } else {
    param.x = static_cast<int32_t>(value["x"].GetInt()); // 값 설정
  }
  if (!value.HasMember("y")) { // "y" 멤버가 없을 시
    param.y = 0; // 기본값 설정
  } else {
    param.y = static_cast<int32_t>(value["y"].GetInt()); // 값 설정
  }
  if (!value.HasMember("z")) { // "z" 멤버가 없을 시
    param.z = 0; // 기본값 설정
  } else {
    param.z = static_cast<int32_t>(value["z"].GetInt()); // 값 설정
  }

  return true; // true 반환
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
