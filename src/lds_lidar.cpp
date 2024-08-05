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

#include "lds_lidar.h"  // "lds_lidar.h" 헤더 파일 포함

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>

#ifdef WIN32  // 윈도우 플랫폼인 경우
#include <winsock2.h>
#include <ws2def.h>
#pragma comment(lib, "Ws2_32.lib")
#else  // 윈도우 플랫폼이 아닌 경우
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif // WIN32

#include "comm/comm.h"  // 통신 관련 헤더 파일 포함
#include "comm/pub_handler.h"

#include "parse_cfg_file/parse_cfg_file.h"
#include "parse_cfg_file/parse_livox_lidar_cfg.h"

#include "call_back/lidar_common_callback.h"
#include "call_back/livox_lidar_callback.h"

using namespace std;  // 표준 라이브러리의 모든 이름을 전역적으로 사용

namespace livox_ros {  // livox_ros 네임스페이스 정의

/** 상수 변수 ------------------------------------------------------------*/
/** 콜백 함수 사용을 위한 전역 변수 */
LdsLidar *g_lds_ldiar = nullptr;

/** 공통적으로 사용되는 전역 함수 -------------------------------------------*/

/** LdsLidar 클래스의 생성자 정의 -------------------------------------------*/
LdsLidar::LdsLidar(double publish_freq)
    : Lds(publish_freq, kSourceRawLidar),  // 부모 클래스 Lds의 생성자 호출
      auto_connect_mode_(true),
      whitelist_count_(0),
      is_initialized_(false) {
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));  // 화이트리스트 배열 초기화
  ResetLdsLidar();  // LdsLidar 초기화 함수 호출
}

LdsLidar::~LdsLidar() {}  // 소멸자 정의 (현재는 아무 작업도 하지 않음)

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }  // Lds 초기화 함수 호출


bool LdsLidar::InitLdsLidar(const std::string& path_name) {  // LdsLidar 초기화 함수
  if (is_initialized_) {
    printf("Lds is already inited!\n");  // 이미 초기화된 경우 메시지 출력
    return false;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;  // g_lds_ldiar 포인터 설정
  }

  path_ = path_name;
  if (!InitLidars()) {
    return false;  // 라이다 초기화 실패 시 false 반환
  }
  SetLidarPubHandle();
  if (!Start()) {
    return false;  // 시작 실패 시 false 반환
  }
  is_initialized_ = true;
  return true;  // 초기화 성공 시 true 반환
}

bool LdsLidar::InitLidars() {  // 라이다 초기화 함수
  if (!ParseSummaryConfig()) {
    return false;  // 설정 파일 파싱 실패 시 false 반환
  }
  std::cout << "config lidar type: " << static_cast<int>(lidar_summary_info_.lidar_type) << std::endl;

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!InitLivoxLidar()) {
      return false;  // Livox 라이다 초기화 실패 시 false 반환
    }
  }
  return true;  // 초기화 성공 시 true 반환
}

bool LdsLidar::Start() {  // 라이다 시작 함수
  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    if (!LivoxLidarStart()) {
      return false;  // Livox 라이다 시작 실패 시 false 반환
    }
  }
  return true;  // 시작 성공 시 true 반환
}

bool LdsLidar::ParseSummaryConfig() {  // 설정 파일 파싱 함수
  return ParseCfgFile(path_).ParseSummaryInfo(lidar_summary_info_);
}

bool LdsLidar::InitLivoxLidar() {  // Livox 라이다 초기화 함수
#ifdef BUILDING_ROS2
  DisableLivoxSdkConsoleLogger();
#endif

  // 사용자 설정 파싱
  LivoxLidarConfigParser parser(path_);
  std::vector<UserLivoxLidarConfig> user_configs;
  if (!parser.Parse(user_configs)) {
    std::cout << "failed to parse user-defined config" << std::endl;
  }

  // SDK 초기화
  if (!LivoxLidarSdkInit(path_.c_str())) {
    std::cout << "Failed to init livox lidar sdk." << std::endl;
    return false;
  }

  // 라이다 장치 설정
  for (auto& config : user_configs) {
    uint8_t index = 0;
    int8_t ret = g_lds_ldiar->cache_index_.GetFreeIndex(kLivoxLidarType, config.handle, index);
    if (ret != 0) {
      std::cout << "failed to get free index, lidar ip: " << IpNumToString(config.handle) << std::endl;
      continue;
    }
    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[index]);
    p_lidar->lidar_type = kLivoxLidarType;
    p_lidar->livox_config = config;
    p_lidar->handle = config.handle;

    LidarExtParameter lidar_param;
    lidar_param.handle = config.handle;
    lidar_param.lidar_type = kLivoxLidarType;
    if (config.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
      // 임시 해상도 설정
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x / 10;
      lidar_param.param.y     = config.extrinsic_param.y / 10;
      lidar_param.param.z     = config.extrinsic_param.z / 10;
    } else {
      lidar_param.param.roll  = config.extrinsic_param.roll;
      lidar_param.param.pitch = config.extrinsic_param.pitch;
      lidar_param.param.yaw   = config.extrinsic_param.yaw;
      lidar_param.param.x     = config.extrinsic_param.x;
      lidar_param.param.y     = config.extrinsic_param.y;
      lidar_param.param.z     = config.extrinsic_param.z;
    }
    pub_handler().AddLidarsExtParam(lidar_param);
  }

  SetLivoxLidarInfoChangeCallback(LivoxLidarCallback::LidarInfoChangeCallback, g_lds_ldiar);
  return true;
}

void LdsLidar::SetLidarPubHandle() {  // 라이다 데이터 출력을 위한 핸들 설정 함수
  pub_handler().SetPointCloudsCallback(LidarCommonCallback::OnLidarPointClounCb, g_lds_ldiar);
  pub_handler().SetImuDataCallback(LidarCommonCallback::LidarImuDataCallback, g_lds_ldiar);

  double publish_freq = Lds::GetLdsFrequency();
  pub_handler().SetPointCloudConfig(publish_freq);
}

bool LdsLidar::LivoxLidarStart() {  // Livox 라이다 시작 함수
  return true;
}

int LdsLidar::DeInitLdsLidar(void) {  // LdsLidar 종료 함수
  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  if (lidar_summary_info_.lidar_type & kLivoxLidarType) {
    LivoxLidarSdkUninit();
    printf("Livox Lidar SDK Deinit completely!\n");
  }

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }  // 종료 준비 함수

}  // namespace livox_ros
