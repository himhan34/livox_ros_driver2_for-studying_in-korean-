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
/** Livox LiDAR 데이터 소스, 의존하는 라이다로부터 데이터 수집 */

#ifndef LIVOX_ROS_DRIVER_LDS_LIDAR_H_  // 헤더 파일이 중복 포함되지 않도록 방지
#define LIVOX_ROS_DRIVER_LDS_LIDAR_H_

#include <memory>  // 스마트 포인터 사용을 위해 포함
#include <mutex>  // 뮤텍스 사용을 위해 포함
#include <vector>  // 벡터 사용을 위해 포함

#include "lds.h"  // Lds 클래스 포함
#include "comm/comm.h"  // 통신 관련 헤더 파일 포함

#include "livox_lidar_api.h"  // Livox LiDAR API 포함
#include "livox_lidar_def.h"  // Livox LiDAR 정의 포함

#include "rapidjson/document.h"  // RapidJSON 라이브러리 포함

namespace livox_ros {  // livox_ros 네임스페이스 정의

class LdsLidar final : public Lds {  // Lds를 상속받는 LdsLidar 클래스 정의
 public:
  static LdsLidar *GetInstance(double publish_freq) {  // LdsLidar 인스턴스를 얻기 위한 정적 메서드
    printf("LdsLidar *GetInstance\n");
    static LdsLidar lds_lidar(publish_freq);  // 정적 LdsLidar 인스턴스 생성
    return &lds_lidar;
  }

  bool InitLdsLidar(const std::string& path_name);  // LdsLidar 초기화 메서드
  bool Start();  // LdsLidar 시작 메서드

  int DeInitLdsLidar(void);  // LdsLidar 종료 메서드
 private:
  LdsLidar(double publish_freq);  // 생성자
  LdsLidar(const LdsLidar &) = delete;  // 복사 생성자 사용 금지
  ~LdsLidar();  // 소멸자
  LdsLidar &operator=(const LdsLidar &) = delete;  // 복사 대입 연산자 사용 금지

  bool ParseSummaryConfig();  // 요약 설정 파싱 메서드

  bool InitLidars();  // 라이다 초기화 메서드
  bool InitLivoxLidar();  // 새로운 SDK를 위한 Livox 라이다 초기화 메서드

  bool LivoxLidarStart();  // Livox 라이다 시작 메서드

  void ResetLdsLidar(void);  // LdsLidar 리셋 메서드

  void SetLidarPubHandle();  // 라이다 데이터 출력을 위한 핸들 설정 메서드

  // 자동 연결 모드 설정 메서드
  void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
  void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
  bool IsAutoConnectMode(void) { return auto_connect_mode_; }

  virtual void PrepareExit(void);  // 종료 준비 메서드

 public:
  std::mutex config_mutex_;  // 설정 뮤텍스

 private:
  std::string path_;  // 설정 파일 경로
  LidarSummaryInfo lidar_summary_info_;  // 라이다 요약 정보

  bool auto_connect_mode_;  // 자동 연결 모드 여부
  uint32_t whitelist_count_;  // 화이트리스트 개수
  volatile bool is_initialized_;  // 초기화 여부
  char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];  // 화이트리스트 배열
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_LIDAR_H_
