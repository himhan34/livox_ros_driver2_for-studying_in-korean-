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


#ifndef LIVOX_DRIVER_PUB_HANDLER_H_ // LIVOX_DRIVER_PUB_HANDLER_H_가 정의되지 않았을 때
#define LIVOX_DRIVER_PUB_HANDLER_H_ // LIVOX_DRIVER_PUB_HANDLER_H_를 정의

#include <atomic> // 원자 변수 사용을 위한 헤더 파일 포함
#include <cstring> // 문자열 처리 함수 사용을 위한 헤더 파일 포함
#include <condition_variable> // 조건 변수 사용을 위한 헤더 파일 포함
#include <deque> // 덱 사용을 위한 헤더 파일 포함
#include <functional> // 함수 객체 사용을 위한 헤더 파일 포함
#include <map> // 맵 사용을 위한 헤더 파일 포함
#include <memory> // 스마트 포인터 사용을 위한 헤더 파일 포함
#include <mutex> // 뮤텍스 사용을 위한 헤더 파일 포함
#include <thread> // 스레드 사용을 위한 헤더 파일 포함

#include "livox_lidar_def.h" // Livox Lidar 정의 포함
#include "livox_lidar_api.h" // Livox Lidar API 포함
#include "comm/comm.h" // 공통 함수 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// LidarPubHandler 클래스 정의
class LidarPubHandler {
 public:
  LidarPubHandler(); // 생성자
  ~LidarPubHandler() {} // 소멸자

  void PointCloudProcess(RawPacket& pkt); // 포인트 클라우드 처리 함수
  void SetLidarsExtParam(LidarExtParameter param); // Lidar 외부 매개변수 설정 함수
  void GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds); // Lidar 포인트 클라우드 가져오기 함수

  uint64_t GetRecentTimeStamp(); // 최근 타임스탬프 가져오기 함수
  uint32_t GetLidarPointCloudsSize(); // Lidar 포인트 클라우드 크기 가져오기 함수
  uint64_t GetLidarBaseTime(); // Lidar 베이스 타임 가져오기 함수

 private:
  void LivoxLidarPointCloudProcess(RawPacket & pkt); // Livox Lidar 포인트 클라우드 처리 함수
  void ProcessCartesianHighPoint(RawPacket & pkt); // Cartesian 고해상도 포인트 처리 함수
  void ProcessCartesianLowPoint(RawPacket & pkt); // Cartesian 저해상도 포인트 처리 함수
  void ProcessSphericalPoint(RawPacket & pkt); // Spherical 포인트 처리 함수
  std::vector<PointXyzlt> points_clouds_; // 포인트 클라우드 벡터
  ExtParameterDetailed extrinsic_ = { // 외부 매개변수 초기화
    {0, 0, 0},
    {
      {1, 0, 0},
      {0, 1, 1},
      {0, 0, 1}
    }
  };
  std::mutex mutex_; // 뮤텍스
  std::atomic_bool is_set_extrinsic_params_; // 외부 매개변수 설정 여부
};

// PubHandler 클래스 정의
class PubHandler {
 public:
  using PointCloudsCallback = std::function<void(PointFrame*, void *)>; // 포인트 클라우드 콜백 함수 타입 정의
  using ImuDataCallback = std::function<void(ImuData*, void*)>; // IMU 데이터 콜백 함수 타입 정의
  using TimePoint = std::chrono::high_resolution_clock::time_point; // 시간 포인트 타입 정의

  PubHandler() {} // 생성자

  ~ PubHandler() { Uninit(); } // 소멸자

  void Uninit(); // 종료 함수
  void RequestExit(); // 종료 요청 함수
  void Init(); // 초기화 함수
  void SetPointCloudConfig(const double publish_freq); // 포인트 클라우드 구성 설정 함수
  void SetPointCloudsCallback(PointCloudsCallback cb, void* client_data); // 포인트 클라우드 콜백 설정 함수
  void AddLidarsExtParam(LidarExtParameter& extrinsic_params); // Lidar 외부 매개변수 추가 함수
  void ClearAllLidarsExtrinsicParams(); // 모든 Lidar 외부 매개변수 제거 함수
  void SetImuDataCallback(ImuDataCallback cb, void* client_data); // IMU 데이터 콜백 설정 함수

 private:
  // Raw 데이터를 처리하는 스레드 함수
  void RawDataProcess();
  std::atomic<bool> is_quit_{false}; // 종료 여부 원자 변수
  std::shared_ptr<std::thread> point_process_thread_; // 포인트 처리 스레드 포인터
  std::mutex packet_mutex_; // 패킷 뮤텍스
  std::condition_variable packet_condition_; // 패킷 조건 변수

  // 발행 콜백 함수
  void CheckTimer(uint32_t id); // 타이머 체크 함수
  void PublishPointCloud(); // 포인트 클라우드 발행 함수
  static void OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                             LivoxLidarEthernetPacket *data, void *client_data); // 포인트 클라우드 콜백 함수
  
  static bool GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id); // Lidar ID 가져오기 함수
  static uint64_t GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size); // 이더넷 패킷 타임스탬프 가져오기 함수

  PointCloudsCallback points_callback_; // 포인트 클라우드 콜백 함수
  void* pub_client_data_ = nullptr; // 클라이언트 데이터 포인터

  ImuDataCallback imu_callback_; // IMU 데이터 콜백 함수
  void* imu_client_data_ = nullptr; // 클라이언트 데이터 포인터

  PointFrame frame_; // 포인트 프레임 객체

  std::deque<RawPacket> raw_packet_queue_; // Raw 패킷 큐

  // 발행 구성 변수
  uint64_t publish_interval_ = 100000000; // 발행 간격 (100 ms)
  uint64_t publish_interval_tolerance_ = 100000000; // 발행 간격 허용 오차 (100 ms)
  uint64_t publish_interval_ms_ = 100; // 발행 간격 (100 ms)
  TimePoint last_pub_time_; // 마지막 발행 시간

  std::map<uint32_t, std::unique_ptr<LidarPubHandler>> lidar_process_handlers_; // Lidar 프로세스 핸들러 맵
  std::map<uint32_t, std::vector<PointXyzlt>> points_; // 포인트 맵
  std::map<uint32_t, LidarExtParameter> lidar_extrinsics_; // Lidar 외부 매개변수 맵
  static std::atomic<bool> is_timestamp_sync_; // 타임스탬프 동기화 여부
  uint16_t lidar_listen_id_ = 0; // Lidar 리스닝 ID
};

PubHandler &pub_handler(); // PubHandler 싱글톤 인스턴스 반환 함수

}  // namespace livox_ros // livox_ros 네임스페이스 끝

#endif  // LIVOX_DRIVER_PUB_HANDLER_H_ // LIVOX_DRIVER_PUB_HANDLER_H_ 종료
