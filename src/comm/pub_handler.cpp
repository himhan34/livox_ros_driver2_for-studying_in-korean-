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

#include "pub_handler.h" // "pub_handler.h" 헤더 파일 포함

#include <cstdlib> // 표준 라이브러리 함수 사용을 위한 헤더 파일 포함
#include <chrono> // 시간 관련 기능을 위한 헤더 파일 포함
#include <iostream> // 입출력을 위한 헤더 파일 포함
#include <limits> // 한계 값을 위한 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

std::atomic<bool> PubHandler::is_timestamp_sync_; // timestamp 동기화 여부를 나타내는 원자 변수 정의

PubHandler &pub_handler() { // PubHandler의 싱글톤 인스턴스 반환 함수
  static PubHandler handler;
  return handler;
}

void PubHandler::Init() { // 초기화 함수 정의
}

void PubHandler::Uninit() { // 종료 함수 정의
  if (lidar_listen_id_ > 0) { // 리스닝 ID가 유효한지 확인
    LivoxLidarRemovePointCloudObserver(lidar_listen_id_); // 포인트 클라우드 옵저버 제거
    lidar_listen_id_ = 0; // 리스닝 ID 초기화
  }

  RequestExit(); // 종료 요청

  if (point_process_thread_ && point_process_thread_->joinable()) { // 포인트 처리 스레드가 존재하고 join 가능하면
    point_process_thread_->join(); // 스레드 종료 대기
    point_process_thread_ = nullptr; // 스레드 포인터 초기화
  } else {
    /* 다른 처리 */
  }
}

void PubHandler::RequestExit() { // 종료 요청 함수 정의
  is_quit_.store(true); // 종료 플래그 설정
}

void PubHandler::SetPointCloudConfig(const double publish_freq) { // 포인트 클라우드 구성 설정 함수 정의
  publish_interval_ = (kNsPerSecond / (publish_freq * 10)) * 10; // 발행 간격 계산
  publish_interval_tolerance_ = publish_interval_ - kNsTolerantFrameTimeDeviation; // 발행 간격 허용 오차 설정
  publish_interval_ms_ = publish_interval_ / kRatioOfMsToNs; // 발행 간격을 밀리초 단위로 변환
  if (!point_process_thread_) { // 포인트 처리 스레드가 존재하지 않으면
    point_process_thread_ = std::make_shared<std::thread>(&PubHandler::RawDataProcess, this); // 포인트 처리 스레드 생성
  }
  return;
}

void PubHandler::SetImuDataCallback(ImuDataCallback cb, void* client_data) { // IMU 데이터 콜백 설정 함수 정의
  imu_client_data_ = client_data; // 클라이언트 데이터 설정
  imu_callback_ = cb; // 콜백 함수 설정
}

void PubHandler::AddLidarsExtParam(LidarExtParameter& lidar_param) { // Lidar 외부 매개변수 추가 함수 정의
  std::unique_lock<std::mutex> lock(packet_mutex_); // 뮤텍스 잠금
  uint32_t id = 0; // ID 초기화
  GetLidarId(lidar_param.lidar_type, lidar_param.handle, id); // Lidar ID 가져오기
  lidar_extrinsics_[id] = lidar_param; // 외부 매개변수 저장
}

void PubHandler::ClearAllLidarsExtrinsicParams() { // 모든 Lidar 외부 매개변수 제거 함수 정의
  std::unique_lock<std::mutex> lock(packet_mutex_); // 뮤텍스 잠금
  lidar_extrinsics_.clear(); // 외부 매개변수 클리어
}

void PubHandler::SetPointCloudsCallback(PointCloudsCallback cb, void* client_data) { // 포인트 클라우드 콜백 설정 함수 정의
  pub_client_data_ = client_data; // 클라이언트 데이터 설정
  points_callback_ = cb; // 콜백 함수 설정
  lidar_listen_id_ = LivoxLidarAddPointCloudObserver(OnLivoxLidarPointCloudCallback, this); // 포인트 클라우드 옵저버 추가
}

void PubHandler::OnLivoxLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type,
                                                LivoxLidarEthernetPacket *data, void *client_data) { // 포인트 클라우드 콜백 함수 정의
  PubHandler* self = (PubHandler*)client_data; // 클라이언트 데이터를 PubHandler로 캐스팅
  if (!self) { // self가 nullptr인지 확인
    return;
  }

  if (data->time_type != kTimestampTypeNoSync) { // timestamp 타입이 동기화된 경우
    is_timestamp_sync_.store(true); // timestamp 동기화 설정
  } else {
    is_timestamp_sync_.store(false); // timestamp 동기화 해제
  }

  if (data->data_type == kLivoxLidarImuData) { // 데이터 타입이 IMU 데이터인 경우
    if (self->imu_callback_) { // IMU 콜백이 설정되어 있는지 확인
      RawImuPoint* imu = (RawImuPoint*) data->data; // 데이터를 RawImuPoint로 캐스팅
      ImuData imu_data; // ImuData 객체 생성
      imu_data.lidar_type = static_cast<uint8_t>(LidarProtoType::kLivoxLidarType); // Lidar 타입 설정
      imu_data.handle = handle; // 핸들 설정
      imu_data.time_stamp = GetEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp)); // 타임스탬프 설정
      imu_data.gyro_x = imu->gyro_x; // 자이로스코프 X 설정
      imu_data.gyro_y = imu->gyro_y; // 자이로스코프 Y 설정
      imu_data.gyro_z = imu->gyro_z; // 자이로스코프 Z 설정
      imu_data.acc_x = imu->acc_x; // 가속도계 X 설정
      imu_data.acc_y = imu->acc_y; // 가속도계 Y 설정
      imu_data.acc_z = imu->acc_z; // 가속도계 Z 설정
      self->imu_callback_(&imu_data, self->imu_client_data_); // IMU 콜백 호출
    }
    return;
  }
  RawPacket packet = {}; // RawPacket 객체 생성
  packet.handle = handle; // 핸들 설정
  packet.lidar_type = LidarProtoType::kLivoxLidarType; // Lidar 타입 설정
  packet.extrinsic_enable = false; // 외부 매개변수 사용 안함 설정
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) { // 디바이스 타입 확인
    packet.line_num = kLineNumberHAP; // 라인 번호 설정
  } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) { // 디바이스 타입 확인
    packet.line_num = kLineNumberMid360; // 라인 번호 설정
  } else {
    packet.line_num = kLineNumberDefault; // 기본 라인 번호 설정
  }
  packet.data_type = data->data_type; // 데이터 타입 설정
  packet.point_num = data->dot_num; // 포인트 개수 설정
  packet.point_interval = data->time_interval * 100 / data->dot_num; // 포인트 간격 계산 (ns)
  packet.time_stamp = GetEthPacketTimestamp(data->time_type, data->timestamp, sizeof(data->timestamp)); // 타임스탬프 설정
  uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1; // 데이터 길이 계산
  packet.raw_data.insert(packet.raw_data.end(), data->data, data->data + length); // 데이터 복사
  {
    std::unique_lock<std::mutex> lock(self->packet_mutex_); // 뮤텍스 잠금
    self->raw_packet_queue_.push_back(packet); // 패킷 큐에 추가
  }
  self->packet_condition_.notify_one(); // 조건 변수 알림

  return;
}

void PubHandler::PublishPointCloud() { // 포인트 클라우드 발행 함수 정의
  // 포인트 발행
  if (points_callback_) { // 콜백이 설정되어 있는지 확인
    points_callback_(&frame_, pub_client_data_); // 포인트 클라우드 콜백 호출
  }
  return;
}

void PubHandler::CheckTimer(uint32_t id) { // 타이머 체크 함수 정의

  if (PubHandler::is_timestamp_sync_.load()) { // 타임스탬프 동기화가 활성화된 경우
    auto& process_handler = lidar_process_handlers_[id]; // 프로세스 핸들러 가져오기
    uint64_t recent_time_ms = process_handler->GetRecentTimeStamp() / kRatioOfMsToNs; // 최근 타임스탬프 가져오기
    if ((recent_time_ms % publish_interval_ms_ != 0) || recent_time_ms == 0) { // 발행 간격에 맞지 않으면
      return;
    }

    uint64_t diff = process_handler->GetRecentTimeStamp() - process_handler->GetLidarBaseTime(); // 타임스탬프 차이 계산
    if (diff < publish_interval_tolerance_) { // 차이가 허용 오차보다 작으면
      return;
    }

    frame_.base_time[frame_.lidar_num] = process_handler->GetLidarBaseTime(); // 베이스 타임 설정
    points_[id].clear(); // 포인트 클리어
    process_handler->GetLidarPointClouds(points_[id]); // 포인트 클라우드 가져오기
    if (points_[id].empty()) { // 포인트가 비어있으면
      return;
    }
    PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num]; // Lidar 포인트 설정
    lidar_point.lidar_type = LidarProtoType::kLivoxLidarType; // Lidar 타입 설정
    lidar_point.handle = id; // 핸들 설정
    lidar_point.points_num = points_[id].size(); // 포인트 개수 설정
    lidar_point.points = points_[id].data(); // 포인트 데이터 설정
    frame_.lidar_num++; // Lidar 개수 증가
    
    if (frame_.lidar_num != 0) { // Lidar 개수가 0이 아니면
      PublishPointCloud(); // 포인트 클라우드 발행
      frame_.lidar_num = 0; // Lidar 개수 초기화
    }
  } else { // 타임스탬프 동기화가 비활성화된 경우
    auto now_time = std::chrono::high_resolution_clock::now(); // 현재 시간 가져오기
    // 처음 설정
    static bool first = true;
    if (first) { // 처음 호출인지 확인
      last_pub_time_ = now_time; // 마지막 발행 시간 설정
      first = false; // 처음 호출 플래그 해제
      return;
    }
    if (now_time - last_pub_time_ < std::chrono::nanoseconds(publish_interval_)) { // 발행 간격보다 시간이 짧으면
      return;
    }
    last_pub_time_ += std::chrono::nanoseconds(publish_interval_); // 마지막 발행 시간 업데이트
    for (auto &process_handler : lidar_process_handlers_) { // 모든 프로세스 핸들러에 대해
      frame_.base_time[frame_.lidar_num] = process_handler.second->GetLidarBaseTime(); // 베이스 타임 설정
      uint32_t handle = process_handler.first; // 핸들 설정
      points_[handle].clear(); // 포인트 클리어
      process_handler.second->GetLidarPointClouds(points_[handle]); // 포인트 클라우드 가져오기
      if (points_[handle].empty()) { // 포인트가 비어있으면
        continue;
      }
      PointPacket& lidar_point = frame_.lidar_point[frame_.lidar_num]; // Lidar 포인트 설정
      lidar_point.lidar_type = LidarProtoType::kLivoxLidarType; // Lidar 타입 설정
      lidar_point.handle = handle; // 핸들 설정
      lidar_point.points_num = points_[handle].size(); // 포인트 개수 설정
      lidar_point.points = points_[handle].data(); // 포인트 데이터 설정
      frame_.lidar_num++; // Lidar 개수 증가
    }
    PublishPointCloud(); // 포인트 클라우드 발행
    frame_.lidar_num = 0; // Lidar 개수 초기화
  }
  return;
}

void PubHandler::RawDataProcess() { // Raw 데이터 처리 함수 정의
  RawPacket raw_data; // RawPacket 객체 생성
  while (!is_quit_.load()) { // 종료 플래그가 설정되지 않은 동안
    {
      std::unique_lock<std::mutex> lock(packet_mutex_); // 뮤텍스 잠금
      if (raw_packet_queue_.empty()) { // 패킷 큐가 비어있으면
        packet_condition_.wait_for(lock, std::chrono::milliseconds(500)); // 500ms 대기
        if (raw_packet_queue_.empty()) { // 패킷 큐가 여전히 비어있으면
          continue;
        }
      }
      raw_data = raw_packet_queue_.front(); // 큐의 첫 번째 패킷 가져오기
      raw_packet_queue_.pop_front(); // 큐의 첫 번째 패킷 제거
    }
    uint32_t id = 0; // ID 초기화
    GetLidarId(raw_data.lidar_type, raw_data.handle, id); // Lidar ID 가져오기
    if (lidar_process_handlers_.find(id) == lidar_process_handlers_.end()) { // 프로세스 핸들러가 존재하지 않으면
      lidar_process_handlers_[id].reset(new LidarPubHandler()); // 새로운 핸들러 생성
    }
    auto &process_handler = lidar_process_handlers_[id]; // 프로세스 핸들러 가져오기
    if (lidar_extrinsics_.find(id) != lidar_extrinsics_.end()) { // 외부 매개변수가 존재하면
        lidar_process_handlers_[id]->SetLidarsExtParam(lidar_extrinsics_[id]); // 외부 매개변수 설정
    }
    process_handler->PointCloudProcess(raw_data); // 포인트 클라우드 처리
    CheckTimer(id); // 타이머 체크
  }
}

bool PubHandler::GetLidarId(LidarProtoType lidar_type, uint32_t handle, uint32_t& id) { // Lidar ID 가져오기 함수 정의
  if (lidar_type == kLivoxLidarType) { // Lidar 타입이 올바른지 확인
    id = handle; // ID 설정
    return true; // 성공 반환
  }
  return false; // 실패 반환
}

uint64_t PubHandler::GetEthPacketTimestamp(uint8_t timestamp_type, uint8_t* time_stamp, uint8_t size) { // 이더넷 패킷 타임스탬프 가져오기 함수 정의
  LdsStamp time; // LdsStamp 객체 생성
  memcpy(time.stamp_bytes, time_stamp, size); // 타임스탬프 복사

  if (timestamp_type == kTimestampTypeGptpOrPtp || timestamp_type == kTimestampTypeGps) { // 타임스탬프 타입 확인
    return time.stamp; // 타임스탬프 반환
  }

  return std::chrono::high_resolution_clock::now().time_since_epoch().count(); // 현재 시간 반환
}

/*******************************/
/*  LidarPubHandler Definitions */
LidarPubHandler::LidarPubHandler() : is_set_extrinsic_params_(false) {} // 생성자 정의

uint64_t LidarPubHandler::GetLidarBaseTime() { // Lidar 베이스 타임 가져오기 함수 정의
  if (points_clouds_.empty()) { // 포인트 클라우드가 비어있는지 확인
    return 0; // 0 반환
  }
  return points_clouds_.at(0).offset_time; // 첫 번째 포인트의 오프셋 타임 반환
}

void LidarPubHandler::GetLidarPointClouds(std::vector<PointXyzlt>& points_clouds) { // Lidar 포인트 클라우드 가져오기 함수 정의
  std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
  points_clouds.swap(points_clouds_); // 포인트 클라우드 교체
}

uint64_t LidarPubHandler::GetRecentTimeStamp() { // 최근 타임스탬프 가져오기 함수 정의
  if (points_clouds_.empty()) { // 포인트 클라우드가 비어있는지 확인
    return 0; // 0 반환
  }
  return points_clouds_.back().offset_time; // 마지막 포인트의 오프셋 타임 반환
}

uint32_t LidarPubHandler::GetLidarPointCloudsSize() { // Lidar 포인트 클라우드 크기 가져오기 함수 정의
  std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
  return points_clouds_.size(); // 포인트 클라우드 크기 반환
}

// 표준 형식으로 변환하고 외부 매개변수 보상
void LidarPubHandler::PointCloudProcess(RawPacket & pkt) { // 포인트 클라우드 처리 함수 정의
  if (pkt.lidar_type == LidarProtoType::kLivoxLidarType) { // Lidar 타입 확인
    LivoxLidarPointCloudProcess(pkt); // Livox Lidar 포인트 클라우드 처리
  } else {
    static bool flag = false;
    if (!flag) { // 프로토콜 타입이 지원되지 않으면
      std::cout << "error, unsupported protocol type: " << static_cast<int>(pkt.lidar_type) << std::endl;
      flag = true;      
    }
  }
}

void LidarPubHandler::LivoxLidarPointCloudProcess(RawPacket & pkt) { // Livox Lidar 포인트 클라우드 처리 함수 정의
  switch (pkt.data_type) { // 데이터 타입에 따라 처리
    case kLivoxLidarCartesianCoordinateHighData:
      ProcessCartesianHighPoint(pkt);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      ProcessCartesianLowPoint(pkt);
      break;
    case kLivoxLidarSphericalCoordinateData:
      ProcessSphericalPoint(pkt);
      break;
    default:
      std::cout << "unknown data type: " << static_cast<int>(pkt.data_type)
                << " !!" << std::endl;
      break;
  }
}

void LidarPubHandler::SetLidarsExtParam(LidarExtParameter lidar_param) { // Lidar 외부 매개변수 설정 함수 정의
  if (is_set_extrinsic_params_) { // 외부 매개변수가 이미 설정된 경우
    return;
  }
  extrinsic_.trans[0] = lidar_param.param.x; // 변환 X 설정
  extrinsic_.trans[1] = lidar_param.param.y; // 변환 Y 설정
  extrinsic_.trans[2] = lidar_param.param.z; // 변환 Z 설정

  double cos_roll = cos(static_cast<double>(lidar_param.param.roll * PI / 180.0)); // 롤 코사인 계산
  double cos_pitch = cos(static_cast<double>(lidar_param.param.pitch * PI / 180.0)); // 피치 코사인 계산
  double cos_yaw = cos(static_cast<double>(lidar_param.param.yaw * PI / 180.0)); // 요 코사인 계산
  double sin_roll = sin(static_cast<double>(lidar_param.param.roll * PI / 180.0)); // 롤 사인 계산
  double sin_pitch = sin(static_cast<double>(lidar_param.param.pitch * PI / 180.0)); // 피치 사인 계산
  double sin_yaw = sin(static_cast<double>(lidar_param.param.yaw * PI / 180.0)); // 요 사인 계산

  extrinsic_.rotation[0][0] = cos_pitch * cos_yaw;
  extrinsic_.rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  extrinsic_.rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  extrinsic_.rotation[1][0] = cos_pitch * sin_yaw;
  extrinsic_.rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  extrinsic_.rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  extrinsic_.rotation[2][0] = -sin_pitch;
  extrinsic_.rotation[2][1] = sin_roll * cos_pitch;
  extrinsic_.rotation[2][2] = cos_roll * cos_pitch;

  is_set_extrinsic_params_ = true; // 외부 매개변수 설정 플래그 설정
}

void LidarPubHandler::ProcessCartesianHighPoint(RawPacket & pkt) { // Cartesian 고해상도 포인트 처리 함수 정의
  LivoxLidarCartesianHighRawPoint* raw = (LivoxLidarCartesianHighRawPoint*)pkt.raw_data.data(); // Raw 데이터 포인터 캐스팅
  PointXyzlt point = {}; // PointXyzlt 객체 생성
  for (uint32_t i = 0; i < pkt.point_num; i++) { // 포인트 개수만큼 반복
    if (pkt.extrinsic_enable) { // 외부 매개변수가 활성화된 경우
      point.x = raw[i].x / 1000.0;
      point.y = raw[i].y / 1000.0;
      point.z = raw[i].z / 1000.0;
    } else { // 외부 매개변수가 비활성화된 경우
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 1000.0;
      point.y = (raw[i].x * extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 1000.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 1000.0;
    }
    point.intensity = raw[i].reflectivity; // 반사율 설정
    point.line = i % pkt.line_num; // 라인 번호 설정
    point.tag = raw[i].tag; // 태그 설정
    point.offset_time = pkt.time_stamp + i * pkt.point_interval; // 오프셋 타임 설정
    std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
    points_clouds_.push_back(point); // 포인트 클라우드에 추가
  }
}

void LidarPubHandler::ProcessCartesianLowPoint(RawPacket & pkt) { // Cartesian 저해상도 포인트 처리 함수 정의
  LivoxLidarCartesianLowRawPoint* raw = (LivoxLidarCartesianLowRawPoint*)pkt.raw_data.data(); // Raw 데이터 포인터 캐스팅
  PointXyzlt point = {}; // PointXyzlt 객체 생성
  for (uint32_t i = 0; i < pkt.point_num; i++) { // 포인트 개수만큼 반복
    if (pkt.extrinsic_enable) { // 외부 매개변수가 활성화된 경우
      point.x = raw[i].x / 100.0;
      point.y = raw[i].y / 100.0;
      point.z = raw[i].z / 100.0;
    } else { // 외부 매개변수가 비활성화된 경우
      point.x = (raw[i].x * extrinsic_.rotation[0][0] +
                raw[i].y * extrinsic_.rotation[0][1] +
                raw[i].z * extrinsic_.rotation[0][2] + extrinsic_.trans[0]) / 100.0;
      point.y = (raw[i].x * extrinsic_.rotation[1][0] +
                raw[i].y * extrinsic_.rotation[1][1] +
                raw[i].z * extrinsic_.rotation[1][2] + extrinsic_.trans[1]) / 100.0;
      point.z = (raw[i].x * extrinsic_.rotation[2][0] +
                raw[i].y * extrinsic_.rotation[2][1] +
                raw[i].z * extrinsic_.rotation[2][2] + extrinsic_.trans[2]) / 100.0;
    }
    point.intensity = raw[i].reflectivity; // 반사율 설정
    point.line = i % pkt.line_num; // 라인 번호 설정
    point.tag = raw[i].tag; // 태그 설정
    point.offset_time = pkt.time_stamp + i * pkt.point_interval; // 오프셋 타임 설정
    std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
    points_clouds_.push_back(point); // 포인트 클라우드에 추가
  }
}

void LidarPubHandler::ProcessSphericalPoint(RawPacket& pkt) { // Spherical 포인트 처리 함수 정의
  LivoxLidarSpherPoint* raw = (LivoxLidarSpherPoint*)pkt.raw_data.data(); // Raw 데이터 포인터 캐스팅
  PointXyzlt point = {}; // PointXyzlt 객체 생성
  for (uint32_t i = 0; i < pkt.point_num; i++) { // 포인트 개수만큼 반복
    double radius = raw[i].depth / 1000.0; // 반지름 계산
    double theta = raw[i].theta / 100.0 / 180 * PI; // 세타 계산
    double phi = raw[i].phi / 100.0 / 180 * PI; // 파이 계산
    double src_x = radius * sin(theta) * cos(phi); // X 좌표 계산
    double src_y = radius * sin(theta) * sin(phi); // Y 좌표 계산
    double src_z = radius * cos(theta); // Z 좌표 계산
    if (pkt.extrinsic_enable) { // 외부 매개변수가 활성화된 경우
      point.x = src_x;
      point.y = src_y;
      point.z = src_z;
    } else { // 외부 매개변수가 비활성화된 경우
      point.x = src_x * extrinsic_.rotation[0][0] +
                src_y * extrinsic_.rotation[0][1] +
                src_z * extrinsic_.rotation[0][2] + (extrinsic_.trans[0] / 1000.0);
      point.y = src_x * extrinsic_.rotation[1][0] +
                src_y * extrinsic_.rotation[1][1] +
                src_z * extrinsic_.rotation[1][2] + (extrinsic_.trans[1] / 1000.0);
      point.z = src_x * extrinsic_.rotation[2][0] +
                src_y * extrinsic_.rotation[2][1] +
                src_z * extrinsic_.rotation[2][2] + (extrinsic_.trans[2] / 1000.0);
    }

    point.intensity = raw[i].reflectivity; // 반사율 설정
    point.line = i % pkt.line_num; // 라인 번호 설정
    point.tag = raw[i].tag; // 태그 설정
    point.offset_time = pkt.time_stamp + i * pkt.point_interval; // 오프셋 타임 설정
    std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스 잠금
    points_clouds_.push_back(point); // 포인트 클라우드에 추가
  }
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
