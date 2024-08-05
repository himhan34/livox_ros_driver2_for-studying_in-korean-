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
#include "livox_lidar_callback.h" // "livox_lidar_callback.h" 헤더 파일 포함

#include <string> // 문자열 처리를 위한 string 라이브러리 포함
#include <thread> // 스레드 처리를 위한 thread 라이브러리 포함
#include <iostream> // 입출력을 위한 iostream 라이브러리 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// LivoxLidarCallback 클래스의 LidarInfoChangeCallback 함수 정의
void LivoxLidarCallback::LidarInfoChangeCallback(const uint32_t handle,
                                                 const LivoxLidarInfo* info,
                                                 void* client_data) {
  if (client_data == nullptr) { // client_data가 nullptr인지 확인
    std::cout << "lidar info change callback failed, client data is nullptr" << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅

  LidarDevice* lidar_device = GetLidarDevice(handle, client_data); // lidar_device 가져오기
  if (lidar_device == nullptr) { // lidar_device가 nullptr인지 확인
    std::cout << "found lidar not defined in the user-defined config, ip: " << IpNumToString(handle) << std::endl; // 오류 메시지 출력
    // lidar 장치 추가
    uint8_t index = 0;
    int8_t ret = lds_lidar->cache_index_.GetFreeIndex(kLivoxLidarType, handle, index); // 인덱스 가져오기
    if (ret != 0) { // 실패 여부 확인
      std::cout << "failed to add lidar device, lidar ip: " << IpNumToString(handle) << std::endl; // 오류 메시지 출력
      return; // 함수 종료
    }
    LidarDevice *p_lidar = &(lds_lidar->lidars_[index]); // 새로운 LidarDevice 가져오기
    p_lidar->lidar_type = kLivoxLidarType; // Lidar 타입 설정
  } else {
    // 사용자 정의 설정에 따라 lidar 설정
    const UserLivoxLidarConfig& config = lidar_device->livox_config;

    // lidar 장치의 set_bits를 수정하기 위한 잠금 설정
    {
      std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_);
      if (config.pcl_data_type != -1 ) { // PCL 데이터 타입 설정 여부 확인
        lidar_device->livox_config.set_bits |= kConfigDataType; // set_bits 설정
        SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                                LivoxLidarCallback::SetDataTypeCallback, lds_lidar); // PCL 데이터 타입 설정
        std::cout << "set pcl data type, handle: " << handle << ", data type: "
                  << static_cast<int32_t>(config.pcl_data_type) << std::endl; // 설정 메시지 출력
      }
      if (config.pattern_mode != -1) { // 패턴 모드 설정 여부 확인
        lidar_device->livox_config.set_bits |= kConfigScanPattern; // set_bits 설정
        SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                              LivoxLidarCallback::SetPatternModeCallback, lds_lidar); // 패턴 모드 설정
        std::cout << "set scan pattern, handle: " << handle << ", scan pattern: "
                  << static_cast<int32_t>(config.pattern_mode) << std::endl; // 설정 메시지 출력
      }
      if (config.blind_spot_set != -1) { // 블라인드 스팟 설정 여부 확인
        lidar_device->livox_config.set_bits |= kConfigBlindSpot; // set_bits 설정
        SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                              LivoxLidarCallback::SetBlindSpotCallback, lds_lidar); // 블라인드 스팟 설정

        std::cout << "set blind spot, handle: " << handle << ", blind spot distance: "
                  << config.blind_spot_set << std::endl; // 설정 메시지 출력
      }
      if (config.dual_emit_en != -1) { // 듀얼 방출 모드 설정 여부 확인
        lidar_device->livox_config.set_bits |= kConfigDualEmit; // set_bits 설정
        SetLivoxLidarDualEmit(handle, (config.dual_emit_en == 0 ? false : true),
                              LivoxLidarCallback::SetDualEmitCallback, lds_lidar); // 듀얼 방출 모드 설정
        std::cout << "set dual emit mode, handle: " << handle << ", enable dual emit: "
                  << static_cast<int32_t>(config.dual_emit_en) << std::endl; // 설정 메시지 출력
      }
    } // set_bits에 대한 잠금 해제

    // lidar에 외부 매개변수 설정
    LivoxLidarInstallAttitude attitude {
      config.extrinsic_param.roll,
      config.extrinsic_param.pitch,
      config.extrinsic_param.yaw,
      config.extrinsic_param.x,
      config.extrinsic_param.y,
      config.extrinsic_param.z
    };
    SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, lds_lidar); // 외부 매개변수 설정
  }

  std::cout << "begin to change work mode to 'Normal', handle: " << handle << std::endl; // 작업 모드 변경 시작 메시지 출력
  SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr); // 작업 모드 변경
  EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar); // IMU 데이터 활성화
  return;
}

// LivoxLidarCallback 클래스의 WorkModeChangedCallback 함수 정의
void LivoxLidarCallback::WorkModeChangedCallback(livox_status status,
                                                 uint32_t handle,
                                                 LivoxLidarAsyncControlResponse *response,
                                                 void *client_data) {
  if (status != kLivoxLidarStatusSuccess) { // 상태가 성공이 아닌 경우
    std::cout << "failed to change work mode, handle: " << handle << ", try again..."<< std::endl; // 오류 메시지 출력
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 1초 대기
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeChangedCallback, nullptr); // 작업 모드 재설정
    return; // 함수 종료
  }
  std::cout << "successfully change work mode, handle: " << handle << std::endl; // 성공 메시지 출력
  return;
}

// LivoxLidarCallback 클래스의 SetDataTypeCallback 함수 정의
void LivoxLidarCallback::SetDataTypeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data); // lidar_device 가져오기
  if (lidar_device == nullptr) { // lidar_device가 nullptr인지 확인
    std::cout << "failed to set data type since no lidar device found, handle: "
              << handle << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅

  if (status == kLivoxLidarStatusSuccess) { // 상태가 성공인 경우
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_); // 설정 변경을 위한 잠금 설정
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDataType)); // set_bits 비트 해제
    if (!lidar_device->livox_config.set_bits) { // 모든 비트가 해제된 경우
      lidar_device->connect_state = kConnectStateSampling; // 연결 상태를 샘플링으로 설정
    }
    std::cout << "successfully set data type, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits << std::endl; // 성공 메시지 출력
  } else if (status == kLivoxLidarStatusTimeout) { // 상태가 타임아웃인 경우
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarPclDataType(handle, static_cast<LivoxLidarPointDataType>(config.pcl_data_type),
                             LivoxLidarCallback::SetDataTypeCallback, client_data); // PCL 데이터 타입 재설정
    std::cout << "set data type timeout, handle: " << handle
              << ", try again..." << std::endl; // 재시도 메시지 출력
  } else {
    std::cout << "failed to set data type, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key << std::endl; // 실패 메시지 출력
  }
  return;
}

// LivoxLidarCallback 클래스의 SetPatternModeCallback 함수 정의
void LivoxLidarCallback::SetPatternModeCallback(livox_status status, uint32_t handle,
                                                LivoxLidarAsyncControlResponse *response,
                                                void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data); // lidar_device 가져오기
  if (lidar_device == nullptr) { // lidar_device가 nullptr인지 확인
    std::cout << "failed to set pattern mode since no lidar device found, handle: "
              << handle << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅

  if (status == kLivoxLidarStatusSuccess) { // 상태가 성공인 경우
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_); // 설정 변경을 위한 잠금 설정
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigScanPattern)); // set_bits 비트 해제
    if (!lidar_device->livox_config.set_bits) { // 모든 비트가 해제된 경우
      lidar_device->connect_state = kConnectStateSampling; // 연결 상태를 샘플링으로 설정
    }
    std::cout << "successfully set pattern mode, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits << std::endl; // 성공 메시지 출력
  } else if (status == kLivoxLidarStatusTimeout) { // 상태가 타임아웃인 경우
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarScanPattern(handle, static_cast<LivoxLidarScanPattern>(config.pattern_mode),
                             LivoxLidarCallback::SetPatternModeCallback, client_data); // 패턴 모드 재설정
    std::cout << "set pattern mode timeout, handle: " << handle
              << ", try again..." << std::endl; // 재시도 메시지 출력
  } else {
    std::cout << "failed to set pattern mode, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key << std::endl; // 실패 메시지 출력
  }
  return;
}

// LivoxLidarCallback 클래스의 SetBlindSpotCallback 함수 정의
void LivoxLidarCallback::SetBlindSpotCallback(livox_status status, uint32_t handle,
                                              LivoxLidarAsyncControlResponse *response,
                                              void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data); // lidar_device 가져오기
  if (lidar_device == nullptr) { // lidar_device가 nullptr인지 확인
    std::cout << "failed to set blind spot since no lidar device found, handle: "
              << handle << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅

  if (status == kLivoxLidarStatusSuccess) { // 상태가 성공인 경우
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_); // 설정 변경을 위한 잠금 설정
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigBlindSpot)); // set_bits 비트 해제
    if (!lidar_device->livox_config.set_bits) { // 모든 비트가 해제된 경우
      lidar_device->connect_state = kConnectStateSampling; // 연결 상태를 샘플링으로 설정
    }
    std::cout << "successfully set blind spot, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits << std::endl; // 성공 메시지 출력
  } else if (status == kLivoxLidarStatusTimeout) { // 상태가 타임아웃인 경우
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarBlindSpot(handle, config.blind_spot_set,
                           LivoxLidarCallback::SetBlindSpotCallback, client_data); // 블라인드 스팟 재설정
    std::cout << "set blind spot timeout, handle: " << handle
              << ", try again..." << std::endl; // 재시도 메시지 출력
  } else {
    std::cout << "failed to set blind spot, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key << std::endl; // 실패 메시지 출력
  }
  return;
}

// LivoxLidarCallback 클래스의 SetDualEmitCallback 함수 정의
void LivoxLidarCallback::SetDualEmitCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data); // lidar_device 가져오기
  if (lidar_device == nullptr) { // lidar_device가 nullptr인지 확인
    std::cout << "failed to set dual emit mode since no lidar device found, handle: "
              << handle << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅
  if (status == kLivoxLidarStatusSuccess) { // 상태가 성공인 경우
    std::lock_guard<std::mutex> lock(lds_lidar->config_mutex_); // 설정 변경을 위한 잠금 설정
    lidar_device->livox_config.set_bits &= ~((uint32_t)(kConfigDualEmit)); // set_bits 비트 해제
    if (!lidar_device->livox_config.set_bits) { // 모든 비트가 해제된 경우
      lidar_device->connect_state = kConnectStateSampling; // 연결 상태를 샘플링으로 설정
    }
    std::cout << "successfully set dual emit mode, handle: " << handle
              << ", set_bit: " << lidar_device->livox_config.set_bits << std::endl; // 성공 메시지 출력
  } else if (status == kLivoxLidarStatusTimeout) { // 상태가 타임아웃인 경우
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    SetLivoxLidarDualEmit(handle, config.dual_emit_en,
                          LivoxLidarCallback::SetDualEmitCallback, client_data); // 듀얼 방출 모드 재설정
    std::cout << "set dual emit mode timeout, handle: " << handle
              << ", try again..." << std::endl; // 재시도 메시지 출력
  } else {
    std::cout << "failed to set dual emit mode, handle: " << handle
              << ", return code: " << response->ret_code
              << ", error key: " << response->error_key << std::endl; // 실패 메시지 출력
  }
  return;
}

// LivoxLidarCallback 클래스의 SetAttitudeCallback 함수 정의
void LivoxLidarCallback::SetAttitudeCallback(livox_status status, uint32_t handle,
                                             LivoxLidarAsyncControlResponse *response,
                                             void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data); // lidar_device 가져오기
  if (lidar_device == nullptr) { // lidar_device가 nullptr인지 확인
    std::cout << "failed to set dual emit mode since no lidar device found, handle: "
              << handle << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅
  if (status == kLivoxLidarStatusSuccess) { // 상태가 성공인 경우
    std::cout << "successfully set lidar attitude, ip: " << IpNumToString(handle) << std::endl; // 성공 메시지 출력
  } else if (status == kLivoxLidarStatusTimeout) { // 상태가 타임아웃인 경우
    std::cout << "set lidar attitude timeout, ip: " << IpNumToString(handle)
              << ", try again..." << std::endl; // 재시도 메시지 출력
    const UserLivoxLidarConfig& config = lidar_device->livox_config;
    LivoxLidarInstallAttitude attitude {
      config.extrinsic_param.roll,
      config.extrinsic_param.pitch,
      config.extrinsic_param.yaw,
      config.extrinsic_param.x,
      config.extrinsic_param.y,
      config.extrinsic_param.z
    };
    SetLivoxLidarInstallAttitude(config.handle, &attitude,
                                 LivoxLidarCallback::SetAttitudeCallback, lds_lidar); // 외부 매개변수 재설정
  } else {
    std::cout << "failed to set lidar attitude, ip: " << IpNumToString(handle) << std::endl; // 실패 메시지 출력
  }
}

// LivoxLidarCallback 클래스의 EnableLivoxLidarImuDataCallback 함수 정의
void LivoxLidarCallback::EnableLivoxLidarImuDataCallback(livox_status status, uint32_t handle,
                                                         LivoxLidarAsyncControlResponse *response,
                                                         void *client_data) {
  LidarDevice* lidar_device =  GetLidarDevice(handle, client_data); // lidar_device 가져오기
  if (lidar_device == nullptr) { // lidar_device가 nullptr인지 확인
    std::cout << "failed to set pattern mode since no lidar device found, handle: "
              << handle << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }
  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅

  if (response == nullptr) { // response가 nullptr인지 확인
    std::cout << "failed to get response since no lidar IMU sensor found, handle: "
              << handle << std::endl; // 오류 메시지 출력
    return; // 함수 종료
  }

  if (status == kLivoxLidarStatusSuccess) { // 상태가 성공인 경우
    std::cout << "successfully enable Livox Lidar imu, ip: " << IpNumToString(handle) << std::endl; // 성공 메시지 출력
  } else if (status == kLivoxLidarStatusTimeout) { // 상태가 타임아웃인 경우
    std::cout << "enable Livox Lidar imu timeout, ip: " << IpNumToString(handle)
              << ", try again..." << std::endl; // 재시도 메시지 출력
    EnableLivoxLidarImuData(handle, LivoxLidarCallback::EnableLivoxLidarImuDataCallback, lds_lidar); // IMU 데이터 활성화 재설정
  } else {
    std::cout << "failed to enable Livox Lidar imu, ip: " << IpNumToString(handle) << std::endl; // 실패 메시지 출력
  }
}

// LivoxLidarCallback 클래스의 GetLidarDevice 함수 정의
LidarDevice* LivoxLidarCallback::GetLidarDevice(const uint32_t handle, void* client_data) {
  if (client_data == nullptr) { // client_data가 nullptr인지 확인
    std::cout << "failed to get lidar device, client data is nullptr" << std::endl; // 오류 메시지 출력
    return nullptr; // nullptr 반환
  }

  LdsLidar* lds_lidar = static_cast<LdsLidar*>(client_data); // client_data를 LdsLidar 타입으로 캐스팅
  uint8_t index = 0;
  int8_t ret = lds_lidar->cache_index_.GetIndex(kLivoxLidarType, handle, index); // 인덱스 가져오기
  if (ret != 0) { // 실패 여부 확인
    return nullptr; // nullptr 반환
  }

  return &(lds_lidar->lidars_[index]); // 인덱스에 해당하는 LidarDevice 반환
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
