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



#include "lidar_common_callback.h" // "lidar_common_callback.h" 헤더 파일 포함

#include "../lds_lidar.h" // 상위 디렉토리의 "lds_lidar.h" 헤더 파일 포함

#include <string> // 문자열 처리를 위한 string 라이브러리 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// LidarCommonCallback 클래스의 OnLidarPointClounCb 함수 정의
void LidarCommonCallback::OnLidarPointClounCb(PointFrame* frame, void* client_data) {
  if (frame == nullptr) { // frame이 nullptr인지 확인
    printf("LidarPointCloudCb frame is nullptr.\n"); // frame이 nullptr이면 메시지 출력
    return; // 함수 종료
  }

  if (client_data == nullptr) { // client_data가 nullptr인지 확인
    printf("Lidar point cloud cb failed, client data is nullptr.\n"); // client_data가 nullptr이면 메시지 출력
    return; // 함수 종료
  }

  if (frame->lidar_num ==0) { // frame의 lidar_num이 0인지 확인
    printf("LidarPointCloudCb lidar_num:%u.\n", frame->lidar_num); // lidar_num이 0이면 메시지 출력
    return; // 함수 종료
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data); // client_data를 LdsLidar 타입으로 캐스팅
  
  //printf("Lidar point cloud, lidar_num:%u.\n", frame->lidar_num); // 디버그용 메시지 (현재는 주석 처리됨)

  lds_lidar->StoragePointData(frame); // LdsLidar 객체의 StoragePointData 함수 호출
}

// LidarCommonCallback 클래스의 LidarImuDataCallback 함수 정의
void LidarCommonCallback::LidarImuDataCallback(ImuData* imu_data, void *client_data) {
  if (imu_data == nullptr) { // imu_data가 nullptr인지 확인
    printf("Imu data is nullptr.\n"); // imu_data가 nullptr이면 메시지 출력
    return; // 함수 종료
  }
  if (client_data == nullptr) { // client_data가 nullptr인지 확인
    printf("Lidar point cloud cb failed, client data is nullptr.\n"); // client_data가 nullptr이면 메시지 출력
    return; // 함수 종료
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data); // client_data를 LdsLidar 타입으로 캐스팅
  lds_lidar->StorageImuData(imu_data); // LdsLidar 객체의 StorageImuData 함수 호출
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
