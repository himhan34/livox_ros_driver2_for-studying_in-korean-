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

#include "lddc.h" // "lddc.h" 헤더 파일 포함
#include "comm/ldq.h" // "ldq.h" 헤더 파일 포함
#include "comm/comm.h" // "comm.h" 헤더 파일 포함

#include <inttypes.h> // int형 정수의 포맷 지정자를 제공
#include <iostream> // 입출력 스트림 제공
#include <iomanip> // 입출력 조작자 제공
#include <math.h> // 수학 함수 제공
#include <stdint.h> // 고정 폭 정수형 제공

#include "include/ros_headers.h" // ROS 헤더 파일 포함

#include "driver_node.h" // "driver_node.h" 헤더 파일 포함
#include "lds_lidar.h" // "lds_lidar.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

/** Lidar Data Distribute Control--------------------------------------------*/
#ifdef BUILDING_ROS1 // ROS1 빌드 시
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type, 
           double frq, std::string &frame_id, bool lidar_bag, bool imu_bag)
    : transfer_format_(format), // transfer_format_ 멤버 초기화
      use_multi_topic_(multi_topic), // use_multi_topic_ 멤버 초기화
      data_src_(data_src), // data_src_ 멤버 초기화
      output_type_(output_type), // output_type_ 멤버 초기화
      publish_frq_(frq), // publish_frq_ 멤버 초기화
      frame_id_(frame_id), // frame_id_ 멤버 초기화
      enable_lidar_bag_(lidar_bag), // enable_lidar_bag_ 멤버 초기화
      enable_imu_bag_(imu_bag) { // enable_imu_bag_ 멤버 초기화
  publish_period_ns_ = kNsPerSecond / publish_frq_; // 퍼블리시 주기 계산
  lds_ = nullptr; // lds 초기화
  memset(private_pub_, 0, sizeof(private_pub_)); // private_pub_ 배열 초기화
  memset(private_imu_pub_, 0, sizeof(private_imu_pub_)); // private_imu_pub_ 배열 초기화
  global_pub_ = nullptr; // global_pub_ 초기화
  global_imu_pub_ = nullptr; // global_imu_pub_ 초기화
  cur_node_ = nullptr; // cur_node_ 초기화
  bag_ = nullptr; // bag_ 초기화
}
#elif defined BUILDING_ROS2 // ROS2 빌드 시
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type, 
           double frq, std::string &frame_id)
    : transfer_format_(format), // transfer_format_ 멤버 초기화
      use_multi_topic_(multi_topic), // use_multi_topic_ 멤버 초기화
      data_src_(data_src), // data_src_ 멤버 초기화
      output_type_(output_type), // output_type_ 멤버 초기화
      publish_frq_(frq), // publish_frq_ 멤버 초기화
      frame_id_(frame_id) { // frame_id_ 멤버 초기화
  publish_period_ns_ = kNsPerSecond / publish_frq_; // 퍼블리시 주기 계산
  lds_ = nullptr; // lds 초기화
#if 0
  bag_ = nullptr; // bag_ 초기화
#endif
}
#endif

Lddc::~Lddc() { // Lddc 소멸자
#ifdef BUILDING_ROS1 // ROS1 빌드 시
  if (global_pub_) { 
    delete global_pub_; // global_pub_ 삭제
  }

  if (global_imu_pub_) {
    delete global_imu_pub_; // global_imu_pub_ 삭제
  }
#endif

  PrepareExit(); // 종료 준비 함수 호출

#ifdef BUILDING_ROS1 // ROS1 빌드 시
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_pub_[i]) {
      delete private_pub_[i]; // private_pub_ 배열 요소 삭제
    }
  }

  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    if (private_imu_pub_[i]) {
      delete private_imu_pub_[i]; // private_imu_pub_ 배열 요소 삭제
    }
  }
#endif
  std::cout << "lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl; // 파괴 메시지 출력
}

int Lddc::RegisterLds(Lds *lds) { // Lds 등록 함수
  if (lds_ == nullptr) { // lds_가 초기화되지 않았을 경우
    lds_ = lds; // lds_에 lds 설정
    return 0; // 성공 시 0 반환
  } else {
    return -1; // 실패 시 -1 반환
  }
}

void Lddc::DistributePointCloudData(void) { // 포인트 클라우드 데이터 분배 함수
  if (!lds_) { // lds_가 등록되지 않았을 경우
    std::cout << "lds is not registered" << std::endl; // 등록되지 않았음을 알림
    return; // 함수 종료
  }
  if (lds_->IsRequestExit()) { // lds_가 종료 요청되었을 경우
    std::cout << "DistributePointCloudData is RequestExit" << std::endl; // 종료 요청되었음을 알림
    return; // 함수 종료
  }
  
  lds_->pcd_semaphore_.Wait(); // 세마포어 대기
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) { // lidar_count_ 만큼 반복
    uint32_t lidar_id = i; // 현재 lidar ID 설정
    LidarDevice *lidar = &lds_->lidars_[lidar_id]; // 현재 LidarDevice 가져오기
    LidarDataQueue *p_queue = &lidar->data; // Lidar 데이터 큐 가져오기
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) { // 연결 상태가 샘플링 상태가 아니거나 큐가 없을 경우
      continue; // 다음 반복으로 넘어감
    }
    PollingLidarPointCloudData(lidar_id, lidar); // 포인트 클라우드 데이터 폴링
  }
}

void Lddc::DistributeImuData(void) { // IMU 데이터 분배 함수
  if (!lds_) { // lds_가 등록되지 않았을 경우
    std::cout << "lds is not registered" << std::endl; // 등록되지 않았음을 알림
    return; // 함수 종료
  }
  if (lds_->IsRequestExit()) { // lds_가 종료 요청되었을 경우
    std::cout << "DistributeImuData is RequestExit" << std::endl; // 종료 요청되었음을 알림
    return; // 함수 종료
  }
  
  lds_->imu_semaphore_.Wait(); // 세마포어 대기
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) { // lidar_count_ 만큼 반복
    uint32_t lidar_id = i; // 현재 lidar ID 설정
    LidarDevice *lidar = &lds_->lidars_[lidar_id]; // 현재 LidarDevice 가져오기
    LidarImuDataQueue *p_queue = &lidar->imu_data; // Lidar IMU 데이터 큐 가져오기
    if ((kConnectStateSampling != lidar->connect_state) || (p_queue == nullptr)) { // 연결 상태가 샘플링 상태가 아니거나 큐가 없을 경우
      continue; // 다음 반복으로 넘어감
    }
    PollingLidarImuData(lidar_id, lidar); // IMU 데이터 폴링
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) { // 포인트 클라우드 데이터 폴링 함수
  LidarDataQueue *p_queue = &lidar->data; // Lidar 데이터 큐 가져오기
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) { // 큐가 없거나 저장 패킷이 없을 경우
    return; // 함수 종료
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) { // 종료 요청되지 않았고 큐가 비어있지 않을 경우
    if (kPointCloud2Msg == transfer_format_) { // 전송 포맷이 PointCloud2일 경우
      PublishPointcloud2(p_queue, index); // PointCloud2 포맷으로 퍼블리시
    } else if (kLivoxCustomMsg == transfer_format_) { // 전송 포맷이 Livox 커스텀일 경우
      PublishCustomPointcloud(p_queue, index); // Livox 커스텀 포맷으로 퍼블리시
    } else if (kPclPxyziMsg == transfer_format_) { // 전송 포맷이 PCL일 경우
      PublishPclMsg(p_queue, index); // PCL 포맷으로 퍼블리시
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar) { // IMU 데이터 폴링 함수
  LidarImuDataQueue& p_queue = lidar->imu_data; // Lidar IMU 데이터 큐 가져오기
  while (!lds_->IsRequestExit() && !p_queue.Empty()) { // 종료 요청되지 않았고 큐가 비어있지 않을 경우
    PublishImuData(p_queue, index); // IMU 데이터 퍼블리시
  }
}

void Lddc::PrepareExit(void) { // 종료 준비 함수
#ifdef BUILDING_ROS1 // ROS1 빌드 시
  if (bag_) { // bag_이 존재할 경우
    DRIVER_INFO(*cur_node_, "Waiting to save the bag file!"); // bag 파일 저장 대기
    bag_->close(); // bag 파일 닫기
    DRIVER_INFO(*cur_node_, "Save the bag file successfully!"); // bag 파일 저장 성공
    bag_ = nullptr; // bag_ 초기화
  }
#endif
  if (lds_) { // lds_가 존재할 경우
    lds_->PrepareExit(); // lds 종료 준비
    lds_ = nullptr; // lds_ 초기화
  }
}

void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) { // PointCloud2 퍼블리시 함수
  while(!QueueIsEmpty(queue)) { // 큐가 비어있지 않을 경우
    StoragePacket pkg; // StoragePacket 객체 생성
    QueuePop(queue, &pkg); // 큐에서 패킷 팝
    if (pkg.points.empty()) { // 패킷의 포인트가 비어있을 경우
      printf("Publish point cloud2 failed, the pkg points is empty.\n"); // 퍼블리시 실패 메시지 출력
      continue; // 다음 반복으로 넘어감
    }

    PointCloud2 cloud; // PointCloud2 객체 생성
    uint64_t timestamp = 0; // 타임스탬프 초기화
    InitPointcloud2Msg(pkg, cloud, timestamp); // PointCloud2 메시지 초기화
    PublishPointcloud2Data(index, timestamp, cloud); // PointCloud2 데이터 퍼블리시
  }
}

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) { // 커스텀 포인트 클라우드 퍼블리시 함수
  while(!QueueIsEmpty(queue)) { // 큐가 비어있지 않을 경우
    StoragePacket pkg; // StoragePacket 객체 생성
    QueuePop(queue, &pkg); // 큐에서 패킷 팝
    if (pkg.points.empty()) { // 패킷의 포인트가 비어있을 경우
      printf("Publish custom point cloud failed, the pkg points is empty.\n"); // 퍼블리시 실패 메시지 출력
      continue; // 다음 반복으로 넘어감
    }

    CustomMsg livox_msg; // CustomMsg 객체 생성
    InitCustomMsg(livox_msg, pkg, index); // CustomMsg 초기화
    FillPointsToCustomMsg(livox_msg, pkg); // 패킷의 포인트를 CustomMsg에 채움
    PublishCustomPointData(livox_msg, index); // 커스텀 포인트 데이터 퍼블리시
  }
}

/* for pcl::pxyzi */
void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index) { // PCL 포인트 클라우드 퍼블리시 함수
#ifdef BUILDING_ROS2 // ROS2 빌드 시
  static bool first_log = true; // 첫 로그 여부 확인
  if (first_log) { // 첫 로그일 경우
    std::cout << "error: message type 'pcl::PointCloud' is NOT supported in ROS2, " // 오류 메시지 출력
              << "please modify the 'xfer_format' field in the launch file"
              << std::endl;
  }
  first_log = false; // 첫 로그 플래그 설정
  return; // 함수 종료
#endif
  while(!QueueIsEmpty(queue)) { // 큐가 비어있지 않을 경우
    StoragePacket pkg; // StoragePacket 객체 생성
    QueuePop(queue, &pkg); // 큐에서 패킷 팝
    if (pkg.points.empty()) { // 패킷의 포인트가 비어있을 경우
      printf("Publish point cloud failed, the pkg points is empty.\n"); // 퍼블리시 실패 메시지 출력
      continue; // 다음 반복으로 넘어감
    }

    PointCloud cloud; // PointCloud 객체 생성
    uint64_t timestamp = 0; // 타임스탬프 초기화
    InitPclMsg(pkg, cloud, timestamp); // PCL 메시지 초기화
    FillPointsToPclMsg(pkg, cloud); // 패킷의 포인트를 PCL 메시지에 채움
    PublishPclData(index, timestamp, cloud); // PCL 데이터 퍼블리시
  }
  return; // 함수 종료
}

void Lddc::InitPointcloud2MsgHeader(PointCloud2& cloud) { // PointCloud2 메시지 헤더 초기화 함수
  cloud.header.frame_id.assign(frame_id_); // 헤더 프레임 ID 설정
  cloud.height = 1; // 높이 설정
  cloud.width = 0; // 너비 설정
  cloud.fields.resize(7); // 필드 크기 설정
  cloud.fields[0].offset = 0; // 필드 0의 오프셋 설정
  cloud.fields[0].name = "x"; // 필드 0의 이름 설정
  cloud.fields[0].count = 1; // 필드 0의 카운트 설정
  cloud.fields[0].datatype = PointField::FLOAT32; // 필드 0의 데이터 타입 설정
  cloud.fields[1].offset = 4; // 필드 1의 오프셋 설정
  cloud.fields[1].name = "y"; // 필드 1의 이름 설정
  cloud.fields[1].count = 1; // 필드 1의 카운트 설정
  cloud.fields[1].datatype = PointField::FLOAT32; // 필드 1의 데이터 타입 설정
  cloud.fields[2].offset = 8; // 필드 2의 오프셋 설정
  cloud.fields[2].name = "z"; // 필드 2의 이름 설정
  cloud.fields[2].count = 1; // 필드 2의 카운트 설정
  cloud.fields[2].datatype = PointField::FLOAT32; // 필드 2의 데이터 타입 설정
  cloud.fields[3].offset = 12; // 필드 3의 오프셋 설정
  cloud.fields[3].name = "intensity"; // 필드 3의 이름 설정
  cloud.fields[3].count = 1; // 필드 3의 카운트 설정
  cloud.fields[3].datatype = PointField::FLOAT32; // 필드 3의 데이터 타입 설정
  cloud.fields[4].offset = 16; // 필드 4의 오프셋 설정
  cloud.fields[4].name = "tag"; // 필드 4의 이름 설정
  cloud.fields[4].count = 1; // 필드 4의 카운트 설정
  cloud.fields[4].datatype = PointField::UINT8; // 필드 4의 데이터 타입 설정
  cloud.fields[5].offset = 17; // 필드 5의 오프셋 설정
  cloud.fields[5].name = "line"; // 필드 5의 이름 설정
  cloud.fields[5].count = 1; // 필드 5의 카운트 설정
  cloud.fields[5].datatype = PointField::UINT8; // 필드 5의 데이터 타입 설정
  cloud.fields[6].offset = 18; // 필드 6의 오프셋 설정
  cloud.fields[6].name = "timestamp"; // 필드 6의 이름 설정
  cloud.fields[6].count = 1; // 필드 6의 카운트 설정
  cloud.fields[6].datatype = PointField::FLOAT64; // 필드 6의 데이터 타입 설정
  cloud.point_step = sizeof(LivoxPointXyzrtlt); // 포인트 스텝 설정
}


void Lddc::InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp) {
  InitPointcloud2MsgHeader(cloud); // PointCloud2 메시지 헤더 초기화

  cloud.point_step = sizeof(LivoxPointXyzrtlt); // 포인트 스텝 크기 설정

  cloud.width = pkg.points_num; // 포인트 개수 설정
  cloud.row_step = cloud.width * cloud.point_step; // 행의 크기 설정

  cloud.is_bigendian = false; // 엔디안 설정 (작은 엔디안)
  cloud.is_dense = true; // 밀집 플래그 설정

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time; // 타임스탬프 설정
  }

#ifdef BUILDING_ROS1
  cloud.header.stamp = ros::Time(timestamp / 1000000000.0); // ROS1 타임스탬프 설정
#elif defined BUILDING_ROS2
  cloud.header.stamp = rclcpp::Time(timestamp); // ROS2 타임스탬프 설정
#endif

  std::vector<LivoxPointXyzrtlt> points; // 포인트 벡터 생성
  for (size_t i = 0; i < pkg.points_num; ++i) {
    LivoxPointXyzrtlt point; // 포인트 객체 생성
    point.x = pkg.points[i].x; // x 좌표 설정
    point.y = pkg.points[i].y; // y 좌표 설정
    point.z = pkg.points[i].z; // z 좌표 설정
    point.reflectivity = pkg.points[i].intensity; // 반사 강도 설정
    point.tag = pkg.points[i].tag; // 태그 설정
    point.line = pkg.points[i].line; // 라인 설정
    point.timestamp = static_cast<double>(pkg.points[i].offset_time); // 타임스탬프 설정
    points.push_back(std::move(point)); // 포인트 벡터에 추가
  }
  cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt)); // 데이터 크기 조정
  memcpy(cloud.data.data(), points.data(), pkg.points_num * sizeof(LivoxPointXyzrtlt)); // 데이터 복사
}

void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp, const PointCloud2& cloud) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index); // 현재 퍼블리셔 가져오기
#elif defined BUILDING_ROS2
  Publisher<PointCloud2>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<PointCloud2>>(GetCurrentPublisher(index)); // 현재 퍼블리셔 가져오기
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud); // ROS에 퍼블리시
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), cloud); // Bag 파일에 기록
    }
#endif
  }
}

void Lddc::InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index) {
  livox_msg.header.frame_id.assign(frame_id_); // 프레임 ID 설정

#ifdef BUILDING_ROS1
  static uint32_t msg_seq = 0; // 메시지 시퀀스 초기화
  livox_msg.header.seq = msg_seq; // 시퀀스 설정
  ++msg_seq; // 시퀀스 증가
#endif

  uint64_t timestamp = 0;
  if (!pkg.points.empty()) {
    timestamp = pkg.base_time; // 타임스탬프 설정
  }
  livox_msg.timebase = timestamp; // 타임베이스 설정

#ifdef BUILDING_ROS1
  livox_msg.header.stamp = ros::Time(timestamp / 1000000000.0); // ROS1 타임스탬프 설정
#elif defined BUILDING_ROS2
  livox_msg.header.stamp = rclcpp::Time(timestamp); // ROS2 타임스탬프 설정
#endif

  livox_msg.point_num = pkg.points_num; // 포인트 개수 설정
  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg.lidar_id = lds_->lidars_[index].handle; // Lidar ID 설정
  } else {
    printf("Init custom msg lidar id failed, the index:%u.\n", index); // 오류 메시지 출력
    livox_msg.lidar_id = 0; // Lidar ID 초기화
  }
}

void Lddc::FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg) {
  uint32_t points_num = pkg.points_num; // 포인트 개수 설정
  const std::vector<PointXyzlt>& points = pkg.points; // 포인트 벡터 가져오기
  for (uint32_t i = 0; i < points_num; ++i) {
    CustomPoint point; // 커스텀 포인트 객체 생성
    point.x = points[i].x; // x 좌표 설정
    point.y = points[i].y; // y 좌표 설정
    point.z = points[i].z; // z 좌표 설정
    point.reflectivity = points[i].intensity; // 반사 강도 설정
    point.tag = points[i].tag; // 태그 설정
    point.line = points[i].line; // 라인 설정
    point.offset_time = static_cast<uint32_t>(points[i].offset_time - pkg.base_time); // 오프셋 타임 설정

    livox_msg.points.push_back(std::move(point)); // 커스텀 포인트 벡터에 추가
  }
}

void Lddc::PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index); // 현재 퍼블리셔 가져오기
#elif defined BUILDING_ROS2
  Publisher<CustomMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<CustomMsg>>(GetCurrentPublisher(index)); // 현재 퍼블리셔 가져오기
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(livox_msg); // ROS에 퍼블리시
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(livox_msg.timebase / 1000000000.0), livox_msg); // Bag 파일에 기록
    }
#endif
  }
}

void Lddc::InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp) {
#ifdef BUILDING_ROS1
  cloud.header.frame_id.assign(frame_id_); // 프레임 ID 설정
  cloud.height = 1; // 높이 설정
  cloud.width = pkg.points_num; // 포인트 개수 설정

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time; // 타임스탬프 설정
  }
  cloud.header.stamp = timestamp / 1000.0; // PCL ROS 타임스탬프 설정
#elif defined BUILDING_ROS2
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" 
            << std::endl; // ROS2에서 PCL 지원 안 함 경고 메시지 출력
#endif
  return;
}

void Lddc::FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg) {
#ifdef BUILDING_ROS1
  if (pkg.points.empty()) {
    return; // 포인트가 비어있을 경우 함수 종료
  }

  uint32_t points_num = pkg.points_num; // 포인트 개수 설정
  const std::vector<PointXyzlt>& points = pkg.points; // 포인트 벡터 가져오기
  for (uint32_t i = 0; i < points_num; ++i) {
    pcl::PointXYZI point; // PCL 포인트 객체 생성
    point.x = points[i].x; // x 좌표 설정
    point.y = points[i].y; // y 좌표 설정
    point.z = points[i].z; // z 좌표 설정
    point.intensity = points[i].intensity; // 강도 설정

    pcl_msg.points.push_back(std::move(point)); // PCL 포인트 벡터에 추가
  }
#elif defined BUILDING_ROS2
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" 
            << std::endl; // ROS2에서 PCL 지원 안 함 경고 메시지 출력
#endif
  return;
}

void Lddc::PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud) {
#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = Lddc::GetCurrentPublisher(index); // 현재 퍼블리셔 가져오기
  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud); // ROS에 퍼블리시
  } else {
    if (bag_ && enable_lidar_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), cloud); // Bag 파일에 기록
    }
  }
#elif defined BUILDING_ROS2
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" 
            << std::endl; // ROS2에서 PCL 지원 안 함 경고 메시지 출력
#endif
  return;
}

void Lddc::InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp) {
  imu_msg.header.frame_id = "livox_frame"; // 프레임 ID 설정

  timestamp = imu_data.time_stamp; // 타임스탬프 설정
#ifdef BUILDING_ROS1
  imu_msg.header.stamp = ros::Time(timestamp / 1000000000.0); // ROS 타임스탬프 설정
#elif defined BUILDING_ROS2
  imu_msg.header.stamp = rclcpp::Time(timestamp); // ROS2 타임스탬프 설정
#endif

  imu_msg.angular_velocity.x = imu_data.gyro_x; // 각속도 x 설정
  imu_msg.angular_velocity.y = imu_data.gyro_y; // 각속도 y 설정
  imu_msg.angular_velocity.z = imu_data.gyro_z; // 각속도 z 설정
  imu_msg.linear_acceleration.x = imu_data.acc_x; // 선형가속도 x 설정
  imu_msg.linear_acceleration.y = imu_data.acc_y; // 선형가속도 y 설정
  imu_msg.linear_acceleration.z = imu_data.acc_z; // 선형가속도 z 설정
}

void Lddc::PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index) {
  ImuData imu_data; // IMU 데이터 객체 생성
  if (!imu_data_queue.Pop(imu_data)) {
    //printf("Publish imu data failed, imu data queue pop failed.\n");
    return; // IMU 데이터 큐 팝 실패 시 함수 종료
  }

  ImuMsg imu_msg; // IMU 메시지 객체 생성
  uint64_t timestamp; // 타임스탬프 생성
  InitImuMsg(imu_data, imu_msg, timestamp); // IMU 메시지 초기화

#ifdef BUILDING_ROS1
  PublisherPtr publisher_ptr = GetCurrentImuPublisher(index); // 현재 IMU 퍼블리셔 가져오기
#elif defined BUILDING_ROS2
  Publisher<ImuMsg>::SharedPtr publisher_ptr = std::dynamic_pointer_cast<Publisher<ImuMsg>>(GetCurrentImuPublisher(index)); // 현재 IMU 퍼블리셔 가져오기
#endif

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(imu_msg); // ROS에 퍼블리시
  } else {
#ifdef BUILDING_ROS1
    if (bag_ && enable_imu_bag_) {
      bag_->write(publisher_ptr->getTopic(), ros::Time(timestamp / 1000000000.0), imu_msg); // Bag 파일에 기록
    }
#endif
  }
}

#ifdef BUILDING_ROS2
std::shared_ptr<rclcpp::PublisherBase> Lddc::CreatePublisher(uint8_t msg_type,
    std::string &topic_name, uint32_t queue_size) {
    if (kPointCloud2Msg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format", topic_name.c_str()); // PointCloud2 포맷 사용 메시지 출력
      return cur_node_->create_publisher<PointCloud2>(topic_name, queue_size); // PointCloud2 퍼블리셔 생성
    } else if (kLivoxCustomMsg == msg_type) {
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format", topic_name.c_str()); // Livox 커스텀 포맷 사용 메시지 출력
      return cur_node_->create_publisher<CustomMsg>(topic_name, queue_size); // CustomMsg 퍼블리셔 생성
    }
#if 0
    else if (kPclPxyziMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format", topic_name.c_str());
      return cur_node_->create_publisher<PointCloud>(topic_name, queue_size); // PCL PointXYZI 퍼블리셔 생성
    }
#endif
    else if (kLivoxImuMsg == msg_type)  {
      DRIVER_INFO(*cur_node_,
          "%s publish use imu format", topic_name.c_str()); // IMU 포맷 사용 메시지 출력
      return cur_node_->create_publisher<ImuMsg>(topic_name, queue_size); // IMU 퍼블리셔 생성
    } else {
      PublisherPtr null_publisher(nullptr); // Null 퍼블리셔 생성
      return null_publisher;
    }
}
#endif

#ifdef BUILDING_ROS1
PublisherPtr Lddc::GetCurrentPublisher(uint8_t index) {
  ros::Publisher **pub = nullptr; // 퍼블리셔 포인터 초기화
  uint32_t queue_size = kMinEthPacketQueueSize; // 큐 크기 설정

  if (use_multi_topic_) {
    pub = &private_pub_[index]; // 멀티 토픽 사용 시 개별 퍼블리셔 설정
    queue_size = queue_size / 8; // 큐 크기 조정
  } else {
    pub = &global_pub_; // 단일 토픽 사용 시 글로벌 퍼블리셔 설정
    queue_size = queue_size * 8; // 큐 크기 조정
  }

  if (*pub == nullptr) {
    char name_str[48]; // 이름 문자열 배열 생성
    memset(name_str, 0, sizeof(name_str)); // 배열 초기화
    if (use_multi_topic_) {
      std::string ip_string = IpNumToString(lds_->lidars_[index].handle); // IP 문자열 변환
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               ReplacePeriodByUnderline(ip_string).c_str()); // 이름 문자열 설정
      DRIVER_INFO(*cur_node_, "Support multi topics."); // 멀티 토픽 지원 메시지 출력
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic."); // 단일 토픽 지원 메시지 출력
      snprintf(name_str, sizeof(name_str), "livox/lidar");
    }

    *pub = new ros::Publisher; // 새로운 퍼블리셔 생성
    if (kPointCloud2Msg == transfer_format_) {
      **pub =
          cur_node_->GetNode().advertise<sensor_msgs::PointCloud2>(name_str, queue_size); // PointCloud2 퍼블리셔 생성
      DRIVER_INFO(*cur_node_,
          "%s publish use PointCloud2 format, set ROS publisher queue size %d",
          name_str, queue_size); // PointCloud2 퍼블리셔 생성 메시지 출력
    } else if (kLivoxCustomMsg == transfer_format_) {
      **pub = cur_node_->GetNode().advertise<livox_ros_driver2::CustomMsg>(name_str,
                                                                queue_size); // CustomMsg 퍼블리셔 생성
      DRIVER_INFO(*cur_node_,
          "%s publish use livox custom format, set ROS publisher queue size %d",
          name_str, queue_size); // CustomMsg 퍼블리셔 생성 메시지 출력
    } else if (kPclPxyziMsg == transfer_format_) {
      **pub = cur_node_->GetNode().advertise<PointCloud>(name_str, queue_size); // PCL PointXYZI 퍼블리셔 생성
      DRIVER_INFO(*cur_node_,
          "%s publish use pcl PointXYZI format, set ROS publisher queue "
          "size %d",
          name_str, queue_size); // PCL PointXYZI 퍼블리셔 생성 메시지 출력
    }
  }

  return *pub;
}

PublisherPtr Lddc::GetCurrentImuPublisher(uint8_t handle) {
  ros::Publisher **pub = nullptr; // 퍼블리셔 포인터 초기화
  uint32_t queue_size = kMinEthPacketQueueSize; // 큐 크기 설정

  if (use_multi_topic_) {
    pub = &private_imu_pub_[handle]; // 멀티 토픽 사용 시 개별 IMU 퍼블리셔 설정
    queue_size = queue_size * 2; // 큐 크기 조정
  } else {
    pub = &global_imu_pub_; // 단일 토픽 사용 시 글로벌 IMU 퍼블리셔 설정
    queue_size = queue_size * 8; // 큐 크기 조정
  }

  if (*pub == nullptr) {
    char name_str[48]; // 이름 문자열 배열 생성
    memset(name_str, 0, sizeof(name_str)); // 배열 초기화
    if (use_multi_topic_) {
      DRIVER_INFO(*cur_node_, "Support multi topics."); // 멀티 토픽 지원 메시지 출력
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle); // IP 문자열 변환
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
               ReplacePeriodByUnderline(ip_string).c_str()); // 이름 문자열 설정
    } else {
      DRIVER_INFO(*cur_node_, "Support only one topic."); // 단일 토픽 지원 메시지 출력
      snprintf(name_str, sizeof(name_str), "livox/imu");
    }

    *pub = new ros::Publisher; // 새로운 퍼블리셔 생성
    **pub = cur_node_->GetNode().advertise<sensor_msgs::Imu>(name_str, queue_size); // IMU 퍼블리셔 생성
    DRIVER_INFO(*cur_node_, "%s publish imu data, set ROS publisher queue size %d", name_str,
             queue_size); // IMU 퍼블리셔 생성 메시지 출력
  }

  return *pub;
}
#elif defined BUILDING_ROS2
std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize; // 큐 크기 설정
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48]; // 이름 문자열 배열 생성
      memset(name_str, 0, sizeof(name_str)); // 배열 초기화

      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle); // IP 문자열 변환
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
          ReplacePeriodByUnderline(ip_string).c_str()); // 이름 문자열 설정
      std::string topic_name(name_str); // 토픽 이름 설정
      queue_size = queue_size * 2; // 큐 크기 조정
      private_pub_[handle] = CreatePublisher(transfer_format_, topic_name, queue_size); // 퍼블리셔 생성
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar"); // 토픽 이름 설정
      queue_size = queue_size * 8; // 큐 크기 조정
      global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size); // 글로벌 퍼블리셔 생성
    }
    return global_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase> Lddc::GetCurrentImuPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize; // 큐 크기 설정
  if (use_multi_topic_) {
    if (!private_imu_pub_[handle]) {
      char name_str[48]; // 이름 문자열 배열 생성
      memset(name_str, 0, sizeof(name_str)); // 배열 초기화
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle); // IP 문자열 변환
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
          ReplacePeriodByUnderline(ip_string).c_str()); // 이름 문자열 설정
      std::string topic_name(name_str); // 토픽 이름 설정
      queue_size = queue_size * 2; // 큐 크기 조정
      private_imu_pub_[handle] = CreatePublisher(kLivoxImuMsg, topic_name, queue_size); // IMU 퍼블리셔 생성
    }
    return private_imu_pub_[handle];
  } else {
    if (!global_imu_pub_) {
      std::string topic_name("livox/imu"); // 토픽 이름 설정
      queue_size = queue_size * 8; // 큐 크기 조정
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size); // 글로벌 IMU 퍼블리셔 생성
    }
    return global_imu_pub_;
  }
}
#endif

void Lddc::CreateBagFile(const std::string &file_name) {
#ifdef BUILDING_ROS1
  if (!bag_) {
    bag_ = new rosbag::Bag; // 새로운 Bag 파일 생성
    bag_->open(file_name, rosbag::bagmode::Write); // Bag 파일 열기
    DRIVER_INFO(*cur_node_, "Create bag file :%s!", file_name.c_str()); // Bag 파일 생성 메시지 출력
  }
#endif
}

}  // namespace livox_ros

























