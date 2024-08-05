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

#include "driver_node.h" // "driver_node.h" 헤더 파일 포함
#include "lddc.h" // "lddc.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// DriverNode 클래스의 GetNode 함수 정의
DriverNode& DriverNode::GetNode() noexcept {
  return *this; // 현재 객체를 반환
}

// DriverNode 클래스의 소멸자 정의
DriverNode::~DriverNode() {
  lddc_ptr_->lds_->RequestExit(); // LDDC의 LDS에 종료 요청
  exit_signal_.set_value(); // 종료 신호 설정
  pointclouddata_poll_thread_->join(); // 포인트 클라우드 데이터 폴링 스레드 종료 대기
  imudata_poll_thread_->join(); // IMU 데이터 폴링 스레드 종료 대기
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
