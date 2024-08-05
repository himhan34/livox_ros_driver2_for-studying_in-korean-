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

#ifndef LIVOX_ROS_DRIVER_SEMAPHORE_H_ // LIVOX_ROS_DRIVER_SEMAPHORE_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER_SEMAPHORE_H_ // LIVOX_ROS_DRIVER_SEMAPHORE_H_를 정의

#include <mutex> // 뮤텍스 사용을 위한 헤더 파일 포함
#include <condition_variable> // 조건 변수 사용을 위한 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

class Semaphore { // Semaphore 클래스 정의
 public:
  explicit Semaphore(int count = 0) : count_(count) { // 생성자, 초기 카운트 설정
  }
  void Signal(); // 세마포어 신호 함수
  void Wait(); // 세마포어 대기 함수
  int GetCount() { // 현재 카운트 반환 함수
    return count_;
  }

 private:
  std::mutex mutex_; // 뮤텍스
  std::condition_variable cv_; // 조건 변수
  volatile int count_; // 카운트
};

} // namespace livox_ros // livox_ros 네임스페이스 끝

#endif // LIVOX_ROS_DRIVER_SEMAPHORE_H_ // LIVOX_ROS_DRIVER_SEMAPHORE_H_ 종료
