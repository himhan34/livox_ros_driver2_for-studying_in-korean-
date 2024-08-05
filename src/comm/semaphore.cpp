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

#include "semaphore.h" // "semaphore.h" 헤더 파일 포함

namespace livox_ros { // livox_ros 네임스페이스 시작

// 세마포어 신호 함수
void Semaphore::Signal() {
  std::unique_lock<std::mutex> lock(mutex_); // 뮤텍스 잠금
  ++count_; // 카운트 증가
  cv_.notify_one(); // 조건 변수 알림
}

// 세마포어 대기 함수
void Semaphore::Wait() {
  std::unique_lock<std::mutex> lock(mutex_); // 뮤텍스 잠금
  cv_.wait(lock, [=] { return count_ > 0; }); // 카운트가 0보다 클 때까지 대기
  --count_; // 카운트 감소
}

} // namespace livox_ros // livox_ros 네임스페이스 끝
