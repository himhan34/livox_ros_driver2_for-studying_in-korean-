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
#ifndef LIVOX_ROS_DRIVER2_INCLUDE_H_ // LIVOX_ROS_DRIVER2_INCLUDE_H_가 정의되지 않았을 때
#define LIVOX_ROS_DRIVER2_INCLUDE_H_ // LIVOX_ROS_DRIVER2_INCLUDE_H_를 정의

#define LIVOX_ROS_DRIVER2_VER_MAJOR 1 // 주 버전 번호 정의
#define LIVOX_ROS_DRIVER2_VER_MINOR 0 // 부 버전 번호 정의
#define LIVOX_ROS_DRIVER2_VER_PATCH 0 // 패치 버전 번호 정의

#define GET_STRING(n) GET_STRING_DIRECT(n) // 문자열 변환 매크로 정의
#define GET_STRING_DIRECT(n) #n // 문자열 변환 매크로 정의

#define LIVOX_ROS_DRIVER2_VERSION_STRING                      \
  GET_STRING(LIVOX_ROS_DRIVER2_VER_MAJOR)                     \
  "." GET_STRING(LIVOX_ROS_DRIVER2_VER_MINOR) "." GET_STRING( \
      LIVOX_ROS_DRIVER2_VER_PATCH) // 버전 문자열 정의

#endif  // LIVOX_ROS_DRIVER2_INCLUDE_H_ // LIVOX_ROS_DRIVER2_INCLUDE_H_ 종료
