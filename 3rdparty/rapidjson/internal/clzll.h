// Tencent is pleased to support the open source community by making RapidJSON
// available.
//
// Copyright (C) 2015 THL A29 Limited, a Tencent company, and Milo Yip. All
// rights reserved.
//
// Licensed under the MIT License (the "License"); you may not use this file
// except in compliance with the License. You may obtain a copy of the License
// at
//
// http://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
// License for the specific language governing permissions and limitations under
// the License.
#ifndef RAPIDJSON_CLZLL_H_
#define RAPIDJSON_CLZLL_H_  // 헤더 파일 중복 포함을 방지하기 위한 매크로 정의

#include "../rapidjson.h"  // rapidjson.h 파일 포함

#if defined(_MSC_VER)  // MSVC 컴파일러가 정의된 경우
#include <intrin.h>  // MSVC용 내장 함수 헤더 파일 포함
#if defined(_WIN64)  // 64비트 윈도우가 정의된 경우
#pragma intrinsic(_BitScanReverse64)  // 64비트 비트 스캔 역방향 intrinsic 함수 사용
#else
#pragma intrinsic(_BitScanReverse)  // 32비트 비트 스캔 역방향 intrinsic 함수 사용
#endif
#endif

RAPIDJSON_NAMESPACE_BEGIN  // RapidJSON 네임스페이스 시작
namespace internal {  // 내부 네임스페이스 시작

#if (defined(__GNUC__) && __GNUC__ >= 4) || \
    RAPIDJSON_HAS_BUILTIN(__builtin_clzll)  // GCC 버전 4 이상이거나 __builtin_clzll이 정의된 경우
#define RAPIDJSON_CLZLL __builtin_clzll  // __builtin_clzll을 RAPIDJSON_CLZLL로 정의
#else

inline uint32_t clzll(uint64_t x) {  // clzll 함수 정의
  // GCC에서 0을 __builtin_clzll에 전달하는 것은 UB이며, 소프트웨어 구현에서는 무한 루프가 발생
  RAPIDJSON_ASSERT(x != 0);  // x가 0이 아닌지 확인

#if defined(_MSC_VER)  // MSVC 컴파일러가 정의된 경우
  unsigned long r = 0;  // 결과를 저장할 변수 r 선언
#if defined(_WIN64)  // 64비트 윈도우가 정의된 경우
  _BitScanReverse64(&r, x);  // 64비트 비트 스캔 역방향 함수 호출
#else
  // 상위 32비트를 스캔
  if (_BitScanReverse(&r, static_cast<uint32_t>(x >> 32))) return 63 - (r + 32);

  // 하위 32비트를 스캔
  _BitScanReverse(&r, static_cast<uint32_t>(x & 0xFFFFFFFF));
#endif  // _WIN64

  return 63 - r;  // 결과 반환
#else
  uint32_t r;  // 결과를 저장할 변수 r 선언
  while (!(x & (static_cast<uint64_t>(1) << 63))) {  // x의 최상위 비트가 1이 아닐 때까지
    x <<= 1;  // x를 왼쪽으로 1비트 이동
    ++r;  // r을 1 증가
  }

  return r;  // 결과 반환
#endif  // _MSC_VER
}

#define RAPIDJSON_CLZLL RAPIDJSON_NAMESPACE::internal::clzll  // clzll 함수를 RAPIDJSON_CLZLL로 정의
#endif  // (defined(__GNUC__) && __GNUC__ >= 4) ||
        // RAPIDJSON_HAS_BUILTIN(__builtin_clzll)

}  // namespace internal  // 내부 네임스페이스 종료
RAPIDJSON_NAMESPACE_END  // RapidJSON 네임스페이스 종료

#endif  // RAPIDJSON_CLZLL_H_  // 헤더 파일 중복 포함 방지 매크로 종료
