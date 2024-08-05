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

#ifndef RAPIDJSON_ERROR_EN_H_ // RAPIDJSON_ERROR_EN_H_가 정의되지 않았을 때
#define RAPIDJSON_ERROR_EN_H_ // RAPIDJSON_ERROR_EN_H_를 정의

#include "error.h" // "error.h" 헤더 파일 포함

#ifdef __clang__ // 클랭 컴파일러를 사용할 때
RAPIDJSON_DIAG_PUSH // 진단 푸시
RAPIDJSON_DIAG_OFF(switch - enum) // switch - enum 경고 비활성화
RAPIDJSON_DIAG_OFF(covered - switch - default) // covered - switch - default 경고 비활성화
#endif

RAPIDJSON_NAMESPACE_BEGIN // rapidjson 네임스페이스 시작

//! Maps error code of parsing into error message.
/*!
    \ingroup RAPIDJSON_ERRORS
    \param parseErrorCode Error code obtained in parsing.
    \return the error message.
    \note User can make a copy of this function for localization.
        Using switch-case is safer for future modification of error codes.
*/
inline const RAPIDJSON_ERROR_CHARTYPE* GetParseError_En(ParseErrorCode parseErrorCode) {
  switch (parseErrorCode) {
    case kParseErrorNone:
      return RAPIDJSON_ERROR_STRING("No error."); // 오류 없음

    case kParseErrorDocumentEmpty:
      return RAPIDJSON_ERROR_STRING("The document is empty."); // 문서가 비어 있음
    case kParseErrorDocumentRootNotSingular:
      return RAPIDJSON_ERROR_STRING("The document root must not be followed by other values."); // 문서 루트 뒤에 다른 값이 올 수 없음

    case kParseErrorValueInvalid:
      return RAPIDJSON_ERROR_STRING("Invalid value."); // 유효하지 않은 값

    case kParseErrorObjectMissName:
      return RAPIDJSON_ERROR_STRING("Missing a name for object member."); // 객체 멤버의 이름이 없음
    case kParseErrorObjectMissColon:
      return RAPIDJSON_ERROR_STRING("Missing a colon after a name of object member."); // 객체 멤버 이름 뒤에 콜론이 없음
    case kParseErrorObjectMissCommaOrCurlyBracket:
      return RAPIDJSON_ERROR_STRING("Missing a comma or '}' after an object member."); // 객체 멤버 뒤에 쉼표 또는 '}'가 없음

    case kParseErrorArrayMissCommaOrSquareBracket:
      return RAPIDJSON_ERROR_STRING("Missing a comma or ']' after an array element."); // 배열 요소 뒤에 쉼표 또는 ']'가 없음

    case kParseErrorStringUnicodeEscapeInvalidHex:
      return RAPIDJSON_ERROR_STRING("Incorrect hex digit after \\u escape in string."); // 문자열에서 \u 이스케이프 뒤의 잘못된 16진수
    case kParseErrorStringUnicodeSurrogateInvalid:
      return RAPIDJSON_ERROR_STRING("The surrogate pair in string is invalid."); // 문자열의 대리 쌍이 유효하지 않음
    case kParseErrorStringEscapeInvalid:
      return RAPIDJSON_ERROR_STRING("Invalid escape character in string."); // 문자열에서 유효하지 않은 이스케이프 문자
    case kParseErrorStringMissQuotationMark:
      return RAPIDJSON_ERROR_STRING("Missing a closing quotation mark in string."); // 문자열에서 닫는 따옴표가 없음
    case kParseErrorStringInvalidEncoding:
      return RAPIDJSON_ERROR_STRING("Invalid encoding in string."); // 문자열에서 유효하지 않은 인코딩

    case kParseErrorNumberTooBig:
      return RAPIDJSON_ERROR_STRING("Number too big to be stored in double."); // double에 저장하기에 너무 큰 숫자
    case kParseErrorNumberMissFraction:
      return RAPIDJSON_ERROR_STRING("Miss fraction part in number."); // 숫자에서 소수 부분이 없음
    case kParseErrorNumberMissExponent:
      return RAPIDJSON_ERROR_STRING("Miss exponent in number."); // 숫자에서 지수 부분이 없음

    case kParseErrorTermination:
      return RAPIDJSON_ERROR_STRING("Terminate parsing due to Handler error."); // 핸들러 오류로 인해 구문 분석 종료
    case kParseErrorUnspecificSyntaxError:
      return RAPIDJSON_ERROR_STRING("Unspecific syntax error."); // 구체적이지 않은 구문 오류

    default:
      return RAPIDJSON_ERROR_STRING("Unknown error."); // 알 수 없는 오류
  }
}

RAPIDJSON_NAMESPACE_END // rapidjson 네임스페이스 끝

#ifdef __clang__ // 클랭 컴파일러를 사용할 때
RAPIDJSON_DIAG_POP // 진단 팝
#endif

#endif  // RAPIDJSON_ERROR_EN_H_ // RAPIDJSON_ERROR_EN_H_ 종료
