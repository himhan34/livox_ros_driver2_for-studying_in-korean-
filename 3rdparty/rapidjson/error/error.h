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

#ifndef RAPIDJSON_ERROR_ERROR_H_ // RAPIDJSON_ERROR_ERROR_H_가 정의되지 않았을 때
#define RAPIDJSON_ERROR_ERROR_H_ // RAPIDJSON_ERROR_ERROR_H_를 정의

#include "../rapidjson.h" // "rapidjson.h" 헤더 파일 포함

#ifdef __clang__ // 클랭 컴파일러를 사용할 때
RAPIDJSON_DIAG_PUSH // 진단 푸시
RAPIDJSON_DIAG_OFF(padded) // 패딩 경고 비활성화
#endif

/*! \file error.h */

/*! \defgroup RAPIDJSON_ERRORS RapidJSON error handling */

///////////////////////////////////////////////////////////////////////////////
// RAPIDJSON_ERROR_CHARTYPE

//! 에러 메시지의 문자 타입 정의
/*! \ingroup RAPIDJSON_ERRORS
    기본 문자 타입은 \c char 입니다.
    Windows에서는 유저가 이 매크로를 \c TCHAR로 정의하여 유니코드/비유니코드 설정을 모두 지원할 수 있습니다.
*/
#ifndef RAPIDJSON_ERROR_CHARTYPE
#define RAPIDJSON_ERROR_CHARTYPE char
#endif

///////////////////////////////////////////////////////////////////////////////
// RAPIDJSON_ERROR_STRING

//! 문자열 리터럴을 \ref RAPIDJSON_ERROR_CHARTYPE[]로 변환하는 매크로
/*! \ingroup RAPIDJSON_ERRORS
    기본적으로 이 변환 매크로는 아무 작업도 수행하지 않습니다.
    Windows에서는 유저가 이 매크로를 \c _T(x)로 정의하여 유니코드/비유니코드 설정을 모두 지원할 수 있습니다.
*/
#ifndef RAPIDJSON_ERROR_STRING
#define RAPIDJSON_ERROR_STRING(x) x
#endif

RAPIDJSON_NAMESPACE_BEGIN // rapidjson 네임스페이스 시작

///////////////////////////////////////////////////////////////////////////////
// ParseErrorCode

//! 파싱 에러 코드
/*! \ingroup RAPIDJSON_ERRORS
    \see GenericReader::Parse, GenericReader::GetParseErrorCode
*/
enum ParseErrorCode {
  kParseErrorNone = 0,  //!< 에러 없음.

  kParseErrorDocumentEmpty,            //!< 문서가 비어 있음.
  kParseErrorDocumentRootNotSingular,  //!< 문서 루트 뒤에 다른 값이 올 수 없음.

  kParseErrorValueInvalid,  //!< 유효하지 않은 값.

  kParseErrorObjectMissName,   //!< 객체 멤버의 이름이 없음.
  kParseErrorObjectMissColon,  //!< 객체 멤버 이름 뒤에 콜론이 없음.
  kParseErrorObjectMissCommaOrCurlyBracket,  //!< 객체 멤버 뒤에 쉼표 또는 '}'가 없음.

  kParseErrorArrayMissCommaOrSquareBracket,  //!< 배열 요소 뒤에 쉼표 또는 ']'가 없음.

  kParseErrorStringUnicodeEscapeInvalidHex,  //!< 문자열에서 \u 이스케이프 뒤의 잘못된 16진수.
  kParseErrorStringUnicodeSurrogateInvalid,  //!< 문자열의 대리 쌍이 유효하지 않음.
  kParseErrorStringEscapeInvalid,      //!< 문자열에서 유효하지 않은 이스케이프 문자.
  kParseErrorStringMissQuotationMark,  //!< 문자열에서 닫는 따옴표가 없음.
  kParseErrorStringInvalidEncoding,    //!< 문자열에서 유효하지 않은 인코딩.

  kParseErrorNumberTooBig,        //!< double에 저장하기에 너무 큰 숫자.
  kParseErrorNumberMissFraction,  //!< 숫자에서 소수 부분이 없음.
  kParseErrorNumberMissExponent,  //!< 숫자에서 지수 부분이 없음.

  kParseErrorTermination,           //!< 파싱이 중단됨.
  kParseErrorUnspecificSyntaxError  //!< 구체적이지 않은 구문 오류.
};

//! 파싱 결과 (ParseErrorCode를 래핑)
/*!
    \ingroup RAPIDJSON_ERRORS
    \code
        Document doc;
        ParseResult ok = doc.Parse("[42]");
        if (!ok) {
            fprintf(stderr, "JSON parse error: %s (%u)",
                    GetParseError_En(ok.Code()), ok.Offset());
            exit(EXIT_FAILURE);
        }
    \endcode
    \see GenericReader::Parse, GenericDocument::Parse
*/
struct ParseResult {
  //!! 명시되지 않은 불리언 타입
  typedef bool (ParseResult::*BooleanType)() const;

 public:
  //! 기본 생성자, 에러 없음.
  ParseResult() : code_(kParseErrorNone), offset_(0) {}
  //! 에러를 설정하는 생성자.
  ParseResult(ParseErrorCode code, size_t offset) : code_(code), offset_(offset) {}

  //! 에러 코드를 가져옴.
  ParseErrorCode Code() const { return code_; }
  //! 에러 오프셋을 가져옴, \ref IsError()일 때만 유효, 그렇지 않으면 0.
  size_t Offset() const { return offset_; }

  //! \c bool로의 명시적 변환, \c true를 반환, !\ref IsError()일 때만.
  operator BooleanType() const { return !IsError() ? &ParseResult::IsError : NULL; }
  //! 결과가 에러인지 여부.
  bool IsError() const { return code_ != kParseErrorNone; }

  bool operator==(const ParseResult &that) const { return code_ == that.code_; }
  bool operator==(ParseErrorCode code) const { return code_ == code; }
  friend bool operator==(ParseErrorCode code, const ParseResult &err) {
    return code == err.code_;
  }

  bool operator!=(const ParseResult &that) const { return !(*this == that); }
  bool operator!=(ParseErrorCode code) const { return !(*this == code); }
  friend bool operator!=(ParseErrorCode code, const ParseResult &err) {
    return err != code;
  }

  //! 에러 코드를 초기화.
  void Clear() { Set(kParseErrorNone); }
  //! 에러 코드와 오프셋을 업데이트.
  void Set(ParseErrorCode code, size_t offset = 0) {
    code_ = code;
    offset_ = offset;
  }

 private:
  ParseErrorCode code_; // 에러 코드
  size_t offset_; // 오프셋
};

//! GetParseError()의 함수 포인터 타입.
/*! \ingroup RAPIDJSON_ERRORS

    이것은 \c GetParseError_X()의 프로토타입입니다, 여기서 \c X는 로케일입니다.
    사용자는 런타임에 로케일을 동적으로 변경할 수 있습니다, 예:
\code
    GetParseErrorFunc GetParseError = GetParseError_En; // 또는 다른 함수
    const RAPIDJSON_ERROR_CHARTYPE* s = GetParseError(document.GetParseErrorCode());
\endcode
*/
typedef const RAPIDJSON_ERROR_CHARTYPE *(*GetParseErrorFunc)(ParseErrorCode);

RAPIDJSON_NAMESPACE_END // rapidjson 네임스페이스 끝

#ifdef __clang__ // 클랭 컴파일러를 사용할 때
RAPIDJSON_DIAG_POP // 진단 팝
#endif

#endif  // RAPIDJSON_ERROR_ERROR_H_ // RAPIDJSON_ERROR_ERROR_H_ 종료
