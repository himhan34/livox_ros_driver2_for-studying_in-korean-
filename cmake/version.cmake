#---------------------------------------------------------------------------------------
# include/livox_ros_driver2.h 파일에서 livox_ros_driver2 버전을 가져옴
#---------------------------------------------------------------------------------------
file(READ "${CMAKE_CURRENT_LIST_DIR}/../src/include/livox_ros_driver2.h" LIVOX_ROS_DRIVER2_VERSION_FILE) # 파일을 읽음
string(REGEX MATCH "LIVOX_ROS_DRIVER2_VER_MAJOR ([0-9]+)" _  "${LIVOX_ROS_DRIVER2_VERSION_FILE}") # 메이저 버전을 정규식으로 추출
set(ver_major ${CMAKE_MATCH_1}) # 메이저 버전 설정

string(REGEX MATCH "LIVOX_ROS_DRIVER2_VER_MINOR ([0-9]+)" _  "${LIVOX_ROS_DRIVER2_VERSION_FILE}") # 마이너 버전을 정규식으로 추출
set(ver_minor ${CMAKE_MATCH_1}) # 마이너 버전 설정
string(REGEX MATCH "LIVOX_ROS_DRIVER2_VER_PATCH ([0-9]+)" _  "${LIVOX_ROS_DRIVER2_VERSION_FILE}") # 패치 버전을 정규식으로 추출
set(ver_patch ${CMAKE_MATCH_1}) # 패치 버전 설정

if (NOT DEFINED ver_major OR NOT DEFINED ver_minor OR NOT DEFINED ver_patch) # 버전 정보가 유효하지 않으면
    message(FATAL_ERROR "Could not extract valid version from include/livox_ros_driver2.h") # 오류 메시지 출력
endif()
set (LIVOX_ROS_DRIVER2_VERSION "${ver_major}.${ver_minor}.${ver_patch}") # 버전 정보 설정
