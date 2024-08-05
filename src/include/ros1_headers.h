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

// Denoting headers specifically used for building ROS1 Driver.

#ifndef ROS1_HEADERS_H_ // ROS1_HEADERS_H_가 정의되지 않았을 때
#define ROS1_HEADERS_H_ // ROS1_HEADERS_H_를 정의

#include <thread> // 스레드 사용을 위한 헤더 파일 포함
#include <future> // 비동기 작업을 위한 헤더 파일 포함

#include <ros/ros.h> // ROS 기본 헤더 파일 포함
#include <rosbag/bag.h> // ROSbag 사용을 위한 헤더 파일 포함
#include <pcl_ros/point_cloud.h> // PCL 포인트 클라우드 사용을 위한 헤더 파일 포함
#include <sensor_msgs/Imu.h> // IMU 센서 메시지 사용을 위한 헤더 파일 포함
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드2 메시지 사용을 위한 헤더 파일 포함
#include "livox_ros_driver2/CustomMsg.h" // CustomMsg 메시지 사용을 위한 헤더 파일 포함
#include "livox_ros_driver2/CustomPoint.h" // CustomPoint 메시지 사용을 위한 헤더 파일 포함

// 디버그 매크로 정의
#define DRIVER_DEBUG(node, ...) ROS_DEBUG(__VA_ARGS__)
#define DRIVER_INFO(node, ...) ROS_INFO(__VA_ARGS__)
#define DRIVER_WARN(node, ...) ROS_WARN(__VA_ARGS__)
#define DRIVER_ERROR(node, ...) ROS_ERROR(__VA_ARGS__)
#define DRIVER_FATAL(node, ...) ROS_FATAL(__VA_ARGS__)

// 확장 디버그 매크로 정의
#define DRIVER_DEBUG_EXTRA(node, EXTRA, ...) ROS_DEBUG_##EXTRA(__VA_ARGS__)
#define DRIVER_INFO_EXTRA(node, EXTRA, ...) ROS_INFO_##EXTRA(__VA_ARGS__)
#define DRIVER_WARN_EXTRA(node, EXTRA, ...) ROS_WARN_##EXTRA(__VA_ARGS__)
#define DRIVER_ERROR_EXTRA(node, EXTRA, ...) ROS_ERROR_##EXTRA(__VA_ARGS__)
#define DRIVER_FATAL_EXTRA(node, EXTRA, ...) ROS_FATAL_##EXTRA(__VA_ARGS__)

#endif // ROS1_HEADERS_H_ // ROS1_HEADERS_H_ 종료
