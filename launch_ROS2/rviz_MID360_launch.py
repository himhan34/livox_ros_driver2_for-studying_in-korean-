import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

################### 사용자 설정 매개변수 시작 ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-사용자 정의 포인트 클라우드 형식
multi_topic   = 0    # 0-모든 LiDAR가 동일한 토픽 사용, 1-각 LiDAR가 개별 토픽 사용
data_src      = 0    # 0-lidar, 그 외-유효하지 않은 데이터 소스
publish_freq  = 10.0 # 발행 주기, 5.0, 10.0, 20.0, 50.0 등
output_type   = 0    # 출력 데이터 형식
frame_id      = 'livox_frame' # 메시지 프레임 ID
lvx_file_path = '/home/livox/livox_test.lvx' # lvx 파일 경로
cmdline_bd_code = 'livox0000000001' # 기기 시리얼 번호

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/' # 현재 파일의 디렉토리 경로 설정
cur_config_path = cur_path + '../config' # 설정 파일 경로 설정
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz') # RViz 설정 파일 경로 설정
user_config_path = os.path.join(cur_config_path, 'MID360_config.json') # 사용자 설정 파일 경로 설정
################### 사용자 설정 매개변수 끝 #####################

# Livox ROS 2 매개변수 설정
livox_ros2_params = [
    {"xfer_format": xfer_format}, # 전송 형식
    {"multi_topic": multi_topic}, # 다중 토픽 사용 여부
    {"data_src": data_src}, # 데이터 소스
    {"publish_freq": publish_freq}, # 발행 주기
    {"output_data_type": output_type}, # 출력 데이터 형식
    {"frame_id": frame_id}, # 메시지 프레임 ID
    {"lvx_file_path": lvx_file_path}, # lvx 파일 경로
    {"user_config_path": user_config_path}, # 사용자 설정 파일 경로
    {"cmdline_input_bd_code": cmdline_bd_code} # 기기 시리얼 번호
]

# Launch 설명을 생성하는 함수
def generate_launch_description():
    # Livox 드라이버 노드 설정
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # RViz 노드 설정
    livox_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    # Launch 설명 반환
    return LaunchDescription([
        livox_driver, # Livox 드라이버 노드 실행
        livox_rviz,   # RViz 노드 실행
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
