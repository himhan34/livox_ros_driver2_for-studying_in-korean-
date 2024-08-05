import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

################### 사용자 설정 매개변수 시작 ###################
xfer_format = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src = 0      # 0-lidar, others-Invalid data src
publish_freq = 10.0 # 발행 주기, 5.0, 10.0, 20.0, 50.0 등
output_type = 0    # 출력 데이터 형식
frame_id = 'livox_frame' # 메시지 프레임 ID
lvx_file_path = '/home/livox/livox_test.lvx' # lvx 파일 경로
cmdline_bd_code = 'livox0000000001' # 기기 시리얼 번호

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = os.path.join(cur_path, '../config')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')
################### 사용자 설정 매개변수 끝 #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    return LaunchDescription([
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
