# Livox publish pointcloud msg format.
std_msgs/Header header    # ROS 표준 메시지 헤더
uint64 timebase           # 첫 번째 포인트의 시간
uint32 point_num          # 포인트 클라우드의 총 개수
uint8  lidar_id           # 라이다 장치 ID 번호
uint8[3]  rsvd            # 예약된 용도
CustomPoint[] points      # 포인트 클라우드 데이터
