# Livox costom pointcloud format.
uint32 offset_time      # 기준 시간에 대한 오프셋 시간
float32 x               # X 축, 단위: m
float32 y               # Y 축, 단위: m
float32 z               # Z 축, 단위: m
uint8 reflectivity      # 반사율, 0~255
uint8 tag               # livox 태그
uint8 line              # 라이다 내 레이저 번호
