std_msgs/Header header
float64 origin_x          # map 系左下角 X
float64 origin_y          # map 系左下角 Y
float32 resolution        # m/cell
int32 width
int32 height
float32[] depth_m         # row-major，-9999=未知
float32[] uncertainty_m
uint8[] quality           # 0=UNKNOWN 1=A1 2=A2 3=B 4=C 5=D 6=U
uint8[] flags             # 1=LAND 2=DRYING 4=NO_DATA 8=HAZARD_UNKNOWN_DEPTH
