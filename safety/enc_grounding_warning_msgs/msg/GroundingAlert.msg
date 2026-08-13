std_msgs/Header header
uint8 level               # 1=INFO 2=WARNING 3=CRITICAL
uint8 type                # 1=CURRENT 2=LOOKAHEAD 3=ROUTE
float32 min_ukc_m
float32 ukc_required_m
float32 distance_to_danger_m
float32 time_to_danger_s
float64 danger_x          # map 系
float64 danger_y
uint32 route_index        # 首个危险路径点序号，无效为 0
string description
