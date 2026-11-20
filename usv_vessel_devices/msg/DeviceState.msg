# Simulation timestamp. Angles are radians, positive folds the mast aft.
std_msgs/Header header
# Ready means the joint has physics feedback and all configured lamp visuals exist.
bool ready
string status
# Same order for both arrays. First five: left_front, left_rear, right_front, right_rear, mast.
# Optional additional lamp names follow; match by name instead of assuming array length.
string[] light_names
# Applied simulation material state, NOT a copy of the most recent command.
bool[] lights_on
float64 mast_position
float64 mast_target
bool mast_moving
bool mast_at_target
