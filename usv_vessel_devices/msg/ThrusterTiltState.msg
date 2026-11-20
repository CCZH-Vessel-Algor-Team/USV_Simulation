# Simulation time; angles in rad, positive raises the outboard aft.
std_msgs/Header header
bool ready
string status
# All arrays use this order. Match by name rather than a fixed index.
string[] thruster_names
float64[] position
float64[] target
bool[] moving
bool[] at_target
# Propeller physics entities are available; independent of tilt angle.
bool[] thrust_enabled
# Last accepted bounded command, NOT the actual hydrodynamic force.
float64[] applied_thrust
