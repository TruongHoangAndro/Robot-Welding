# /awm/plc/status — M10 → M8, M11, HMI. Signal detail in DES-002 (TODO(OI-1), TODO(OI-2), TODO(OI-4)).
std_msgs/Header header
string plc_state
bool heartbeat_ok
bool safety_ok
bool door_closed
bool welder_ready
bool laser_ready
bool arc_ok
float32 actual_current_a
float32 actual_voltage_v
float32 actual_laser_power_w
