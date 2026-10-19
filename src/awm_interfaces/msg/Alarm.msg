# /awm/alarm — all modules → M8, M11, HMI (DES-001 §7.1)
uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 FATAL=3

std_msgs/Header header
string alarm_id
string source
uint8 severity
string text_en
string text_vn
