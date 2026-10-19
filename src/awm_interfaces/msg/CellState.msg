# /awm/cell/state (latched) — M8 → HMI, M11. States per DES-001 §6.1.
string INIT=INIT
string NOT_READY=NOT_READY
string IDLE=IDLE
string READY=READY
string LOCATE_PART=LOCATE_PART
string WELDING=WELDING
string CYCLE_END=CYCLE_END
string FAULT=FAULT
string E_STOP=E_STOP

std_msgs/Header header
string state
string job_id
string program_rev
string seam_id
float32 progress            # 0..1 of the current cycle
string[] active_alarms      # alarm ids
