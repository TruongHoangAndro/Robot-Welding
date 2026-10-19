# awm_interfaces

ROS 2 interfaces of DES-001 §7.1, first version (v0) for the runtime modules (Phase 1 R2):
`CellCommand.srv`, `CellState.msg`, `ExecuteSegment.action`, `WeldEvent.action`,
`PlcStatus.msg`, `Alarm.msg`. The offline chain (A1-A7) uses MoveIt / ros2_control interfaces only.
Signal details wait for DES-002 (TODO(OI-1), TODO(OI-2), TODO(OI-4)).
