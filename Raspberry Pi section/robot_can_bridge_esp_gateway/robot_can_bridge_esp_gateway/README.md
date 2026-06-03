# robot_can_bridge

ROS2 Jazzy package for Raspberry Pi SocketCAN MCP2515 -> ESP32 gateway -> S32K motor controller.

Current S32K protocol used:
- TX control to ESP32 gateway: ID 0x120, DLC 5: cmd, speedL_hi, speedL_lo, speedR_hi, speedR_lo
- ESP32 forwards to S32K using ID 0x111
- RX status from S32K: ID 0x222, DLC 5: fbL_hi, fbL_lo, fbR_hi, fbR_lo, emergency

Commands:
- 0x00 STOP
- 0x01 FORWARD
- 0x02 BACKWARD
- 0x03 TURN_LEFT
- 0x04 TURN_RIGHT
