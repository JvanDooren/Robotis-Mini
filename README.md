# Robotis-Mini
Robotis Mini Arduino IDE code, ROS2 interface

## Set BT-210 baudrate
The BT-210 is default set to 57600 baud, which is too slow for relaying state.
Follow the instructions at https://emanual.robotis.com/docs/en/parts/communication/bt-210/ to set to a higher baudrate.
After changing the baudrate by AT commands, enforce the setting by power-cycling the BT-210, it is activated only @ boot, not changed @ runtime. Once the setting is active, this new baudrate will be the fixed rate for opening Serial2 on Arduino.
