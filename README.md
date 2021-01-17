# Robotis-Mini
Robotis Mini Arduino IDE code, ROS2 interface

## Set BT-210 baudrate
The BT-210 is default set to 57600 baud, which is too slow for relaying state.
Follow the instructions at https://emanual.robotis.com/docs/en/parts/communication/bt-210/ to set to a higher baudrate.
Use the define in the serial2serial ino file to set the BT-210 communication baudrate to 57600, then upload the ino.
After changing the baudrate by AT commands, enforce the setting by power-cycling the BT-210 or 'ATZ'. Once the setting is active, this new baudrate (1382400) will be the fixed rate for opening Serial2 on Arduino.

### Effective transmission rate
After some tests, it appears that the max rate the BT-210 can withstand internally (without overflowing buffers and loosing bytes) is around 22000 bytes/s, which is roughly 176000 baud. Essentially, there needs to be a minimum delay of 175us between every Serial2.write call to prevent the BT-210 from loosing bytes. So, although the set baudrate of 1382400 baud is far from achievable, this setting still gives us around 175us of time do do something else in the loop (which should be perfect to get the status of all Dynamixels).

## Arduino source

https://github.com/chcbaram/OpenCM9.04/tree/master/hardware/robotis/OpenCM9.04/cores/arduino
