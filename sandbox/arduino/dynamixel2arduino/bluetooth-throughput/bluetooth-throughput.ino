#include <Dynamixel2Arduino.h>

const int DXL_DIR_PIN = 28;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);

#define DEGREES2RAW(x) (512 + (x/0.29))

#define BLUETOOTH_BAUDRATE 1382400
// > hcitool scan
// > sudo rfcomm connect 1 B8:63:BC:00:72:E7
// > sudo cat /dev/rfcomm1 or screen /dev/rfcomm1 1382400
// > sudo rfcomm release 1 or CTRL-C
//https://emanual.robotis.com/docs/en/parts/communication/bt-210/
// default baudrate is set to 57600, change baudrate through AT+BTUART command

void setPosition(uint8_t id, float value) {
  dxl.setGoalPosition(id, value);
  delayMicroseconds(150);// wait for the response to pass, default sent after 50us
}

static unsigned long ts_value = 0;

void setup() {
  // put your setup code here, to run once:
  while (!Serial) ;
  Serial.begin(115200);
  Serial1.setDxlMode(true);
  dxl.begin(1000000);
  for (int i = 1; i <= 17; i++) {
    dxl.torqueOn(i);
  }

  Serial.println(dxl.getPortBaud());
  Serial.println(dxl.scan());

  //left arm
  setPosition(1, DEGREES2RAW(0));
  setPosition(3, DEGREES2RAW(-73.24));
  setPosition(5, DEGREES2RAW(0));

  // right arm
  setPosition(2, DEGREES2RAW(0));
  setPosition(4, DEGREES2RAW(73.24));
  setPosition(6, DEGREES2RAW(0));

  // head
  setPosition(17, DEGREES2RAW(0));

  //left leg
  setPosition(7, DEGREES2RAW(0));
  setPosition(9, DEGREES2RAW(-26.37));
  setPosition(11, DEGREES2RAW(29.3));
  setPosition(13, DEGREES2RAW(13.18));
  setPosition(15, DEGREES2RAW(0));

  // right leg
  setPosition(8, DEGREES2RAW(0));
  setPosition(10, DEGREES2RAW(26.37));
  setPosition(12, DEGREES2RAW(-29.3));
  setPosition(14, DEGREES2RAW(-13.18));
  setPosition(16, DEGREES2RAW(0));

  Serial2.begin(BLUETOOTH_BAUDRATE);

  // test code
  for (int i=0; i<10;i++)
  {
    ts_value = micros();
    //Serial.println(ts_value);
    Serial2.write((uint8_t *)&ts_value, 4);
  }
}


void loop() {
  //ts_value = micros();
  //Serial.println(ts_value);
  //Serial2.write((uint8_t *)&ts_value, 4);
}
