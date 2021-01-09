#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <OLLO.h>

OLLO myLed;
OLLO myIR;

const int DXL_DIR_PIN = 28;
Dynamixel2Arduino dxl(Serial1, DXL_DIR_PIN);

float goalPos;

#define DEGREES2RAW(x) (512 + (x/0.29))

#define LED_PORT (3)
#define IR_PORT (4)

void setPosition(uint8_t id, float value) {
  dxl.setGoalPosition(id, value);
  delayMicroseconds(150);// wait for the response to pass, default sent after 50us 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.setDxlMode(true);
  dxl.begin(1000000);
  for (int i=1; i <= 17; i++) {
    dxl.torqueOn(i);
  }  

  Serial.println(dxl.getPortBaud());
  Serial.println(dxl.scan());
  
  goalPos = 400;
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

  myLed.begin(LED_PORT, LED_DISPLAY);//LED Display Module must be connected at port 3.
  myIR.begin(IR_PORT, IR_SENSOR);//IR Module must be connected at port 1.
}


void loop() {
#if 0
  Serial.print("Press any key to continue! (or press q to quit!)\n");

  while(Serial.available()==0);

    int ch;

    ch = Serial.read();
    if (ch == 'q')
      return;

  // put your main code here, to run repeatedly:
    dxl.setGoalPosition(1, goalPos); //use default encoder value
    if (goalPos == 400) goalPos = 600;
    else goalPos = 400;
#endif

  //write( port number, left LED(blue), unused, right LED(yellow) )
  myLed.write(LED_PORT, 1, 0, 0);
  delay(1000);
  myLed.write(LED_PORT, 0, 0, 1);
  delay(1000);

  Serial.print("IR Sensor ADC = ");
  int adc = myIR.read(IR_PORT, IR_SENSOR);//read ADC value from OLLO port 4
  Serial.println(adc); 

  if (adc < 100) {
    setPosition(17, DEGREES2RAW(0));
  }
  else {
    setPosition(17, DEGREES2RAW(30));
  }
  
}
