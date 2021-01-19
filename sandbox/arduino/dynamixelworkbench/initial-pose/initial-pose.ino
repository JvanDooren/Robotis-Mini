#include <DynamixelWorkbench.h>
#include <OLLO.h>

#define DEVICE_NAME "1" //Dynamixel on Serial1(USART1)
#define BAUDRATE 1000000
#define EXPECTED_NR_DYNAMIXEL_SERVOS 17

#define DEGREES2RAW(x) (int32_t)(512 + (x/0.29))
#define LED_PORT (3)
#define LED_OFF (1)
#define LED_ON (0)

#define ADDRESS_XL320_GOAL_POSITION       30
#define LENGTH_XL320_GOAL_POSITION        2
#define SYNC_GOAL_POSITION_HANDLER_INDEX  0

//#define SYNC_WRITE

OLLO myLed;
DynamixelWorkbench dxl_wb;

void setPosition(uint8_t id, int value) {
  const char *workbench_error;
  bool result = dxl_wb.goalPosition(id, value, &workbench_error);
  if (result == false) {
    Serial.print("Failed to set goalpos for ID: ");
    Serial.print(id);
    Serial.print(" due to: ");
    Serial.println(workbench_error);
    //right LED(blue)
    myLed.write(LED_PORT, LED_OFF, 0, LED_ON);
    delayMicroseconds(150);
  }
}

void setup() 
{
  //while (!Serial);
  Serial.begin(115200);  

  myLed.begin(LED_PORT, LED_DISPLAY);//LED Display Module must be connected at port 3.
  myLed.write(LED_PORT, LED_OFF, 0, LED_OFF);

  const char *workbench_error;
  bool result = false;

  uint8_t scanned_id[EXPECTED_NR_DYNAMIXEL_SERVOS] = {0};
  uint8_t dxl_cnt = 0;

  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &workbench_error);
  if (result == false)
  {
    Serial.print("Failed to init: ");
    Serial.println(workbench_error);
  }
  else
  {
    Serial.print("Succeed to init : ");
    Serial.println(BAUDRATE);
  }

  result = dxl_wb.scan(scanned_id, &dxl_cnt, EXPECTED_NR_DYNAMIXEL_SERVOS, &workbench_error);
  if (result == false)
  {
    Serial.print("Failed to scan: ");
    Serial.println(workbench_error);
  }
  else
  {
    Serial.print("Found ");
    Serial.print(dxl_cnt);
    Serial.println(" Dynamixels");

    for (int cnt = 0; cnt < dxl_cnt; cnt++)
    {
      Serial.print("id : ");
      Serial.print(scanned_id[cnt]);
      Serial.print(" model name : ");
      Serial.println(dxl_wb.getModelName(scanned_id[cnt]));
    }
  }

  for (int cnt = 0; cnt < dxl_cnt; cnt++)
  {
    // this causes intermittent failures in servo 16
    // sometimes it doesn't enable torque and goalposition isn't responded to
    //result = dxl_wb.torqueOn(scanned_id[cnt], &workbench_error);
    
    result = dxl_wb.jointMode(scanned_id[cnt], 0, 0, &workbench_error);
    if (result == false)
    {
      Serial.print("Failed to activate torque for ID: ");
      Serial.print(scanned_id[cnt]);
      Serial.print(" due to: ");
      Serial.println(workbench_error);
      //left LED(yellow)
      myLed.write(LED_PORT, LED_ON, 0, LED_OFF);
    }
    else
    {
      Serial.print("Succeed to activate torque for ID: ");
      Serial.println(scanned_id[cnt]);
    }
  }
#ifdef SYNC_WRITE
  Serial.println("Using sync write position");
  result = dxl_wb.addSyncWriteHandler(ADDRESS_XL320_GOAL_POSITION, LENGTH_XL320_GOAL_POSITION, &workbench_error);
  if (result == false)
  {
    Serial.println(workbench_error);
    Serial.println("Failed to add sync write handler");
  }
  else
  {
    int32_t positions[EXPECTED_NR_DYNAMIXEL_SERVOS] = {
      DEGREES2RAW(0),//1
      DEGREES2RAW(0),//2
      DEGREES2RAW(-73.24),//3
      DEGREES2RAW(73.24),//4
      DEGREES2RAW(0),//5
      DEGREES2RAW(0),//6
      DEGREES2RAW(0),//8
      DEGREES2RAW(-26.37),//9
      DEGREES2RAW(26.37),//10
      DEGREES2RAW(29.3),//11
      DEGREES2RAW(-29.3),//12
      DEGREES2RAW(13.18),//13
      DEGREES2RAW(-13.18),//14
      DEGREES2RAW(0),//15
      DEGREES2RAW(0),//16
      DEGREES2RAW(0),//17
      };
    Serial.println("Calling sync write position");
    // holy sh*t, what is going on here, the right leg is not going where it is supposed to go, nor is the head servo
    result = dxl_wb.syncWrite(SYNC_GOAL_POSITION_HANDLER_INDEX, scanned_id, EXPECTED_NR_DYNAMIXEL_SERVOS, positions, 1, &workbench_error);
    if (result == false)
    {
      Serial.println(workbench_error);
      Serial.println("Failed to sync write position");
    }
    else
    {
      Serial.println("Success to sync write position");
    }
  }
#else
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
#endif
}

void loop() 
{

}
