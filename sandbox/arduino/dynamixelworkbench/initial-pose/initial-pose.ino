#include <DynamixelWorkbench.h>

#define DEVICE_NAME "1" //Dynamixel on Serial1(USART1)
#define BAUDRATE 1000000
#define EXPECTED_NR_DYNAMIXEL_SERVOS 17

#define DEGREES2RAW(x) (512 + (x/0.29))

DynamixelWorkbench dxl_wb;

void setPosition(uint8_t id, int value) {
  dxl_wb.goalPosition(id, value);
  //delayMicroseconds(150);// wait for the response to pass, default sent after 50us 
}

void setup() 
{
  while (!Serial) ;
  Serial.begin(115200);  

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
    result = dxl_wb.torqueOn(scanned_id[cnt], &workbench_error);
    if (result == false)
    {
      Serial.print("Failed to activate torque for ID: ");
      Serial.print(scanned_id[cnt]);
      Serial.print(" due to: ");
      Serial.println(workbench_error);
    }
    else
    {
      Serial.print("Succeed to activate torque for ID: ");
      Serial.println(scanned_id[cnt]);
    }
  }

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
}

void loop() 
{

}
