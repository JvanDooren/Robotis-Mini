#define BLUETOOTH_BAUDRATE 1382400
// > hcitool scan
// > sudo rfcomm connect 1 B8:63:BC:00:72:E7
// > sudo cat /dev/rfcomm1 or screen /dev/rfcomm1 1382400
// > sudo rfcomm release 1 or CTRL-C
//https://emanual.robotis.com/docs/en/parts/communication/bt-210/
// default baudrate is set to 57600, change baudrate through AT+BTUART command

static unsigned long ts_value = 0;

void setup() {
  // put your setup code here, to run once:
  while (!Serial) ;
  Serial.begin(115200);
  Serial2.begin(BLUETOOTH_BAUDRATE);

  // test code
  Serial.println("Sending");
  for (int i=0; i<100000;i++)
  {
    ts_value = micros();
    //Serial.println(ts_value);
    while (Serial2.availableForWrite() < 4);
    Serial2.write((uint8_t *)&ts_value, 4);
    Serial2.flush();
    delayMicroseconds(135);//135==ok, resulting in 22500 bytes/sec
    
  }
  Serial.println("Done");
}


void loop() {
  //ts_value = micros();
  //Serial.println(ts_value);
  //Serial2.write((uint8_t *)&ts_value, 4);
}
