#include <DynamixelSDK.h>


// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "1"                 // Check which port is being used on your controller
// DEVICENAME "1" -> Serial1
// DEVICENAME "2" -> Serial2
// DEVICENAME "3" -> Serial3(OpenCM 485 EXP)

// Control table address
#define ADDR_TORQUE_ENABLE          24                 // Control table address is different in Dynamixel model
#define ADDR_LED                    25
#define ADDR_GOAL_POSITION          30
#define ADDR_PRESENT_POSITION       37

// Data Byte Length
#define LEN_LED                  1
#define LEN_GOAL_POSITION        2
#define LEN_PRESENT_POSITION     2


#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#3 ID: 3
#define DXL4_ID                         4                   // Dynamixel#4 ID: 4
#define DXL5_ID                         5                   // Dynamixel#5 ID: 5
#define DXL6_ID                         6                   // Dynamixel#6 ID: 6
#define DXL7_ID                         7                   // Dynamixel#7 ID: 7
#define DXL8_ID                         8                   // Dynamixel#8 ID: 8
#define DXL9_ID                         9                   // Dynamixel#9 ID: 9
#define DXL10_ID                       10                   // Dynamixel#10 ID: 10
#define DXL11_ID                       11                   // Dynamixel#11 ID: 11
#define DXL12_ID                       12                   // Dynamixel#12 ID: 12
#define DXL13_ID                       13                   // Dynamixel#13 ID: 13
#define DXL14_ID                       14                   // Dynamixel#14 ID: 14
#define DXL15_ID                       15                   // Dynamixel#15 ID: 15
#define DXL16_ID                       16                   // Dynamixel#16 ID: 16
#define DXL17_ID                       17                   // Dynamixel#17 ID: 17

#define LED_COLOR_NONE   0
#define LED_COLOR_RED    1
#define LED_COLOR_GREEN  2
#define LED_COLOR_BLUE   4
#define LED_COLOR_YELLOW 3
#define LED_COLOR_CYAN   6
#define LED_COLOR_PURPLE 5
#define LED_COLOR_WHITE  7


#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque


dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

#define DEBUG 1

void setup() {
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start..");
#endif


  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);


  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_model_number;                      // Dynamixel model number

  std::vector<uint8_t> vec;                       // Dynamixel data storages

  // Open port
  if (portHandler->openPort())
  {
#ifdef DEBUG
    Serial.print("Succeeded to open the port!\n");
#endif
  }
  else
  {
#ifdef DEBUG
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
#endif
    return;
  }


  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
#ifdef DEBUG
    Serial.print("Succeeded to change the baudrate!\n");
#endif
  }
  else
  {
#ifdef DEBUG
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
#endif
    return;
  }

  // Try to broadcast ping the Dynamixel
  dxl_comm_result = packetHandler->broadcastPing(portHandler, vec);
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

#ifdef DEBUG
  Serial.print("Detected Dynamixel : \n");
  for (int i = 0; i < (int)vec.size(); i++)
  {
    Serial.print("ID : ");
    Serial.println(vec.at(i));
  }
#endif

  // set the dynamixels to initial pose

  // Initialize GroupBulkWrite instance
  dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

  uint8_t param_led[1];
  param_led[0] = DXL_LOBYTE(LED_COLOR_YELLOW);

  // Add parameter storage for Dynamixel#1 goal position
  bool dxl_addparam_result = groupBulkWrite.addParam(DXL1_ID, ADDR_LED, LEN_LED, param_led);
  if (dxl_addparam_result != true)
  {
#ifdef DEBUG
    Serial.print(DXL1_ID);
    Serial.println(" groupBulkWrite addparam failed");
#endif
    return;
  }

  // test
  for (uint8_t i = DXL2_ID; i <= DXL17_ID; i++) {
    dxl_addparam_result = groupBulkWrite.addParam(i, ADDR_LED, LEN_LED, param_led);
    if (dxl_addparam_result != true)
    {
#ifdef DEBUG
      Serial.print(i);
      Serial.println(" groupBulkWrite addparam failed");
#endif
      return;
    }
  }

  // Bulkwrite goal position and LED value
  dxl_comm_result = groupBulkWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
#ifdef DEBUG
    Serial.print("txpacket failure : ");
    Serial.println(dxl_comm_result);
#endif
  }

  // Clear bulkwrite parameter storage
  groupBulkWrite.clearParam();

  // test: read position ID1

  // Close port
  //portHandler->closePort();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t dxl_error = 0;                          // Dynamixel error
  int16_t dxl_present_position = 0;               // Present position

   // Enable Dynamixel Torque
  int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
#ifdef DEBUG
    Serial.print("write1ByteTxRx comm failure : ");
    Serial.println(dxl_comm_result);
#endif
  }
  else if (dxl_error != 0)
  {
#ifdef DEBUG
    Serial.print("write1ByteTxRx error failure : ");
    Serial.println(dxl_error);
#endif
  }
  else
  {
    Serial.print("Dynamixel has been successfully connected \n");
  }

  
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint16_t*)&dxl_present_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
#ifdef DEBUG
    Serial.print("read2ByteTxRx comm failure : ");
    Serial.println(dxl_comm_result);
#endif
  }
  else if (dxl_error != 0)
  {
#ifdef DEBUG
    Serial.print("read2ByteTxRx error failure : ");
    Serial.println(dxl_error);
#endif
  }

#ifdef DEBUG
  Serial.print("current pos : ");
  Serial.println(dxl_present_position);
#endif


  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
#ifdef DEBUG
    Serial.print("write1ByteTxRx comm failure : ");
    Serial.println(dxl_comm_result);
#endif
  }
  else if (dxl_error != 0)
  {
#ifdef DEBUG
    Serial.print("write1ByteTxRx error failure : ");
    Serial.println(dxl_error);
#endif
  }

  delay(1000);

}
