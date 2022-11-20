// log: 
//  20180114 - the ant has risen! all the servos turning at once consume about 4A @ 5V (!)  teensy 3.5 version
//  20180819 - hw functions properly
//  20180821 - cmd interface
//  20181218 - added feet contact pins
//  20190329 - added serial1 for bt connectivity
//  20200423 - build fixes for arduino 1.8.12 teensyduino 1.51
//  20201022 - dynamixel version
//  20210119 - temp read improvements
//  20220709 - arduino nano 33 iot + realant mainboard v1.1 works, requires modified dynamixelsdk library

//#include <DynamixelWorkbench.h>
#include <DynamixelSDK.h>

// AX-series Control table address
#define ADDR_AX_TORQUE_ENABLE           24                 // Control table address is different in Dynamixel model
#define ADDR_AX_GOAL_POSITION           30
#define ADDR_AX_TORQUE_LIMIT            34
#define ADDR_AX_PRESENT_POSITION        36
#define ADDR_AX_PRESENT_TEMPERATURE     43

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel




#define DEVICE_NAME "1" //Dynamixel on Serial1 (Arduino Nano 33 IoT)

#define BAUDRATE  1000000



int servopos[] = {512, 512, 512, 512, 512, 512, 512, 512};


// feet contact pins 33-36 (plywood), (white ant pins 29-32)

//const int feet_contact_pins[] = {29,30,31,32};

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

dynamixel::PortHandler *portHandler = NULL;
dynamixel::PacketHandler *packetHandler = NULL;

int read_servo_i = 1;
int servo_temps[] = {0,0,0,0, 0,0,0,0};

void read_servo_positions()
{
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;  
  const char *log;

  uint8_t pos_failures = 0;
  uint8_t temp_failures = 0;

  bool result = false;

    //unsigned long start_read = millis();

    Serial.print("meas ");

    Serial.print(millis());
    Serial.print("\t");

  
    for (int i = 1; i <= 8; i++) {
      uint16_t get_data = servopos[i-1]; // backup estimate

      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_AX_PRESENT_POSITION, (uint16_t*)&get_data, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        pos_failures |= 1<<(i-1);
      }

      Serial.print(get_data);
      Serial.print("\t");
    }


    for (int i = 0; i < 8; i++) {
      Serial.print(servopos[i]);
      Serial.print("\t");
    }

    for (int i = 0; i < 4; i++) {
      Serial.print(0); //not digitalRead(feet_contact_pins[i]));
      Serial.print("\t");
    }

    // temps, read one servo per cycle to speed up cycle time
    {
      int8_t get_data = 0;
  
      dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, read_servo_i, ADDR_AX_PRESENT_TEMPERATURE, (uint8_t*)&get_data, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        Serial.println("- Failed to get present temperature");
        temp_failures |= 1<<(read_servo_i-1);
      } else {
        servo_temps[read_servo_i-1] = get_data;
      }
  
      read_servo_i++;
      if (read_servo_i == 9)
        read_servo_i = 1;
        
      for (int i = 1; i <= 8; i++) {
        Serial.print(servo_temps[i-1]);
        Serial.print("\t");
      }
    }

    Serial.print("\t");
    Serial.print(freeMemory());
    
    Serial.println();

    //Serial.print("- read took ");
    //Serial.println((millis() - start_read));


    if (pos_failures > 0)
    {
        Serial.print("- Failed to get present position 0x");
        Serial.println(pos_failures, HEX);  
    }
    if (temp_failures > 0)
    {
        Serial.print("- Failed to get present temperature 0x");
        Serial.println(temp_failures, HEX);  
    }    
}

unsigned long last_time_millis = 0;

void setup() {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    const char *log;
    bool result = false;

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


    inputString.reserve(200);

    Serial.begin(115200);
    while(!Serial);
    Serial.println("realant says hello - ant14_cmd_dxl 2022-07-09");

#if defined(__SAMD21G18A__)
#if defined(SAMD_NANO_33_IOT)
    Serial.println("arduino nano 33 iot")
#endif
    Serial.println("samd21g18a");
#endif
    delay(1000);

    // Open port
    if (portHandler->openPort())
    {
      Serial.print("- Succeeded to open the Dynamixel port\n");
    }
    else
    {
      Serial.print("- Failed to open the Dynamixel port\n");
    }
  
    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
      Serial.print("- Succeeded to change the baudrate\n");
    }
    else
    {
      Serial.print("- Failed to change the baudrate\n");
    }

    for (int i = 1; i <= 8; i++) {

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_AX_TORQUE_LIMIT, 512, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
          Serial.print("- id: ");
          Serial.print(i);
          Serial.println("- Failed set torque limit (1)");
          
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
          Serial.print("- id: ");
          Serial.print(i);
          Serial.println("- Failed set torque limit (2)");
        }
        else
        {
          Serial.print("- id: ");
          Serial.print(i);
          Serial.println("- Succeeded to set torque limit");
        }
    }
      
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}



void loop() {

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  
  const char *log;
  bool result = false;

  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  if (stringComplete) {
    Serial.print("- recv: ");
    Serial.println(inputString);

    if (inputString.startsWith("attach_servos")) {
      for (int i = 1; i <= 8; i++) {
        //result = dxl_wb.jointMode(i, 0, 0, &log);
        //if (result == false)
        //{
        //  Serial.println(log);
        //  Serial.println("- Failed to change joint mode");
        //}
        //else
        //{
        //  Serial.println("- Succeeded to change joint mode");
        //} 

        
        //result = dxl_wb.torqueOn(i, NULL);
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_AX_TORQUE_ENABLE, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
          Serial.println("- Failed to change torque on");
        }
        else
        {
          Serial.println("- Succeeded to change torque on");
        } 
      }
    }
    else if (inputString.startsWith("detach_servos")) {
      for (int i = 1; i <= 8; i++) {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_AX_TORQUE_ENABLE, 0, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
          Serial.println("- Failed to change torque off");
        }
        else
        {
          Serial.println("- Succeeded to change torque off");
        } 
      }
    }
    else if (inputString.startsWith("reset")) {
      for (int i = 1; i <= 8; i++) {
        servopos[i-1] = 512;
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_AX_GOAL_POSITION, servopos[i-1], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          Serial.println("- Failed to set goal position (reset)");
        }
      }
    }
    else if (inputString.startsWith("standup")) {
      for (int i = 1; i <= 8; i++) {
        int val = (i % 2 == 1) ? 512 : 224;
        servopos[i-1] = val;
        dxl_comm_result = packetHandler->write2ByteTxOnly(portHandler, i, ADDR_AX_GOAL_POSITION, servopos[i-1]);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          Serial.println("- Failed to set goal position (standup)");
        }
      }
      
    }
    else {
      //unsigned long start_write = millis();
  
      for (int i = 0; i < 8; i++) {
        String k = getValue(inputString, ' ', 2*i);
        String v = getValue(inputString, ' ', 2*i+1);
  
        if (k != "" && k.startsWith("s"))
        {
          int s = k.substring(1).toInt();
          if (v != "" && (s >= 1 && s <= 8))
          {
            int pos =  v.toInt();
            //Serial.print("- driving servo ");
            //Serial.print(s);
            //Serial.print(" to pos ");
            //Serial.println(pos);
            servopos[s-1] = pos;
            
            dxl_comm_result = packetHandler->write2ByteTxOnly(portHandler, s, ADDR_AX_GOAL_POSITION, servopos[s-1]);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              Serial.println("- Failed to set goal position (standup)");
            }
          }
        }
      }

      //Serial.print("- write took ");
      //Serial.println((millis() - start_write));
    }



    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  if ((millis() - last_time_millis) > 20) {
    read_servo_positions();
    last_time_millis = millis();
  }
}


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
