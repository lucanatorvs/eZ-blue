#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(8);


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 6) {
      Serial.print(canMsg.can_id, HEX); // print ID
      Serial.print(" "); 
      Serial.print(canMsg.can_dlc, HEX); // print DLC
      Serial.print(" ");
      
      for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
        Serial.print(canMsg.data[i],HEX);
        Serial.print(" ");
      }

      // take the 3d and 4th byte and convert it to a int
      int16_t value = (canMsg.data[3] << 8) | canMsg.data[2];
      Serial.print("Value: ");
      Serial.println(value);

      // we map the value from 0 to 7000 to 0 230
      int mappedValue = map(value, 0, 7000, 0, 230); 

      // D3 is the rpm guage pin
      // put a frequency of 50Hz on pin D3

      // mappedValue is never lower than 10
      if (mappedValue < 10) {
        mappedValue = 10;
      } else if (mappedValue > 230) {
        mappedValue = 230;
      }

      tone(3, mappedValue);

      Serial.println();     
    } 
  }
}