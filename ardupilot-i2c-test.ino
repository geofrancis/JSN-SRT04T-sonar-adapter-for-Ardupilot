#include <Wire.h>

//Standard MaxsonarI2CXL address
#define I2C_SLAVE_ADDR  0x09 // set 9 in ardupilot

//#define I2C_SLAVE_ADDR  0x112 // set 9 in ardupilot
//#define I2C_SLAVE_ADDR  0x113 // set 9 in ardupilot
//define I2C_SLAVE_ADDR  0x114 // set 9 in ardupilot
//#define I2C_SLAVE_ADDR  0x115 // set 9 in ardupilot


unsigned char data_buffer[4] = {0};
unsigned char CS;
uint8_t Index;
byte received;

int distance = 99 ;

//send results over i2c
void requestEvent() 
{  
Wire.write (highByte(distance));
Wire.write (lowByte(distance));
        Serial.print ("i2c out");
}

 void receiveEvent(int howMany) {
      while (Wire.available()) { // loop through all but the last
        {         
      received = Wire.read(); 
      if (received == 0x51);
      {
        Serial.print ("i2c in");
    }
    }
  }
}



void setup() {
 Serial.begin(115200);
 Wire.begin(I2C_SLAVE_ADDR);
 Wire.onReceive(receiveEvent); // register event
 Wire.onRequest(requestEvent);
}

void loop() {
}
