#include <Wire.h>
#include <SoftwareSerial.h>
#define I2C_SLAVE_ADDR  0x70  //Standard MaxsonarI2CXL address

const int numReadings = 10;

int pinRX = 10;
int pinTX = 11;
SoftwareSerial mySerial(pinRX, pinTX);
unsigned char data_buffer[4] = {0};
int distances = 0;
int distance = 0;
unsigned char CS;
uint8_t Index;
byte received;



int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average



void readsonar(){     
   if (mySerial.available() > 0) {
    delay(4);
 
    // Check for packet header character 0xff
    if (mySerial.read() == 0xff) {
      
      // Insert header into array
      data_buffer[0] = 0xff;
      
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = mySerial.read();
      }
 
      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];
      // If checksum is valid compose distance from data
      if (data_buffer[3] == CS) {
        distances = 0.1 * (data_buffer[1] << 8) + data_buffer[2];
      
       // Smoothing
         
       // subtract the last reading:
        total = total - readings[readIndex];
       // read from the sensor:
         readings[readIndex] = distances;
       // add the reading to the total:
         total = total + readings[readIndex];
       // advance to the next position in the array:
        readIndex = readIndex + 1;
       // if we're at the end of the array...
    if (readIndex >= numReadings) {
       // ...wrap around to the beginning:
       readIndex = 0;
  }

      // calculate the average:
      average = total / numReadings;

      //ratio for speed of sound in air vs water
       distance = average * 4.126;
       }
    }
  }
}



/*
 The default I2C-Address of the sensor is 0x70. 
 To perform a range measurement you must send the "Take Range Reading” command byte 0x5
 */
void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last
        {         
      received = Wire.read(); 
      if (received == 0x51)
      {
       mySerial.write(0x55);
      }
    }
  }
}

void requestEvent() 
{
Wire.write (highByte(distance));
Wire.write (lowByte(distance));
}

void setup() {
 Serial.begin(9600);
 mySerial.begin(9600);
 Wire.begin(I2C_SLAVE_ADDR);
 Wire.onReceive(receiveEvent); // register event
 Wire.onRequest(requestEvent);
 for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}
void loop() {
  readsonar();
  Serial.print(distance);
}
