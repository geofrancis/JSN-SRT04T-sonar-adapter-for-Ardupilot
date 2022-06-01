#include <Wire.h>
#include <SoftwareSerial.h>

const int numReadings = 10;



#define I2C_ADDR1  0x70 
#define I2C_ADDR2  0x71  
#define I2C_ADDR3  0x72  

SoftwareSerial portOne(10, 11);
SoftwareSerial portTwo(8, 9);



//sonar 1////////////////////////////////////////////////////////////////
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

//Sonar 2//////////////////////////////////////////////////////////////////////

unsigned char data_buffer2[4] = {0};
int distances2 = 0;
int distance2 = 0;
unsigned char CS2;
uint8_t Index2;
byte received2;
int readings2[numReadings];      // the readings from the analog input
int readIndex2 = 0;              // the index of the current reading
int total2 = 0;                  // the running total
int average2 = 0;                // the average

//sonar 3 //////////////////////////////////////////////////////////////////////////


unsigned char data_buffer3[4] = {0};
int distances3 = 0;
int distance3 = 0;
unsigned char CS3;
uint8_t Index3;
byte received3;
int readings3[numReadings];      // the readings from the analog input
int readIndex3 = 0;              // the index of the current reading
int total3 = 0;                  // the running total
int average3 = 0;                // the average


///////////////////////////////////////////////////////////////////////////////////////////
void readsonar1(){     

    portOne.listen();
    while (portOne.available() > 0) {
    portOne.write(0x55);
    delay(20);
 
    // Check for packet header character 0xff
    if (portOne.read() == 0xff) {
      
      // Insert header into array
      data_buffer[0] = 0xff;
      
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = portOne.read();
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
       //portOne.write(0xff);
      }
    }
  }
}

void requestEvent() 
{
Wire.write (highByte(distance));
Wire.write (lowByte(distance));
}




/////////////////////////////////////////////////////////////////////////////////////////
void readsonar2(){  
     
    portTwo.listen();
    while (portTwo.available() > 0) {
    portTwo.write(0x55);
    delay(4);
 
    // Check for packet header character 0xff
    if (portTwo.read() == 0xff) {
      
      // Insert header into array
      data_buffer2[0] = 0xff;
      
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer2[i] = portTwo.read();
      }
 
      //Compute checksum
      CS = data_buffer2[0] + data_buffer2[1] + data_buffer2[2];
      // If checksum is valid compose distance from data
      if (data_buffer2[3] == CS2) {
        distances2= 0.1 * (data_buffer2[1] << 8) + data_buffer2[2];
      
       // Smoothing
         
       // subtract the last reading:
        total2 = total2 - readings2[readIndex];
       // read from the sensor:
         readings2[readIndex] = distances2;
       // add the reading to the total:
         total2 = total2 + readings2[readIndex];
       // advance to the next position in the array:
        readIndex2 = readIndex2 + 1;
       // if we're at the end of the array...
    if (readIndex2 >= numReadings) {
       // ...wrap around to the beginning:
       readIndex2 = 0;
  }

      // calculate the average:
      average2 = total2 / numReadings;

      //ratio for speed of sound in air vs water
       distance2 = average2 * 4.126;
       }
    }
  }
}



void receiveEvent2(int howMany2) {
  while (Wire.available()) { // loop through all but the last
        {         
      received = Wire.read(); 
      if (received == 0x51)
      {
       //portTwo.write(0xff);
      }
    }
  }
}

void requestEvent2() 
{
Wire.write (highByte(distance2));
Wire.write (lowByte(distance2));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

void readsonar3(){     

    if (Serial.available() > 0) {
    Serial.write(0x55);
    delay(4);
 
    // Check for packet header character 0xff
    if (Serial.read() == 0xff) {
      
      // Insert header into array
      data_buffer3[0] = 0xff;
      
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer3[i] = Serial.read();
      }
 
      //Compute checksum
      CS = data_buffer3[0] + data_buffer3[1] + data_buffer3[2];
      // If checksum is valid compose distance from data
      if (data_buffer3[3] == CS3) {
        distances3 = 0.1 * (data_buffer3[1] << 8) + data_buffer3[2];
      
       // Smoothing
         
       // subtract the last reading:
        total3 = total3 - readings3[readIndex];
       // read from the sensor:
         readings3[readIndex] = distances3;
       // add the reading to the total:
         total3 = total3 + readings3[readIndex];
       // advance to the next position in the array:
        readIndex3 = readIndex3 + 1;
       // if we're at the end of the array...
    if (readIndex3 >= numReadings) {
       // ...wrap around to the beginning:
       readIndex3 = 0;
  }

      // calculate the average:
      average3 = total3 / numReadings;

      //ratio for speed of sound in air vs water
       distance3 = average3 * 4.126;
       }
    }
  }
}



/*
 The default I2C-Address of the sensor is 0x70. 
 To perform a range measurement you must send the "Take Range Reading” command byte 0x5
 */
void receiveEvent3(int howMany3) {
  while (Wire.available()) { // loop through all but the last
        {         
      received3 = Wire.read(); 
      if (received3 == 0x51)
      {
       //Serial.write(0xff);
      }
    }
  }
}

void requestEvent3() 
{
Wire.write (highByte(distance3));
Wire.write (lowByte(distance3));
}



////////////////////////////////////////////////////////////////////
void setup() {
 Serial.begin(9600);
 portOne.begin(9600);
 portTwo.begin(9600);
 Wire.begin(I2C_ADDR1);
 Wire.begin(I2C_ADDR2);
 Wire.begin(I2C_ADDR3);
 Wire.onReceive(receiveEvent); // register event
 Wire.onRequest(requestEvent);
 for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  for (int thisReading2 = 0; thisReading2 < numReadings; thisReading2++) {
    readings2[thisReading2] = 0;
  for (int thisReading3 = 0; thisReading3 < numReadings; thisReading3++) {
    readings3[thisReading3] = 0;
   }
  }
 }
}

/////////////////////////////////////////////////////////////////////////
void loop() {
  readsonar1();
  readsonar2();
  readsonar3();

}
