// NEEDS RP-2040

#include <SoftwareSerial.h>
#include "mavlink/common/mavlink.h"  // Mavlink interface
#include "mavlink/common/mavlink_msg_distance_sensor.h"


unsigned char buffer_RTT[4] = { 0 };
uint8_t CS;
#define COM 0x55


#define JSNSRT04 9600
//#define GL041MT 115200

#define sonar1_orientation 25
#define sonar2_orientation 25
#define sonar3_orientation 25
#define sonar4_orientation 25

int Distance1 = 0;
int Distance2 = 0;
int Distance3 = 0;
int Distance4 = 0;

int sysid = 1;
int compid = 196;
uint32_t time_boot_ms = 0;   /*< Time since system boot*/
uint16_t min_distance = 5;   /*< Minimum distance the sensor can measure in centimeters*/
uint16_t max_distance = 600; /*< Maximum distance the sensor can measure in centimeters*/
uint8_t type = 0;            /*< Type from MAV_DISTANCE_SENSOR enum.*/
uint8_t covariance = 0;      /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/
float horizontal_fov = 0;
float vertical_fov = 0;
const float* signal_quality = 0;
uint8_t quaternion = 0;


SerialPIO sonar1(2, 3);
SerialPIO sonar2(4, 5);
SerialPIO sonar3(6, 7);
SerialPIO sonar4(10, 11);

void setup() {
  Serial.begin(115200);   // USB
  Serial1.begin(115200);  // 0,1
  //  Serial2.begin(115200);  // 8,9

  sonar1.begin(115200);
  sonar2.begin(115200);
  sonar3.begin(115200);
  sonar4.begin(115200);
}

void loop() {

  ping_sonar();
  read_sonar1();
  read_sonar2();
  read_sonar3();
  read_sonar4();
  printvalue();
}

void ping_sonar() {
  sonar1.write(COM);
  sonar2.write(COM);
  sonar3.write(COM);
  sonar4.write(COM);
  delay(100);
}

void read_sonar1() {
  if (sonar1.available() > 0) {
    delay(4);
    if (sonar1.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = sonar1.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        Distance1 = (buffer_RTT[1] << 8) + buffer_RTT[2];
        /*< Type from MAV_DISTANCE_SENSOR enum.*/
        uint8_t id = 1;                           /*< Onboard ID of the sensor*/
        uint8_t orientation = sonar1_orientation; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance1, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial1.write(buf, len);
      }
    }
  }
}


void read_sonar2() {
  if (sonar2.available() > 0) {
    delay(4);
    if (sonar2.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = sonar2.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        Distance2 = (buffer_RTT[1] << 8) + buffer_RTT[2];
        /*< Type from MAV_DISTANCE_SENSOR enum.*/
        uint8_t id = 2;           /*< Onboard ID of the sensor*/
        uint8_t orientation = 25; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance2, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial1.write(buf, len);
      }
    }
  }
}





void read_sonar3() {
  if (sonar3.available() > 0) {
    delay(4);
    if (sonar3.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = sonar3.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        Distance3 = (buffer_RTT[1] << 8) + buffer_RTT[2];
        /*< Type from MAV_DISTANCE_SENSOR enum.*/
        uint8_t id = 3;                           /*< Onboard ID of the sensor*/
        uint8_t orientation = sonar2_orientation; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance3, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial1.write(buf, len);
      }
    }
  }
}





void read_sonar4() {
  if (sonar4.available() > 0) {
    delay(4);
    if (sonar4.read() == 0xff) {
      buffer_RTT[0] = 0xff;
      for (int i = 1; i < 4; i++) {
        buffer_RTT[i] = sonar4.read();
      }
      CS = buffer_RTT[0] + buffer_RTT[1] + buffer_RTT[2];
      if (buffer_RTT[3] == CS) {
        Distance4 = (buffer_RTT[1] << 8) + buffer_RTT[2];
        /*< Type from MAV_DISTANCE_SENSOR enum.*/
        uint8_t id = 14;                          /*< Onboard ID of the sensor*/
        uint8_t orientation = sonar4_orientation; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_distance_sensor_pack(sysid, compid, &msg, time_boot_ms, min_distance, max_distance, Distance4, type, id, orientation, covariance, horizontal_fov, vertical_fov, signal_quality, quaternion);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial1.write(buf, len);
      }
    }
  }
}





void printvalue() {
  Serial.print("Distance1:");
  Serial.print(Distance1);
  Serial.print("mm");
  Serial.print("   ");
  Serial.print("Distance2:");
  Serial.print(Distance2);
  Serial.print("mm");
  Serial.print("   ");
  Serial.print("Distance3:");
  Serial.print(Distance3);
  Serial.print("mm");
  Serial.print("   ");
  Serial.print("Distance4:");
  Serial.print(Distance4);
  Serial.print("mm");
  Serial.print("   ");
}
