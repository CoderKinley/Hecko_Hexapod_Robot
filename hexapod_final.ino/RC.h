#include <SPI.h>
#include "RF24.h"

// for receiving those data
int gait_count;
int joy1x;
int joy1y;
int joy2x;
int joy2y;
int liPo1;
int liPo2;

// required function 
void RC_Setup();
bool RC_GetDataPackage();
void RC_DisplayData();
void RC_ResetData();
void SendData();
void sensor_data_send();
void initializeHexPayload();

RF24 radio(7, 8);
byte node_A_address[11] = "HexaRemote";
byte node_B_address[11] = "HexaPRobot";

void RC_Setup(){
  Serial.println(F("Receiver Starting from Robot's side..."));
  RC_ResetData();
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(node_A_address);
  radio.openReadingPipe(1, node_B_address);
  radio.startListening();
}

bool RC_GetDataPackage() {
  if (radio.available()) {
    char data_packet[48];
    radio.read(&data_packet, sizeof(data_packet));
    sscanf(data_packet, "%d,%d,%d,%d,%d,%d,%d",&gait_count, &joy1x, &joy1y, &joy2x, &joy2y, &liPo1, &liPo2);
    // RC_DisplayData();
  }
  return true;
}

void RC_DisplayData(){
    // Print received data to serial monitor
    Serial.println("Receiver Data:");
    Serial.println("----------------");
    Serial.print("Gait: "); Serial.println(gait_count);
    Serial.print("Joy1 X: "); Serial.println(joy1x);
    Serial.print("Joy1 Y: "); Serial.println(joy1y);
    Serial.print("Joy2 X: "); Serial.println(joy2x);
    Serial.print("Joy2 Y: "); Serial.println(joy2y);
    Serial.print("LiPo1: "); Serial.println(liPo1);
    Serial.print("LiPo2: "); Serial.println(liPo2);
    Serial.println();
}

void sensor_data_send(){
  Serial.println("This is the Ultimate sensor data...");
}

void RC_ResetData(){
  gait_count = 0;
  joy1x = 531;
  joy1y = 533;
  joy2x = 537;
  joy2y = 544;
  liPo1 = 1024;
  liPo2 = 0;
}