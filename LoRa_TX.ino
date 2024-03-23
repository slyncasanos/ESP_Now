/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <RH_RF95.h>

  #define RFM95_CS    18
  #define RFM95_INT   26
  #define RFM95_RST   14

#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int id;
  float x;
  float y;
  float z;
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;

// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2, board3};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].x = myData.x;
  boardsStruct[myData.id-1].y = myData.y;
  boardsStruct[myData.id-1].z = myData.z;
  boardsStruct[myData.id-2].x = myData.x;
  boardsStruct[myData.id-2].y = myData.y;

    // Update the structure with the new incoming data
  if (myData.id == 1) {
    boardsStruct[0].x = myData.x;
    boardsStruct[0].y = myData.y;
    boardsStruct[0].z = myData.z;
    Serial.printf("Board 1 - Temperature: %.2f C\n", boardsStruct[0].x);
    Serial.printf("Board 1 - Humidity: %.2f %%\n", boardsStruct[0].y);
    Serial.printf("Board 1 - Water Level: %.2f cm\n", boardsStruct[0].z);
  } else if (myData.id == 2) {
    boardsStruct[1].x = myData.x;
    boardsStruct[1].y = myData.y;
    Serial.printf("Board 2 - Flow Rate: %.2f L/min\n", boardsStruct[1].x);
    Serial.printf("Board 2 - Rain in last minute: %.2f mm\n", boardsStruct[1].y);
  } else {
    // Handle other boards if needed
    Serial.println("Unknown board ID");
  }

 /* Serial.printf("Temperature: %.2f C\n", boardsStruct[myData.id-1].x);
  Serial.printf("Humidity: %.2f % %\n", boardsStruct[myData.id-1].y);
  Serial.printf("Water Level: %.2f cm\n", boardsStruct[myData.id-1].z);
  Serial.printf("Flow Rate: %.2f L/min\n", boardsStruct[myData.id-2].x);
  Serial.printf("Rain in last minute: %.2f mm\n", boardsStruct[myData.id-2].y);*/
}
 
void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;
 
void loop() {
  Serial.println("Transmitting..."); // Send a message to rf95_server

  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;

  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len)) {
  // Acess the variables for each board
  float board1X = boardsStruct[0].x;
  float board1Y = boardsStruct[0].y;
  float board1Z = boardsStruct[0].z;
  float board2X = boardsStruct[1].x;
  float board2Y = boardsStruct[1].y;
  /*int board2X = boardsStruct[1].x;
  int board2Y = boardsStruct[1].y;
  int board3X = boardsStruct[2].x;
  int board3Y = boardsStruct[2].y;*/
  } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is there a listener around?");
  }

  delay(10000);  
}
