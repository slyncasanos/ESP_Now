#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS    18
#define RFM95_INT   26
#define RFM95_RST   14

#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

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

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      updateBoardsStruct();
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Receive failed");
    }
  }
}

void updateBoardsStruct() {
  boardsStruct[myData.id - 1].x = myData.x;
  boardsStruct[myData.id - 1].y = myData.y;
  boardsStruct[myData.id - 1].z = myData.z;
  Serial.printf("Board 1 - Temperature: %.2f C\n", boardsStruct[0].x);
  Serial.printf("Board 1 - Humidity: %.2f %%\n", boardsStruct[0].y);
  Serial.printf("Board 1 - Water Level: %.2f cm\n", boardsStruct[0].z);
  boardsStruct[myData.id - 2].x = myData.x;
  boardsStruct[myData.id - 2].y = myData.y;
  Serial.printf("Board 2 - Flow Rate: %.2f L/min\n", boardsStruct[1].x);
  Serial.printf("Board 2 - Rain in last minute: %.2f mm\n", boardsStruct[1].y);
}
