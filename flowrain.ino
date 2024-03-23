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
#include "RTClib.h"
#include <Wire.h>

#define SENSOR  27
#define RainPin 2    

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

bool bucketPositionA = false;             // one of the two positions of tipping-bucket               
const double bucketAmount = 0.4090909;   // inches equivalent of ml to trip tipping-bucket
double dailyRain = 0.0;                   // rain accumulated for the day
double minuteRain = 0.0;                  // rain accumulated for one hour
double dailyRain_till_LastMinute = 0.0;     // rain accumulated for the day till the last hour          
bool first;                               // as we want readings of the (MHz) loops only at the 0th moment 

RTC_Millis rtc;  

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    float x;
    float y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  rtc.begin(DateTime(__DATE__, __TIME__));       // start the RTC
  pinMode(RainPin, INPUT);  

  pinMode(SENSOR, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {

  DateTime now = rtc.now();

// ++++++++++++++++++++++++ Count the bucket tips ++++++++++++++++++++++++++++++++
  if ((bucketPositionA==false)&&(digitalRead(RainPin)==HIGH)){
    bucketPositionA=true;
    dailyRain+=bucketAmount;                               // update the daily rain
  }
  
  if ((bucketPositionA==true)&&(digitalRead(RainPin)==LOW)){
    bucketPositionA=false;  
  } 

  if(now.second() != 0) first = true;                     // after the first minute is over, be ready for next read

  currentMillis = millis();
  if (currentMillis - previousMillis > interval || now.second() == 0 && first == true) {

    pulse1Sec = pulseCount;
    pulseCount = 0;

    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(float(flowRate));  // Print the integer part of the variable
    Serial.println(" L/min");

    minuteRain = dailyRain - dailyRain_till_LastMinute;      // calculate the last hour's rain
    dailyRain_till_LastMinute = dailyRain;                   // update the rain till last hour for next calculation

    // facny display for humans to comprehend
    Serial.print("Rain in last minute = ");
    Serial.print(minuteRain,2);
    Serial.println(" mm");
    
    first = false;                                        // execute calculations only once per hour
    
    if(now.minute()== 0) {
    dailyRain = 0.0;                                      // clear daily-rain at midnight
    dailyRain_till_LastMinute = 0.0;                        // we do not want negative rain at 01:00
  }

  // Set values to send
  myData.id = 2;
  myData.x = flowRate;
  myData.y = minuteRain;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(10000);

  // Deep sleep for 10 seconds using timer
  esp_sleep_enable_timer_wakeup(10 * 1000000); // 10 seconds
  esp_deep_sleep_start();
}
}
