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
#include "DHT.h"

#define DHTPIN 4 
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define echoPin 18 // attach pin GPIO18 to pin Echo of JSN-SR04
#define trigPin 5  // attach pin GPIO5 ESP32 to pin Trig of JSN-SR04                     

long duration; // Time taken for the pulse to reach the receiver
int distance; 

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x10, 0x52, 0x1C, 0x64, 0xEF, 0x6C};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    float x;
    float y;
    float z;
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
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  dht.begin();

  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

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

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); 

  digitalWrite(trigPin, HIGH); // turn on the Trigger to generate pulse
  delayMicroseconds(10); // keep the trigger "ON" for 10 ms to generate pulse
  digitalWrite(trigPin, LOW); // Turn off the pulse trigger to stop pulse

  // If pulse reached the receiver echoPin
  // become high Then pulseIn() returns the
  // time taken by the pulse to reach the receiver
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.0344 / 2; 

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(100);

  // Set values to send
  myData.id = 1;
  myData.x = t;
  myData.y = h;
  myData.z = distance;

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
