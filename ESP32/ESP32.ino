#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
HardwareSerial SerialB(2); // A9: TX, A10: RX
void setup(){
  Serial.begin(9600);
  SerialB.begin(9600);
}
void loop(){    
  if (SerialB.available() > 0 )
  {
    // Read from  STM module and send to serial monitor
    String input = SerialB.readString();
    DynamicJsonDocument jsonDocCG(128); // Kích thước buffer là 128 byte
    // Phân tích chuỗi JSON
    deserializeJson(jsonDocCG, input);
    // Lấy giá trị nhiệt độ và độ ẩm
    float t = jsonDocCG["t"];
    float h = jsonDocCG["h"];
    float lightIntensity = jsonDocCG["lightIntensity"];
    Serial.println(input);
    Serial.print("Temperature: ");
    Serial.println(t);
    Serial.print("Humidity: ");
    Serial.println(h);
    Serial.print("lightIntensity: ");
    Serial.println(lightIntensity);
  }
  delay(20);
}
