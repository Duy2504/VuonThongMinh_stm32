#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>

HardwareSerial SerialA(0); // TX:GPIO1 RX:GPIO3
HardwareSerial SerialB(2); // TX:GPIO17 RX:GPIO16
const char *ssid = "iPhone";
const char *password = "1234567890";
const char *mqtt_server = "172.20.10.8";

WiFiClient espClient;
PubSubClient client(espClient);
int value = 0;
int ledState1;
int ledState2;
DynamicJsonDocument jsonDocES(128);
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

#define sub1 "led1"
#define sub2 "led2"
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  if (strstr(topic, sub1)) {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    if ((char)payload[0] == '1') {
      ledState1 = 1;
      Serial.println("on");
    } else {
      ledState1 = 0;
      Serial.println("off");
    }
  } else if (strstr(topic, sub2)) {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    if ((char)payload[0] == '1') {
       ledState2 = 1;
       Serial.println("on");
    } else {
      ledState2 = 0;
      Serial.println("off");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      client.subscribe(sub1);
      client.subscribe(sub2);
    } else {
      Serial.println(" try again in 2 seconds");
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup() {
  SerialB.begin(115200);
  SerialA.begin(115200);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Wire.begin();
}

  float tempSTM;
  float humidSTM;
  float lightSTM;   
void loop() {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
      jsonDocES["led1"] = (int)ledState1;
      jsonDocES["led2"] = (int)ledState2;
      String jsonStringES;
      serializeJson(jsonDocES, jsonStringES);
      SerialB.println(jsonStringES);
      
       if (SerialB.available() > 0 )
           {
              // Read from  STM module and send to serial monitor
              String input = SerialB.readString();
              DynamicJsonDocument jsonDocER(128); // Kích thước buffer là 128 byte
              // Phân tích chuỗi JSON
              deserializeJson(jsonDocER, input);
              // Lấy giá trị nhiệt độ và độ ẩm
              tempSTM = jsonDocER["tempValue"];
              humidSTM = jsonDocER["humidValue"];
              lightSTM = jsonDocER["lightIntensity"];
                }
        float temp = tempSTM;
        float humi = humidSTM;
        int light = lightSTM;
        int state_1 = ledState1;
        int state_2 = ledState2;
        DynamicJsonDocument jsonDoc(128);
        jsonDoc["temperature"] = temp;
        jsonDoc["humidity"] = humi;
        jsonDoc["light"] = light;
        jsonDoc["state_1"] = state_1;
        jsonDoc["state_2"] = state_2;
        Serial.println(state_1);
        Serial.println(state_2);
        Serial.println();
        Serial.println(temp);
        Serial.println(humi);
        Serial.println(light);
        String jsonString;
        serializeJson(jsonDoc, jsonString);
        client.publish("sensor", jsonString.c_str());
        delay(1000);
        }
