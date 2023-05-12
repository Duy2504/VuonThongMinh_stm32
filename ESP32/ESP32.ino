#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "MQ135.h"
#include <SPI.h>
HardwareSerial SerialB(2); 

const char *ssid = "wifi";
const char *password = "12345678";
const char *mqtt_server = "192.168.43.105";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

// MQ135 gasSensor = MQ135(35);
MQ135 mq135_sensor(35);
int val;
int sensorPin = 35;
int sensorValue = 0;

#define DHTPIN 14
#define DHTTYPE DHT11
#define BUTTON1 25
#define BUTTON2 26
#define light_sensor 33 
/////////
// String ledStatus1 = "on";
// String ledStatus2 = "on";

DHT dht(DHTPIN, DHTTYPE);
// char myStr[20];
int ledStart1;
int ledStart2;
DynamicJsonDocument jsonDocES(128);
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  /*Feel free to add more if statements to control more GPIOs with MQTT
  If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  Changes the output state according to the message*/
  Serial.println(topic);
  Serial.println(messageTemp);
  if (String(topic) == "buttona") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      // ledStatus1 = "on";
      ledStart1 = 1;
      Serial.println("on");
      digitalWrite(BUTTON1, HIGH);
    }
    else if(messageTemp == "off"){
      // ledStatus1 = "off";
      ledStart1 = 0;
      Serial.println("off");
      digitalWrite(BUTTON1, LOW);
    }
  }
  if (String(topic) == "buttonb") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      // ledStatus2 = "on";
      ledStart2 = 1;
      Serial.println("on");
      digitalWrite(BUTTON2, HIGH);
    }
    else if(messageTemp == "off"){
      // ledStatus2 = "off";
      ledStart2 = 0;
      Serial.println("off");
      digitalWrite(BUTTON2, LOW);
    }
  }
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32uClient-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      client.publish("dataSensor", "Connected");
      client.subscribe("buttona");
      client.subscribe("buttonb");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}

void setup()
{
  SerialB.begin(9600);
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dht.begin();
  Wire.begin();
  pinMode(sensorPin, INPUT);
  client.subscribe("buttona");
  client.subscribe("buttonb");
  pinMode(BUTTON1, OUTPUT); 
  pinMode(BUTTON2, OUTPUT);
}

  float tempSTM;
  float humidSTM;
  float lightSTM;
  float gasSTM;
void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  jsonDocES["led1"] = (int)ledStart1;
  jsonDocES["led2"] = (int)ledStart2;
  String jsonStringES;
  
  if (SerialB.available() > 0 )
  {
    String input = SerialB.readString();
    DynamicJsonDocument jsonDocER(128);
    // Phân tích chuỗi JSON
    deserializeJson(jsonDocER, input);
    // Lấy giá trị nhiệt độ và độ ẩm
    tempSTM = jsonDocER["tempValue"];
    humidSTM = jsonDocER["humidValue"];
    lightSTM = jsonDocER["lightIntensity"];
    gasSTM = jsonDocER["gasValue"];
    // Serial.println(input);
    // Serial.print("Temperature: ");
    // Serial.println(tempSTM);
    // Serial.print("Humidity: ");
    // Serial.println(humidSTM);
    // Serial.print("lightIntensity: ");
    // Serial.println(lightSTM);
  }
  unsigned long now = millis();
  if (now - lastMsg > 2000)
  {
    serializeJson(jsonDocES, jsonStringES);
    SerialB.println(jsonStringES);
    Serial.println(jsonStringES);
    // int temp = dht.readTemperature();
    int temp = tempSTM;

    Serial.print("Nhiet do: ");
    Serial.print(temp);
    Serial.println("℃");

    // int humid = dht.readHumidity();
    int humid = humidSTM;
    Serial.print("Do am: ");
    Serial.print(humid);
    Serial.println("%");
    // int lightRaw = analogRead(light_sensor);
    // int light =100 - map(lightRaw, 0, 4095, 0, 100);
    int light = (int)lightSTM;
    Serial.print("Anh sang: ");
    Serial.print(light);
    Serial.println("%");
    // int dustppm = mq135_sensor.getPPM();
    // int dust = dustppm/10000;
    int dust = gasSTM;
    // int dust = mq135_sensor.getResistance();
    Serial.print("Dust: ");
    Serial.print(dust);
    Serial.println("ppm");
    String ssData =String(temp) + "," + String(humid) + "," + String(light) + "," + String(dust);
    client.publish("dataSensor", ssData.c_str());
    Serial.print("Pushed Data: ");
    Serial.println(ssData);
    Serial.println("//////////////////////");
    lastMsg = now;
  }
}
