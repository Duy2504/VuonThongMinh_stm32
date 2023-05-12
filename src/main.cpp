#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>            
#include <LiquidCrystal_I2C.h>                 
#include <STM32FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>
#include <FreeRTOS.h>

HardwareSerial Serial_Mon(PA10, PA9);
// HardwareSerial Serial_Third(PA2, PA3);

#define LDR_PIN   PA1
#define DHT11_PIN1 PB4    // Chân GPIO được sử dụng để kết nối với cảm biến DHT
#define DHT11_PIN2 PB1    // Chân GPIO được sử dụng để kết nối với cảm biến DHT
#define DHTTYPE DHT11     // Loại cảm biến DHT (DHT11 hoặc DHT22)
#define I2C_ADDR 0x27           
#define LCD_COLS 20             
#define LCD_ROWS 4   
// Khai báo các chân và biến của led 1
const int led1Pin = PA6;
const int buttonPin1 = PA7;
int buttonState1 = 0;
int lastButtonState1 = 0;
// Khai báo chân và biến của led 2
const int led2Pin = PA0;
const int button2Pin = PA4;
int button2State = 0;
int lastButton2State = 0;
 
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLS, LCD_ROWS);          
SemaphoreHandle_t lcdMutex;
TimerHandle_t dhtTimer;
DHT dht1(DHT11_PIN1, DHTTYPE);
DHT dht2(DHT11_PIN2,DHTTYPE);

SemaphoreHandle_t dht_semaphore;
float tempValue, humidValue;

int ldrValue;
float voltage;
float lightIntensity;

void readDhtTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
  if(xSemaphoreTake(dht_semaphore, portMAX_DELAY) == pdTRUE) {
      humidValue= dht1.readHumidity();
      tempValue = dht1.readTemperature();
      ldrValue = analogRead(LDR_PIN);
      voltage = ldrValue * (5.0 / 1023.0);
      lightIntensity = 10 / voltage;
      xSemaphoreGive(dht_semaphore);
  }
   vTaskDelay(pdMS_TO_TICKS(2000));
}
}
void displayLCD(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if(xSemaphoreTake(dht_semaphore, portMAX_DELAY) == pdTRUE) {
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");                
      lcd.print(tempValue);
      lcd.print("*C");                
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");             
      lcd.print(humidValue);
      lcd.print("%");
      lcd.setCursor(0,2);
      lcd.print("light: ");
      lcd.print(lightIntensity);
      xSemaphoreGive(dht_semaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
// bool ledState1 = digitalRead(led1Pin);
// bool ledState2 = digitalRead(led2Pin);
void sendData(void *pvParameters){
    (void) pvParameters;
    HardwareSerial Serial_Mon(PA10, PA9);
    Serial_Mon.begin(115200);
    for (;;)
    {
      // if(xSemaphoreTake(dht_semaphore, portMAX_DELAY) == pdTRUE){
      xSemaphoreTake(dht_semaphore, portMAX_DELAY);
       // Định dạng chuỗi JSON
      DynamicJsonDocument jsonDocSS(128);
      // jsonDocSS["led1"] = ledState1;
      // jsonDocSS["led2"] = ledState2;
      jsonDocSS["tempValue"] = 1.00*tempValue;
      jsonDocSS["humidValue"] = 1.00*humidValue;
      jsonDocSS["lightIntensity"] = lightIntensity;
      String jsonStringSS;
      serializeJson(jsonDocSS, jsonStringSS);
      Serial_Mon.println(jsonStringSS);

      if (Serial_Mon.available() > 0)
      {
    // Read from  STM module and send to serial monitor
    String input = Serial_Mon.readString();
    DynamicJsonDocument jsonDocSR(128); // Kích thước buffer là 128 byte
    deserializeJson(jsonDocSR, input);
    bool led1 = jsonDocSR["led1"];
    digitalWrite(led1Pin, (led1 == 1 ) ? HIGH : LOW);
    
    bool led2 = jsonDocSR["led2"];
    digitalWrite(led2Pin, (led2 == 1 ) ? HIGH : LOW);
    }

      xSemaphoreGive(dht_semaphore);
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Gửi lại sau 1 giây
      }
}
void receiveData(void *pvParameters){
    (void) pvParameters;
    pinMode(led1Pin,OUTPUT);
    pinMode(buttonPin1,INPUT_PULLUP);
    pinMode(led2Pin,OUTPUT);
    pinMode(button2Pin,INPUT_PULLUP);
    for (;;)
    {
       if (Serial_Mon.available() > 0)
      {
    // Read from  STM module and send to serial monitor
    String input = Serial_Mon.readString();
    DynamicJsonDocument jsonDocSR(128); // Kích thước buffer là 128 byte
    deserializeJson(jsonDocSR, input);
    bool ledState1 = jsonDocSR["ledState1"];
    if(ledState1 ==1){
      digitalWrite(led1Pin ,HIGH);
    }
    else{
      digitalWrite(led1Pin,LOW);
    }
    // digitalWrite(led1Pin, (led1 == 1 ) ? HIGH : LOW);
    
    bool ledState2 = jsonDocSR["ledState2"];
    if(ledState2 ==1){
      digitalWrite(led2Pin,HIGH);
    }
    else{
      digitalWrite(led2Pin,LOW);
    }
    // digitalWrite(led2Pin, (led2 == 1 ) ? HIGH : LOW);
    }
  }      
}
// void task3(void *pvParameters){
//   (void) pvParameters;
//     pinMode(led1Pin, OUTPUT);
//     pinMode(buttonPin1, INPUT_PULLUP);
//     for (;;)
//     {
//         /*------------LED1------------------------*/
//          buttonState1 = digitalRead(buttonPin1);
//         if (buttonState1 != lastButtonState1) {
//             if (buttonState1 == LOW) {
//              digitalWrite(led1Pin, !digitalRead(led1Pin)); // toggle LED state

//          }
//             lastButtonState1 = buttonState1;
//              delay(50); // debounce delay
//         }
//     }
// }
// void task4(void *pvParameters){
//     (void) pvParameters;
//     pinMode(led2Pin,OUTPUT);
//     pinMode(button2Pin,INPUT_PULLUP);
//     for (;;)
//     {
//        /*----------LED 2---------------------*/
//         button2State = digitalRead(button2Pin);
//         if (button2State != lastButton2State) {
//             if (button2State == LOW) {
//              digitalWrite(led2Pin, !digitalRead(led2Pin)); // toggle LED state
//          }
//             lastButton2State = button2State;
//              delay(50); // debounce delay
//         }
//     }      
// }

void setup() {
  Serial_Mon.begin(115200);

  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  digitalWrite(led1Pin, LOW);
  digitalWrite(led2Pin, LOW);
  dht1.begin();
  dht2.begin();
  lcd.init();                  
  lcd.backlight();              
  lcd.setCursor(0, 0);
  lcd.print("Vuon Thong Minh");
  delay(1000);
  lcd.clear();
  dht_semaphore = xSemaphoreCreateCounting(1, 1);
  xTaskCreate(readDhtTask,"readDhtTask",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY + 4,NULL);
  xTaskCreate(displayLCD, "DisplayLCD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(sendData,"sendData",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY + 2,NULL);
  xTaskCreate(receiveData,"receiveData",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY ,NULL);
  // xTaskCreate(task3,"Task3",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY ,NULL);
  // xTaskCreate(task4,"Task4",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY ,NULL);

  vTaskStartScheduler();
}

void loop() {

}