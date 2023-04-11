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
//HardwareSerial Serial3(PB10, PB11);//tx,rx
HardwareSerial Serial_Mon(PA10, PA9);

#define LDR_PIN   PA1
#define DHT11_PIN PA2    // Chân GPIO được sử dụng để kết nối với cảm biến DHT
#define DHTTYPE DHT11  // Loại cảm biến DHT (DHT11 hoặc DHT22)
#define I2C_ADDR 0x27           
#define LCD_COLS 20             
#define LCD_ROWS 4   
// Khai báo các chân và biến của led 1
const int ledPin1 = PA6;
const int buttonPin1 = PA7;
int buttonState1 = 0;
int lastButtonState1 = 0;
// Khai báo chân và biến của led 2
const int led2Pin = PA3;
const int button2Pin = PA4;
int button2State = 0;
int lastButton2State = 0;
 
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLS, LCD_ROWS);          
SemaphoreHandle_t lcdMutex;
TimerHandle_t dhtTimer;
DHT dht(DHT11_PIN, DHTTYPE);

SemaphoreHandle_t dht_semaphore;
float t, h;
int ldrValue;
float voltage;
float lightIntensity;

void readDhtTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
  if(xSemaphoreTake(dht_semaphore, portMAX_DELAY) == pdTRUE) {
      h= dht.readHumidity();
      t = dht.readTemperature();
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
      lcd.print(t);
      lcd.print("*C");                
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");             
      lcd.print(h);
      lcd.print("%");
      lcd.setCursor(0,2);
      lcd.print("light: ");
      lcd.print(lightIntensity);
      xSemaphoreGive(dht_semaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void sendData(void *pvParameters){
    (void) pvParameters;
    HardwareSerial Serial_Mon(PA10, PA9);
    Serial_Mon.begin(9600);
    for (;;)
    {
      // if(xSemaphoreTake(dht_semaphore, portMAX_DELAY) == pdTRUE){
      xSemaphoreTake(dht_semaphore, portMAX_DELAY);
       // Định dạng chuỗi JSON
      DynamicJsonDocument jsonDocSS(128);
      jsonDocSS["t"] = 1.00*t;
      jsonDocSS["h"] = 1.00*h;
      jsonDocSS["lightIntensity"] = lightIntensity;
      String jsonStringSS;
      serializeJson(jsonDocSS, jsonStringSS);
      Serial_Mon.println(jsonStringSS);
      xSemaphoreGive(dht_semaphore);
      vTaskDelay(5000 / portTICK_PERIOD_MS);  // Gửi lại sau 5 giây
      }
}
void task3(void *pvParameters){
  (void) pvParameters;
    pinMode(ledPin1, OUTPUT);
    pinMode(buttonPin1, INPUT_PULLUP);
    for (;;)
    {
        /*------------LED1------------------------*/
         buttonState1 = digitalRead(buttonPin1);
        if (buttonState1 != lastButtonState1) {
            if (buttonState1 == LOW) {
             digitalWrite(ledPin1, !digitalRead(ledPin1)); // toggle LED state

         }
            lastButtonState1 = buttonState1;
             delay(50); // debounce delay
        }
    }
}
void task4(void *pvParameters){
    (void) pvParameters;
    pinMode(led2Pin,OUTPUT);
    pinMode(button2Pin,INPUT_PULLUP);
    for (;;)
    {
       /*----------LED 2---------------------*/
        button2State = digitalRead(button2Pin);
        if (button2State != lastButton2State) {
            if (button2State == LOW) {
             digitalWrite(led2Pin, !digitalRead(led2Pin)); // toggle LED state
         }
            lastButton2State = button2State;
             delay(50); // debounce delay
        }
    }      
}

void setup() {
  Serial_Mon.begin(9600);

  pinMode(ledPin1, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  digitalWrite(ledPin1, LOW);
  digitalWrite(led2Pin, LOW);
  dht.begin();
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
  xTaskCreate(task3,"Task3",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY ,NULL);
  xTaskCreate(task4,"Task4",configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY ,NULL);

  vTaskStartScheduler();
}

void loop() {

}

