#include <HardwareSerial.h>

#define RX_PIN 16
#define TX_PIN 17
#define BUTTON_PIN 18

HardwareSerial mySerial(1); 

QueueHandle_t myQueue;
volatile bool buttonPressed = false;

void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

void readTask(void *parameter) {
  String data = "";
  while (true) {
    while (mySerial.available() > 0) {
      char ch = mySerial.read();
      data += ch;
    }
    if (data.length() > 0) {
      xQueueSend(myQueue, &data, portMAX_DELAY);
      data = "";
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void MotorTask(void *parameter) {
  String data = "";
  while (true) {
    if (buttonPressed) {
      int numMessages = uxQueueMessagesWaiting(myQueue);
      for (int i = 0; i < numMessages; i++) {
        xQueueReceive(myQueue, &data, 0);
        // delete whitespace
        data.trim();
        if (data == "1") {
          forward();
         
        } else if (data == "2") {
          backward();
         
        } 
        data = "";
      }
      buttonPressed = false; 
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void LCDTask(void *parameter) {
  String data = "";
  while (true) {
    if (buttonPressed) {
      int numMessages = uxQueueMessagesWaiting(myQueue);
      for (int i = 0; i < numMessages; i++) {
        xQueueReceive(myQueue, &data, 0);
        // delete whitespace
        data.trim();
        if (data == "3") {
          sword();
         
        } else if (data == "4") {
          giant();
         
        } 
        data = "";
      }
      buttonPressed = false; 
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void forward() {
  Serial.println("Moving Forward");
  delay(5000);
  Serial.println("Moving Forward");
}

void backward() {
  Serial.println("Moving Backward");
}


void sword() {
  Serial.println("Sword");
  delay(3000);
  Serial.println("Sword222");
}

void giant() {
  Serial.println("Giant");
}

void setup() {
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  myQueue = xQueueCreate(10, sizeof(String));

  xTaskCreate(readTask, "readTask", 10000, NULL, 1, NULL);
  xTaskCreate(MotorTask, "MotorTask", 10000, NULL, 1, NULL);
  xTaskCreate(LCDTask, "LCDTask", 10000, NULL, 1, NULL);
}

void loop() {
  
}
