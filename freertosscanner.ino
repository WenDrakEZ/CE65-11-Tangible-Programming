
#include <SoftwareSerial.h>
#define RX 16
#define TX 17
HardwareSerial mySoftwareSerial(1);
bool stringComplete =false; //ใช้เชคว่ามีการรับข้อมูลครบแล้วหรือไม่
String inputString = "";      // ตัวแปรสำหรับเก็บค่าจากตัวแสกนเนอร์
QueueHandle_t ScannerQueue;

void setup()
{
 Serial.begin(9600);
 
mySoftwareSerial.begin(9600, SERIAL_8N1, RX, TX);
 
 ScannerQueue =  xQueueCreate(20,sizeof(int32_t));

 xTaskCreate(vSenderTask,"Sender Task",100,NULL,1,NULL);
 xTaskCreate(vReceiverTask,"Receiver Task", 100,NULL, 1, NULL);
  
}

void vSenderTask(void *pvParameters)
{
  BaseType_t qStatus;
  String valueToSend = ""; // ตัวแปรสำหรัยเก๋บข้อมูลลงไปใน Queue
  
  while(mySoftwareSerial.available())
  {
    Serial.println("Serial is available");
    char inChar = (char)mySoftwareSerial.read();
    inputString += inChar;
    Serial.println((char)mySoftwareSerial.read());
    Serial.println(inputString);
    
    if (inChar == '\n') {
      stringComplete = true;
      Serial.println("String Complete");
      valueToSend = String(inputString);
    }
    
    qStatus = xQueueSend(ScannerQueue,&valueToSend,0); 
    vTaskDelay(10);
  }
}

void vReceiverTask(void *pvParameters)
{
  String valueReceived;
  BaseType_t qStatus;
  const TickType_t xTicksToWait = pdMS_TO_TICKS(100);
  while(1)
  {
    xQueueReceive(ScannerQueue,&valueReceived,xTicksToWait);
    Serial.print("Received value  : ");
    Serial.println(valueReceived);
    vTaskDelay(10);
  }
}
void loop(){}
