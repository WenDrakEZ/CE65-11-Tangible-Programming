#include <SoftwareSerial.h>

SoftwareSerial mySerial(16, 17); // RX, TX

String inputString = "";      // ตัวแปรสำหรับเก็บค่าจากตัวแสกนเนอร์
String DataScanner = "";      //ตัวแปรสำหรับเก็บค่าในการประมวลผล
bool stringComplete = false;  // ตัวแปรเช็คว่ารับข้อมูลครบแล้วหรือยัง
 
void setup() {
  // initialize serial:
  mySerial.begin(115200);
  Serial.begin(115200);
   while (!Serial) {
  }
  
  inputString.reserve(200);
}
 
void loop() {
  if(mySerial.available()) {
    
    Serial.println("Test");
    char inChar = (char)mySerial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      
      stringComplete = true;
      DataScanner = String(inputString);
      
    }
  }
  
  if (stringComplete) {
    
       if (DataScanner == "4549526608711\r\n") // ข้อมูลตรงกับที่ตั้งไว้ ให้แสดงข้อความ OK
       {
          Serial.println("OK");
          Serial.println(DataScanner);
          delay(1000); 
       }   
       else Serial.println("No Data"); // ข้อมูลไม่ตรงกับที่ตั้งไว้ ให้แสดงข้อความ No Data
        
    inputString = "";
    DataScanner = "";
    stringComplete = false;
  }
}
 

 
