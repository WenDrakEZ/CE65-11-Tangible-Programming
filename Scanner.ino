String inputString = "";      // ตัวแปรสำหรับเก็บค่าจากตัวแสกนเนอร์
String DataScanner = "";      //ตัวแปรสำหรับเก็บค่าในการประมวลผล
bool stringComplete = false;  // ตัวแปรเช็คว่ารับข้อมูลครบแล้วหรือยัง
 
void setup() {  
  
  //pinMode(16, OUTPUT); //พอร์ตสำหรับกระตุ้นให้ตัวแสกนเนอร์ทำงาน
  // initialize serial:
  Serial.begin(115200);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // ประกาศตัวแปรสำหรับเก็บข้อความขนาด 200 ตัวอักษร
  inputString.reserve(200);
}
 
void loop() {
  //digitalWrite(16, HIGH);   // ส่งค่าลอจิก High
  //delay(200);              // หน่วงเวลา 200 msec
  //digitalWrite(16, LOW);    // ส่งค่าลอจิก Low เปรียบเสมือนการกดสวิตช์ เพื่อให้ตัวแสกนเนอร์ทำงาน
  //delay(200);              // หน่วงเวลา 200 msec 
  
  // เริ่มเช็คว่ามีข้อมูลจากตัวแสกนเนอร์เข้ามาครบหรือยัง
  if (stringComplete) {
    
       if (DataScanner == "test\r\n") // ข้อมูลตรงกับที่ตั้งไว้ ให้แสดงข้อความ OK
       {
          Serial.println("OK");
          delay(1000); 
       }   
       else Serial.println("No Data"); // ข้อมูลไม่ตรงกับที่ตั้งไว้ ให้แสดงข้อความ No Data
        
    // เคลียร์ค่า เพื่อรอข้อมูลใหม่
    inputString = "";
    DataScanner = "";
    stringComplete = false;
  }
}
 
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
 
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      DataScanner = String(inputString);
    }
  }
}
