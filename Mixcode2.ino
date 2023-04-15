#include <lvgl.h>
#include "SPI.h"
#include "TFT_eSPI.h"
#include <Ticker.h>
#include "SD.h"
#include "FS.h"
#include <HardwareSerial.h>
#include "Audio.h"

//// Define I2S connections
//#define I2S_DOUT  1
//#define I2S_BCLK  26
//#define I2S_LRC   25
//
//// Define SD card connections
//#define SD_CS    15
//#define SPI_MOSI 23
//#define SPI_MISO 19
//#define SPI_SCK  18
//
////I2S
//Audio audio;

//Ultrasonic
int Pingpin = 33;
int Inpin = 32;
long duration , cm;

// INPUT MOTOR
#define IN1 0
#define IN2 2
#define IN3 13
#define IN4 12

//INPUT PAUSE
const int ENCA = 36;
const int ENCB = 34;
const int ENCC = 39;
const int ENCD = 35;

int pos = 0;
long posi = 0;

int dos = 0;
long dosi = 0;

//PWM value
int Value = 255;

// define scanner
#define RX_PIN 16
#define TX_PIN 17
#define scannerPin 4
HardwareSerial mySerial(1); 
QueueHandle_t myQueue;
QueueHandle_t ledQueue;

int checkplay = 1; // tab2
boolean checkreset = false; //tab3
int scanactive = 0; //tab1

//boolean musics = false; // test music

TaskHandle_t GUI;

Ticker tick;
#define LVGL_TICK_PERIOD 1

uint16_t calData[5] = { 410, 3443, 269, 3364, 7 };



//display buffer
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

TFT_eSPI tft = TFT_eSPI();

// ใช้งานร่วม Driver TFT_sSPI
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint16_t c;

  tft.startWrite(); /* Start new TFT transaction */
  tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
  for (int y = area->y1; y <= area->y2; y++) {
    for (int x = area->x1; x <= area->x2; x++) {
      c = color_p->full;
      tft.writeColor(c, 1);
      color_p++;
    }
  }
  tft.endWrite(); /* terminate TFT transaction */
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

//ใช้งานร่วม Driver TFT_sSPI
bool my_input_read(lv_indev_drv_t * indev, lv_indev_data_t * data) {
  static uint16_t prev_x, prev_y;
  uint16_t X = 0, Y = 0;
  data -> state = tft.getTouch(&X,&Y) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  if(data -> state == LV_INDEV_STATE_PR) {
    prev_x = X;
    prev_y = Y;
  }
  data -> point.x = prev_x;
  data -> point.y = prev_y;
  return false;
}

void setup() {
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(9600);
  
  lv_init();
  tft.begin();
  tft.setRotation(1);
  tft.setTouch(calData);
  
  //Input Ultrasonic
  pinMode(Pingpin,OUTPUT);
  pinMode(Inpin,INPUT);

  
//  //music
//  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); 
//  
//  //SD card and I2S
//  SD.begin(SD_CS);
//  pinMode(SD_CS,OUTPUT);
//  digitalWrite(SD_CS,HIGH);
//  if(!SD.begin(SD_CS)){
//    Serial.println("Error talking to SD card!");
//    while(true);
//  }
//    Serial.println("Connect");
//
//  //I2S Setup
//  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
//  audio.setVolume(20);

   //INPUT Motor
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  //INPUT Encoder
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
  pinMode(ENCC,INPUT_PULLUP);
  pinMode(ENCD,INPUT_PULLUP);
  //Read interrupt Encoder
  digitalWrite(ENCA,HIGH);
  digitalWrite(ENCB,HIGH);  
  digitalWrite(ENCC,HIGH);  
  digitalWrite(ENCD,HIGH);

   //Interrupt function Encoder
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoderB,RISING);
 
  myQueue = xQueueCreate(20, sizeof(String));
  ledQueue = xQueueCreate(1, sizeof(int));

  xTaskCreatePinnedToCore(readTask, "readTask", 4096*2, NULL, 1, NULL,1);
  xTaskCreatePinnedToCore(LCDTask, "LCDTask", 4096*2, NULL, 1, NULL,1);
  xTaskCreatePinnedToCore(PrintDataTask, "PrintDataTask", 4096*2, NULL, 1, NULL,1);
//  xTaskCreatePinnedToCore(SoundTask,"SoundTask",4096*2,NULL,1,NULL,0);
  
  //display interface
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX *10);
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);
  
  //input device interface
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init (&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_input_read;
  lv_indev_drv_register(&indev_drv);
  
  
  Create_gui();
  
  tick.attach_ms(LVGL_TICK_PERIOD,lv_tick_handler);
}

void loop(void) {
  lv_task_handler();
  delay(5);
//  PrintData();
  //Serial.print("Free memory: ");
  //Serial.println(ESP.getFreeHeap());
}

static void lv_tick_handler(void){
  lv_tick_inc(LVGL_TICK_PERIOD);
}

//button Style
static lv_style_t style_btn;
static lv_anim_path_t path;


lv_obj_t * img;
LV_IMG_DECLARE(start);
LV_IMG_DECLARE(finish);
LV_IMG_DECLARE(sword_small);
LV_IMG_DECLARE(sword_large);
LV_IMG_DECLARE(vs_monster);
LV_IMG_DECLARE(front);
LV_IMG_DECLARE(left);
LV_IMG_DECLARE(right);
LV_IMG_DECLARE(back);
LV_IMG_DECLARE(pick);

void Create_gui(){
  
    lv_obj_t * label;
    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, LV_STATE_DEFAULT, 10);
    lv_style_set_bg_opa(&style_btn, LV_STATE_DEFAULT, LV_OPA_COVER);
    lv_style_set_bg_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_SILVER);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_GRAY);
    lv_style_set_bg_grad_dir(&style_btn, LV_STATE_DEFAULT, LV_GRAD_DIR_VER);

    /*Swap the colors in pressed state*/
    lv_style_set_bg_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_GRAY);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_SILVER);

    /*Add a border*/
    lv_style_set_border_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_style_set_border_opa(&style_btn, LV_STATE_DEFAULT, LV_OPA_70);
    lv_style_set_border_width(&style_btn, LV_STATE_DEFAULT, 2);

    lv_style_init(&style_btn);
    lv_style_set_bg_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_MAROON);
    lv_style_set_bg_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_MAROON);
    lv_style_set_bg_grad_color(&style_btn, LV_STATE_PRESSED, LV_COLOR_RED);
    lv_style_set_text_color(&style_btn, LV_STATE_DEFAULT, LV_COLOR_WHITE);

    
    lv_anim_path_init(&path);
    lv_anim_path_set_cb(&path, lv_anim_path_overshoot);
    lv_style_set_transition_prop_1(&style_btn, LV_STATE_DEFAULT, LV_STYLE_TRANSFORM_HEIGHT);
    lv_style_set_transition_prop_2(&style_btn, LV_STATE_DEFAULT, LV_STYLE_TRANSFORM_WIDTH);
    lv_style_set_transition_time(&style_btn, LV_STATE_DEFAULT, 300);
    lv_style_set_transition_path(&style_btn, LV_STATE_DEFAULT, &path);

    lv_style_set_transform_height(&style_btn, LV_STATE_PRESSED, -5);
    lv_style_set_transform_width(&style_btn, LV_STATE_PRESSED, -10);


    //Button
    lv_obj_t * btn1 = lv_btn_create(lv_scr_act(),NULL);
    lv_obj_set_event_cb(btn1, event_tab1);
    lv_obj_set_size(btn1, 100, 50);
    lv_obj_align(btn1, NULL, LV_ALIGN_IN_BOTTOM_LEFT,5,0);
    lv_obj_add_style(btn1, LV_BTN_PART_MAIN,&style_btn);
    label = lv_label_create(btn1, NULL);
    lv_label_set_text(label,"Read");

    lv_obj_t * btn2 = lv_btn_create(lv_scr_act(),NULL);
    lv_obj_set_event_cb(btn2, event_tab2);
    lv_obj_set_size(btn2, 100, 50);
    lv_obj_align(btn2, NULL, LV_ALIGN_IN_BOTTOM_MID,0,0);
    lv_obj_add_style(btn2, LV_BTN_PART_MAIN,&style_btn);
    label = lv_label_create(btn2, NULL);
    lv_label_set_text(label,"Play");

    lv_obj_t * btn3 = lv_btn_create(lv_scr_act(),NULL);
    lv_obj_set_event_cb(btn3, event_tab3);
    lv_obj_set_size(btn3, 100, 50);
    lv_obj_align(btn3, NULL, LV_ALIGN_IN_BOTTOM_RIGHT,-5,0);
    lv_obj_add_style(btn3, LV_BTN_PART_MAIN,&style_btn);
    label = lv_label_create(btn3, NULL);
    lv_label_set_text(label,"Stop");
}
   

// read data from QR Code Scanner
void readTask(void * pvParameters) {
  pinMode(scannerPin, OUTPUT);
  String data = "";
   for (;;) {
    while (mySerial.available() > 0) {
      char ch = mySerial.read();
      data += ch;
      
    }
    if (data.length() > 0) {
      data.trim();
      Serial.println("Data = " + String(data));
      xQueueSend(myQueue, &data, portMAX_DELAY);

      
      
      if (data == "sword") {
          lv_obj_del(img);
          lv_sword_large();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      } 
      else if (data == "sword2") {
          lv_obj_del(img);
          lv_sword_small();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      } 
      else if (data == "pick") {
          lv_obj_del(img);
          lv_pick();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else if (data == "front") {
          lv_obj_del(img);
          lv_front();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else if (data == "back") {
          lv_obj_del(img);
          lv_back();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else if (data == "left") {
          lv_obj_del(img);
          lv_left();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else if (data == "right") {
          lv_obj_del(img);
          lv_right();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else if (data == "start") {
          lv_start();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      else if (data == "finish") {
          lv_obj_del(img);
          lv_finish();
          Serial.println("Data =" + data);
          checkreset = true;
          vTaskDelay(pdMS_TO_TICKS(1000));
      }


      digitalWrite(scannerPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000));
      digitalWrite(scannerPin, LOW);
   
      
      data = "";
       
    }
    
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void LCDTask(void *parameter) {
  String data = "";
  int databutton = 2;

     for (;;) {
        xQueueReceive(ledQueue, &databutton, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        if(databutton == 1){
          int numMessages = uxQueueMessagesWaiting(myQueue);
          for (int i = 0; i < numMessages; i++) {
            xQueueReceive(myQueue, &data, 0);
            // delete whitespace
            data.trim();
            Serial.println(data);
            if (data == "sword") {
              lv_obj_del(img);
              delay(1000);
              lv_sword_large();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            } 
            else if (data == "sword2") {
              lv_obj_del(img);
              delay(1000);
              lv_sword_small();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            } 
            else if (data == "pick") {
              lv_obj_del(img);
              delay(1000);
              lv_pick();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (data == "front") {
              lv_obj_del(img);
              delay(1000);
              lv_front();
              checkreset = true;
              Serial.println("Data =" + data);
              Forward();
              delay(1000);
              Stop();
              vTaskDelay(pdMS_TO_TICKS(1000));
              
              
             
            }
            else if (data == "back") {
              lv_obj_del(img);
              delay(1000);
              lv_back();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (data == "left") {
              lv_obj_del(img);
              delay(1000);
              lv_left();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (data == "right") {
              lv_obj_del(img);
              delay(1000);
              lv_right();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (data == "start") {
              lv_obj_del(img);
              delay(1000);
              lv_start();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (data == "finish") {
              lv_obj_del(img);
              delay(1000);
              lv_finish();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            else if (data == "VS1") {
              lv_obj_del(img);
              delay(1000);
              lv_vs_monster();
              Serial.println("Data =" + data);
              checkreset = true;
              vTaskDelay(pdMS_TO_TICKS(1000));
            }
            data = "";
            databutton = 2;
          }
         vTaskDelay(pdMS_TO_TICKS(10));
       }
    
      
  }
  
  
}

//void SoundTask(void * pvParameters){
//  for(;;){
//    audio.loop();
//    delay(1);
//  }
//}




//Button event
static void event_tab1(lv_obj_t * obj, lv_event_t tab1){
  
  pinMode(scannerPin, OUTPUT);
  if(tab1 == LV_EVENT_CLICKED){
    
    scanactive += 1;
  
    if(scanactive == 1){
      Serial.println("A");
      Serial.println(scanactive);
      digitalWrite(scannerPin, HIGH);
      delay(1);
      digitalWrite(scannerPin, LOW);
   
    }
    else if (scanactive == 2){
      Serial.println("B");
      Serial.println(scanactive);
      digitalWrite(scannerPin, HIGH);
      scanactive = 0;
    
    }
  }

  else{
  }
}





static void event_tab2(lv_obj_t * obj, lv_event_t tab2){
  if (tab2 == LV_EVENT_CLICKED){
    
    Serial.println("Play");
    xQueueSend(ledQueue, &checkplay, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));
   
  }
}


//static void event_tab3(lv_obj_t * obj, lv_event_t tab3){
//  if (tab3 == LV_EVENT_CLICKED){
//    audio.connecttoFS(SD,"/MUSIC2.mp3");
//  }else{
//  }
//}


static void event_tab3(lv_obj_t * obj, lv_event_t tab3){
  if (tab3 == LV_EVENT_CLICKED){
    if(checkreset == true){
      lv_obj_del(img);
      checkreset = false;
    }else{
      Serial.println("No image");
    }
    
  }else{
  }
}




void readEncoderA(){            //Pause A
  int b = digitalRead(ENCB);  
  if(b > 0){
    posi++;
  }else{
    posi--;
  }
}

void readEncoderB(){          //Pause B
  int c = digitalRead(ENCD); 
  if(c > 0){
    dosi++;
  }else{
    dosi--;
  }
}


void PrintDataTask(void *parameter){

  for (;;) {
         //Encoder value
        pos = posi;
        dos = dosi;
        
        //MPU 6050 data
      //  mpu.update();
      //  X = mpu.getAngleX();
      //  Y = mpu.getAngleY();
      //  Z = mpu.getAngleZ();
        
      //  Ultrasonic duration
        digitalWrite(Pingpin,LOW);
        delayMicroseconds(2);
        digitalWrite(Pingpin,HIGH);
        delayMicroseconds(5);
        digitalWrite(Pingpin, LOW);
        duration = pulseIn(Inpin, HIGH);
      
        cm = microTime(duration);
      
        //print data
        Serial.print("ENA :");
        Serial.println(pos);
        Serial.print("ENB :");
        Serial.println(dos);
      
      //  Serial.print("X :");
      //  Serial.println(X);
      //  Serial.print("Y :");
      //  Serial.println(Y);
      //  Serial.print("Z :");
      //  Serial.println(Z);
        
        Serial.print("Ultrasonic: ");
        Serial.print(cm);
        Serial.print("cm");
        Serial.println();
      
        Serial.println(" ");
        Serial.println("------------------------------------------------------------");
      
        vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
 

}

long microTime(int microseconds){
  return microseconds / 29/ 2;
}

void Forward(){
  analogWrite(IN1,LOW);
  analogWrite(IN2,Value);
  
  analogWrite(IN3,Value);
  analogWrite(IN4,LOW);
}

void Reverse(){
  analogWrite(IN1,Value);
  analogWrite(IN2,LOW);
  
  analogWrite(IN3,LOW);
  analogWrite(IN4,Value);  
}

void TurnRight(){
  analogWrite(IN1,LOW);
  analogWrite(IN2,Value);
  
  analogWrite(IN3,LOW);
  analogWrite(IN4,Value);
}

void TurnLeft(){ 
  analogWrite(IN1,Value); 
  analogWrite(IN2,LOW);
  
  analogWrite(IN3,Value);
  analogWrite(IN4,LOW);
}

void Stop(){
  analogWrite(IN1,LOW);
  analogWrite(IN2,LOW);
  
  analogWrite(IN3,LOW);
  analogWrite(IN4,LOW);
}

void lv_sword_small(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &sword_small);
  lv_obj_set_size(img, 50, 50);
  lv_obj_align(img,NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
}

void lv_sword_large(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &sword_large);
  lv_obj_set_size(img, 120, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, 0);
}

void lv_vs_monster(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &vs_monster);
  lv_obj_set_size(img, 120, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_front(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &front);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_left(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &left);
  lv_obj_set_size(img, 120, 80);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, 0);
}

void lv_right(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &right);
  lv_obj_set_size(img, 120, 80);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, 0);
}

void lv_back(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &back);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_start(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &start);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_finish(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &finish);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}

void lv_pick(){
  img = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(img, &pick);
  lv_obj_set_size(img, 80, 120);
  lv_obj_align(img,NULL, LV_ALIGN_CENTER, 0, -20);
}
