#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include <MPU6050_light.h>

// Define I2S connections
#define I2S_DOUT  14
#define I2S_BCLK  26
#define I2S_LRC   25

// Define SD card connections
#define SD_CS    15
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK  18

//Multitask Core
const byte Core0 = 0;
const byte Core1 = 1;

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

//I2S
Audio audio;

//Ultrasonic
int Pingpin = 33;
int Inpin = 32;
long duration , cm;

//MPU6050 Gyroscope
MPU6050 mpu(Wire);
long X = 0;
long Y = 0;
long Z = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
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

  //Input Ultrasonic
  pinMode(Pingpin,OUTPUT);
  pinMode(Inpin,INPUT);
  
  //SD card and I2S
  pinMode(SD_CS,OUTPUT);
  digitalWrite(SD_CS,HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SD.begin(SD_CS);
  Serial.println("Start SD");

  //I2S Setup
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(10);
  audio.connecttoFS(SD,"/MUSIC2.mp3");

  //MPU6050 Setup
  mpu.begin();
  Serial.println(F("Caculating offsets, do not move MPU6050"));

  mpu.calcOffsets();
  Serial.println("Done!\n");
  
  //Interrupt function Encoder
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoderB,RISING);

  //FreeRtos
  //xTaskCreatePinnedToCore(Sound,"Sound Task",4096*2,NULL,1,NULL,Core0);
  //xTaskCreatePinnedToCore(AngleData,"Angle Task",4096*2,NULL,1,NULL,Core1);
  
}

void loop() {
  PrintData();
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

void PlayForward(void * pvParameters){
  Forward();

  delay(500);

  Stop();
}

void Sound(void * pvParameters){
  for(;;){
    audio.loop();
  }
}

long microTime(int microseconds){
  return microseconds / 29/ 2;
}

void PrintData(){
  //Encoder value
  pos = posi;
  dos = dosi;
  
  //MPU 6050 data
  mpu.update();
  X = mpu.getAngleX();
  Y = mpu.getAngleY();
  Z = mpu.getAngleZ();
  
  //Ultrasonic duration
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

  Serial.print("X :");
  Serial.println(X);
  Serial.print("Y :");
  Serial.println(Y);
  Serial.print("Z :");
  Serial.println(Z);
  
  Serial.print("Ultrasonic: ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  Serial.println(" ");
  Serial.println("------------------------------------------------------------");

  delay(100);

}
