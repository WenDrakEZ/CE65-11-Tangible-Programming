#include "Arduino.h"
#include "WiFi.h"
#include "Audio.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// Define I2S connections
#define I2S_DOUT  22
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
int Value = 150;

//I2S
Audio audio;

//Ultrasonic
int Pingpin = 33;
int Inpin = 32;
long duration , cm;

void setup() {
  Serial.begin(115200);
   
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
  Serial.begin(115200);
  if(!SD.begin(SD_CS)){
    Serial.println("Error talking to SD card!");
    while(true);
  }
  Serial.println("Start SD");

  //I2S Setup
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(10);
  audio.connecttoFS(SD,"/MUSIC2.mp3");

  //Interrupt function Encoder
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoderB,RISING);

  //FreeRtos
  xTaskCreatePinnedToCore(Sound,"Sound Task",4096*2,NULL,1,NULL,Core0);
  
}

void loop() {
  //Encoder value
  pos = posi;
  dos = dosi;
  
  Serial.print("ENA :");
  Serial.println(pos);
  Serial.print("ENB :");
  Serial.println(dos);

  delay(100);
  
  //Ultrasonic duration
  digitalWrite(Pingpin,LOW);
  delayMicroseconds(2);
  digitalWrite(Pingpin,HIGH);
  delayMicroseconds(5);
  digitalWrite(Pingpin, LOW);
  duration = pulseIn(Inpin, HIGH);

  cm = microTime(duration);
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(100);

  
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

void Forward(void){
  analogWrite(IN1,Value);
  analogWrite(IN2,LOW);
  
  analogWrite(IN3,Value);
  analogWrite(IN4,LOW);
}

void Reverse(void){
  analogWrite(IN1,LOW);
  analogWrite(IN2,Value);
  
  analogWrite(IN3,LOW);
  analogWrite(IN4,Value);  
}

void Stop(void){
  analogWrite(IN1,LOW);
  analogWrite(IN2,LOW);
  
  analogWrite(IN3,LOW);
  analogWrite(IN4,LOW);
}

void TurnLeft(void){
  analogWrite(IN1,Value);
  analogWrite(IN2,LOW);
  
  analogWrite(IN3,LOW);
  analogWrite(IN4,Value);
}

void TurnRight(void){ 
  analogWrite(IN1,LOW); 
  analogWrite(IN2,Value);
  
  analogWrite(IN3,Value);
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
