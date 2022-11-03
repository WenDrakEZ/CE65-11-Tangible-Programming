// Include required libraries
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
#define SD_CS    5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK  18

#define Button 2
int Buttonstate = 0;

Audio audio;
void setup() {
  pinMode(Button,INPUT);
  pinMode(SD_CS,OUTPUT);
  digitalWrite(SD_CS,HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.begin(115200);
  if(!SD.begin(SD_CS)){
    Serial.println("Error talking to SD card!");
    while(true);
  }
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10);
    audio.connecttoFS(SD,"/MUSIC2.mp3");
}

void loop() {
  // put your main code here, to run repeatedly:
  Buttonstate = digitalRead(Button);
  if (Button == HIGH){
    audio.loop();
  }else{    
 }
}
