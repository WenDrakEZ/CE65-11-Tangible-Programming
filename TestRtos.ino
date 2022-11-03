#include <Arduino_FreeRTOS.h>

#define RED     6
#define YELLOW  7
#define BLUE    8

void setup() {
  xTaskCreate(redLedControllerTask, "RED LED Task", 128, NULL, 1, NULL);
  xTaskCreate(blueLedControllerTask, "BLUE LED Task", 128, NULL, 1, NULL);
  xTaskCreate(yellowLedControllerTask, "YELLOW LED Task", 128, NULL, 1, NULL);
}

void redLedControllerTask(void *pvParameters)
{
  pinMode(RED, OUTPUT);

  while (1)
  {
    delay(500);
    digitalWrite(RED, digitalRead(RED) ^ 1);

  }
}

void  blueLedControllerTask(void *pvParameters)
{
  pinMode(BLUE, OUTPUT);

  while (1)
  {
    delay(1000);
    digitalWrite(BLUE, digitalRead(BLUE) ^ 1);

  }
}

void yellowLedControllerTask(void *pvParameters)
{
  pinMode(YELLOW, OUTPUT);

  while (1)
  {
    delay(2000);
    digitalWrite(YELLOW, digitalRead(YELLOW) ^ 1);

  }
}
void loop() {}
