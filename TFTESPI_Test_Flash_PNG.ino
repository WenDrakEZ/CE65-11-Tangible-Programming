

#include <PNGdec.h>
#include "panda.h" 

PNG png; // PNG decoder inatance

#define MAX_IMAGE_WDITH 240 

int16_t xpos = 0;
int16_t ypos = 0;


#include "SPI.h"
#include <TFT_eSPI.h>             
TFT_eSPI tft = TFT_eSPI();         

void setup()
{
  Serial.begin(115200);
  Serial.println("\n\n Using the PNGdec library");

  // Initialise the TFT
  tft.begin();
  tft.fillScreen(TFT_BLACK);

  Serial.println("\r\nInitialisation done.");
}


void loop()
{
  int16_t rc = png.openFLASH((uint8_t *)panda, sizeof(panda), pngDraw);
  if (rc == PNG_SUCCESS) {
    Serial.println("Successfully png file");
    Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    Serial.print(millis() - dt); Serial.println("ms");
    tft.endWrite();
    // png.close(); // not needed for memory->memory decode
  }
  delay(3000);
  tft.fillScreen(random(0x10000));
}



}
