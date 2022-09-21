 /*
 This is an example on how to use the 1.8" TFT 128x160 SPI ST7735 display using the Ucglib library.
 
 ST7735 TFT SPI display pins for Arduino Uno/Nano:
 LED = 3.3V
 SCK = 13
 SDA = 11
 A0 = 8
 RESET = 9
 CS = 10
 GND = GND
 VCC = 5V
 
 Another version marked as KMR-1.8 SPI:
 This version only supports 3.3V logic so put a level shifter for all I/O pins, or a 2.2k resistor between
 the display and arduino, and a 3.3k resistor to ground to create a simple voltage divider to produce a 3V output.
 LED- = GND
 LED+ = 15Ω resistor to 5V
 CS = 10
 SCL = 13
 SDA = 11
 A0 = 8
 RESET = 9
 VCC = 5V or 3.3V (the display has it's own 3.3V regulator)
 GND = GND
 
 Hardware SPI Pins:
 Arduino Uno SCK=13, SDA=11
 Arduino Nano SCK=13, SDA=11
 Arduino Due SCK=76, SDA=75
 Arduino Mega SCK=52, SDA=51
 
 SPI pin names can be confusing. These are the alternative names for the SPI pins:
 MOSI = DIN = R/W = SDO = DI = SI = MTSR = SDA = D1 = SDI
 CS = CE = RS = SS
 DC = A0 = DO = DOUT = SO = MRST
 RESET = RST
 SCLK = CLK = E = SCK = SCL = D0
 
 
 Ucglib library: https://github.com/olikraus/ucglib
 
 
 Reference Page: https://github.com/olikraus/ucglib/wiki/reference
 
 
 List of fonts: https://github.com/olikraus/ucglib/wiki/fontsize
 https://github.com/olikraus/ucglib/wiki/fontgroup
 
 Font Purpose Description:
 t = Transparent font, use together with UCG_FONT_MODE_TRANSPARENT. t is identical to h but font size is smaller.
 h = All glyphs have common height, use together with UCG_FONT_MODE_TRANSPARENT and UCG_FONT_MODE_SOLID.
 m = All glyphs have common height and width (monospace), use together with UCG_FONT_MODE_TRANSPARENT and UCG_FONT_MODE_SOLID.
 8 = All glyphs have common height and width, use together with UCG_FONT_MODE_TRANSPARENT and UCG_FONT_MODE_SOLID. Height and width are devisible by 8.
 
 Font Char Set Description:
 f = The font includes all glyphs of the original font (up to 256).
 r = Only glyphs on the range of the ASCII codes 32 to 127 are included in the font.
 n = Only numbers and extra glyphs for writing date and time strings are included in the font.
 
 Recommended fonts end with _mr or _mf
 _mr lack some special characters but uses less memory
 _mf supports more special characters but uses more memory
 
 
 Sketch made by: InterlinkKnight
 Last modification: 01/11/2018
*/
 
 
#include "Ucglib.h" // Include Ucglib library to drive the display
 
 
// Create display and define the pins:
Ucglib_ST7735_18x128x160_HWSPI ucg(8, 10, 9); // (A0=8, CS=10, RESET=9)
// The rest of the pins are pre-selected as the default hardware SPI for Arduino Uno (SCK=13 and SDA=11)
 
 
 
 
 
 
int Variable1; // Create a variable to have something dynamic to show on the display
 
 
 
 
 
 
void setup(void) // Start of setup
{
 
// Display setup:
 
// Select a type of text background:
//ucg.begin(UCG_FONT_MODE_TRANSPARENT); // It doesn't overwrite background, so it's a mess for text that changes
ucg.begin(UCG_FONT_MODE_SOLID); // It writes a background for the text. This is the recommended option
 
//ucg.clearScreen(); // Clear the screen
 
// Set display orientation:
ucg.setRotate180(); // Put 90, 180 or 270, or comment to leave default
 
// Set text orientation:
// It only alters the text, not the entire display
//ucg.setPrintDir(0); // 0, 1, 2 or 3 to change orientation of text. If not specify, orientation is 0
 
 
 
 
 
//************* My favorite fonts:
// Font name: fontgroupx11:
//ucg.setFont(ucg_font_4x6_mr);
//ucg.setFont(ucg_font_5x7_mr);
//ucg.setFont(ucg_font_5x8_mr);
//ucg.setFont(ucg_font_6x10_mr);
//ucg.setFont(ucg_font_6x12_mr);
//ucg.setFont(ucg_font_6x13_mr);
//ucg.setFont(ucg_font_7x13_mr);
//ucg.setFont(ucg_font_7x14_mr);
//ucg.setFont(ucg_font_7x14B_mr);
//ucg.setFont(ucg_font_8x13_mr);
//ucg.setFont(ucg_font_8x13B_mr);
//ucg.setFont(ucg_font_8x13O_mr);
//ucg.setFont(ucg_font_9x15_mr);
//ucg.setFont(ucg_font_9x15B_mr);
//ucg.setFont(ucg_font_9x18_mr);
//ucg.setFont(ucg_font_9x18B_mr);
//ucg.setFont(ucg_font_10x20_mr); // This is the biggest size for this font
 
// Font name: fontgroupadobex11:
//ucg.setFont(ucg_font_courR08_mr);
//ucg.setFont(ucg_font_courB08_mr);
//ucg.setFont(ucg_font_courR10_mr);
//ucg.setFont(ucg_font_courR12_mr);
//ucg.setFont(ucg_font_courB12_mr);
//ucg.setFont(ucg_font_courR14_mr);
//ucg.setFont(ucg_font_courB14_mr);
//ucg.setFont(ucg_font_courR18_mr);
//ucg.setFont(ucg_font_courB18_mr);
//ucg.setFont(ucg_font_courR24_mr);
//ucg.setFont(ucg_font_courB24_mr);
//ucg.setFont(ucg_font_courR24_mr);
//ucg.setFont(ucg_font_courB24_mr); // This is the biggest size for this font
 
// Font name: fontgroupprofont
//ucg.setFont(ucg_font_profont10_mr);
//ucg.setFont(ucg_font_profont11_mr);
//ucg.setFont(ucg_font_profont12_mr);
//ucg.setFont(ucg_font_profont15_mr);
//ucg.setFont(ucg_font_profont17_mr);
//ucg.setFont(ucg_font_profont22_mr);
//ucg.setFont(ucg_font_profont29_mr);
 
// Font name: fontgroupinconsolata:
//ucg.setFont(ucg_font_inr16_mr); // This is the smallest size for this font
//ucg.setFont(ucg_font_inb16_mr);
//ucg.setFont(ucg_font_inr19_mr);
//ucg.setFont(ucg_font_inb19_mr);
//ucg.setFont(ucg_font_inr21_mr);
//ucg.setFont(ucg_font_inb21_mr);
//ucg.setFont(ucg_font_inr24_mr);
//ucg.setFont(ucg_font_inb24_mr);
//ucg.setFont(ucg_font_inr27_mr);
//ucg.setFont(ucg_font_inb27_mr);
//ucg.setFont(ucg_font_inr30_mr);
//ucg.setFont(ucg_font_inb30_mr);
//ucg.setFont(ucg_font_inr33_mr);
//ucg.setFont(ucg_font_inb33_mr);
//ucg.setFont(ucg_font_inr38_mr);
//ucg.setFont(ucg_font_inb38_mr);
//ucg.setFont(ucg_font_inr42_mr);
//ucg.setFont(ucg_font_inb42_mr);
//ucg.setFont(ucg_font_inr46_mr);
//ucg.setFont(ucg_font_inb46_mr);
//ucg.setFont(ucg_font_inr49_mr);
//ucg.setFont(ucg_font_inb49_mr);
//ucg.setFont(ucg_font_inr53_mr);
//ucg.setFont(ucg_font_inb53_mr);
//ucg.setFont(ucg_font_inr57_mr);
//ucg.setFont(ucg_font_inb57_mr);
//*************************************
 
 
 
 
// We are going to print on the display everything that is static on the setup, to leave the loop free for dynamic elements:
ucg.setFont(ucg_font_profont15_mr);
ucg.setColor(0, 255, 255, 255); // Set color (0,R,G,B)
ucg.setColor(1, 0, 0, 0); // Set color of text background (1,R,G,B)
ucg.setPrintPos(0, 24); // Set position (x,y)
ucg.print("myarduino.net"); // Print text or value
ucg.setFont(ucg_font_inr16_mr);
// Write to the display the text "myarduino.net":
ucg.setColor(0, 255, 0, 0); // Set color (0,R,G,B)
ucg.setPrintPos(0, 50); // Set position (x,y)
ucg.print("myarduino"); // Print text or value
 
 
// Draw rectangle:
ucg.setColor(0, 255, 255); // Set color (0,R,G,B)
ucg.drawFrame(0, 58, 60, 30); // Start from top-left pixel (x,y,wigth,height)
 
 
// Draw rounded rectangle:
ucg.setColor(0, 255, 255); // Set color (0,R,G,B)
ucg.drawRFrame(68, 58, 59, 30, 8); // Start from top-left pixel (x,y,wigth,height,radius)
 
 
// Draw filled circle:
ucg.setColor(255, 0, 0); // Set color (0,R,G,B)
ucg.drawDisc(22, 106, 16, UCG_DRAW_ALL); // (x,y,radius,option)
 
 
// Draw filled rectangle:
ucg.setColor(128, 0, 128); // Set color (0,R,G,B)
ucg.drawBox(50, 92, 70, 31); // Start from top-left pixel (x,y,wigth,height)
 
 
// Draw horizontal line:
ucg.setColor(255, 255, 255); // Set color (0,R,G,B)
ucg.drawHLine(0, 126, 128); // Start from left pixel (x,y,lenght)
 
 
// Draw filled rounded rectangle:
ucg.setColor(0, 255, 0); // Set color (0,R,G,B)
ucg.drawRBox(2, 132, 45, 25, 8); // Start from top-left pixel (x,y,wigth,height,radius)
 
 
// Draw circle:
ucg.setColor(255, 255, 0); // Set color (0,R,G,B)
ucg.drawCircle(110, 144, 14, UCG_DRAW_ALL); // (x,y,radius,option)
 
 
// Draw filled triangle:
ucg.setColor(0, 0, 255); // Set color (0,R,G,B)
ucg.drawTriangle(50, 159, 70, 130, 90, 159); // (x0,y0,x1,y1,x2,y2)
 
} // End of setup
 
 
 
 
 
 
 
void loop(void) // Start of loop
{
 
Variable1++; // Increase variable by 1
if (Variable1 > 150) // If Variable1 is greater than 150
{
Variable1 = 0; // Set Variable1 to 0
}
 
 
 
// Convert Variable1 into a string, so we can change the text alignment to the right:
// It can be also used to add or remove decimal numbers.
char string[10]; // Create a character array of 10 characters
// Convert float to a string:
dtostrf(Variable1, 3, 0, string); // (,,,)
 
 
 
 
 
 
 
// We are going to print on the display everything that is dynamic on the loop, to refresh continuously:
 
// Write to the display the Variable1 with left text alignment:
ucg.setFont(ucg_font_inb16_mr); // Set font
ucg.setColor(0, 255, 255, 0); // Set color (0,R,G,B)
ucg.setColor(1, 0, 0, 0); // Set color of text background (1,R,G,B)
ucg.setPrintPos(9, 81); // Set position (x,y)
ucg.print(Variable1); // Print text or value
 
// There is a problem when we go, for example, from 100 to 99 because it doesn't automatically write a background on
// the last digit we are not longer refreshing. We need to check how many digits are and fill the space remaining.
if (Variable1 < 10) // If Variable1 is less than 10...
{
// Fill the other digit with background color:
ucg.print(" ");
}
if (Variable1 < 100) // If Variable1 is less than 100...
{
// Fill the other digit with background color:
ucg.print(" ");
}
 
 
 
 
 
// Write to the display the string with right text alignment:
ucg.setFont(ucg_font_inb16_mr); // Set font
ucg.setColor(0, 0, 255, 0); // Set color (0,R,G,B)
ucg.setColor(1, 0, 0, 0); // Set color of text background (1,R,G,B)
ucg.setPrintPos(76, 81); // Set position (x,y)
ucg.print(string); // Print text or value
 
 
 
 
 
 
} // End of loop
