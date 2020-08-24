// Tobias Kuch 2020 GPL 3.0  tobias.kuch@googlemail.com
// https://github.com/kuchto 

/* 
Dieser kurze Sketch durchsucht den I2C-Bus nach Geräten. Wenn ein Gerät gefunden wird, wird es auf einem 128x64 OLed Display
mit seiner Adresse in den Formaten Binär, Hexadezimal und Dezimal angezeigt.
*/

#include <Wire.h>
#include <SoftWire.h>
#include <AsyncDelay.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define oLed_I2C_ADDRESS 0x3C
#define SoftSDA 4
#define SoftSCL 5

const uint8_t firstAddr = 1;
const uint8_t lastAddr = 0x7F;

SSD1306AsciiWire oled;
SoftWire sw(SoftSDA, SoftSCL);

//------------------------------------------------------------------------------

uint8_t I2C_Device_found_at_Address= 0;
uint16_t Round = 0;
bool AlreadyScan = false;

void setup() 
{
  Wire.begin();
  Wire.setClock(400000L);
  sw.setTimeout_ms(200); // Set how long we are willing to wait for a device to respond 
  oled.begin(&Adafruit128x32, oLed_I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.set1X(); 
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("Scanning I2C Bus...");
  oled.setCursor(0, 1);
  oled.print("Initital Scan.");
}
//------------------------------------------------------------------------------

bool scanI2C()
{
bool detected=false;
for (uint8_t addr = firstAddr; addr <= lastAddr; addr++) 
  {
  delayMicroseconds(50);
  uint8_t startResult = sw.llStart((addr << 1) + 1); // Signal a read
  sw.stop();
  if ((startResult==0) & (I2C_Device_found_at_Address != addr)) // New I2C Device found
    {
    Round = 0;  
    detected=true;
    I2C_Device_found_at_Address = addr;
    AlreadyScan = false;
    oled.set1X();
    oled.setCursor(0,0);
    // oled.clear();
    oled.print("I2C Device found at:  ");
    oled.setCursor(0,1);
    oled.set2X();
    oled.setCursor(0,1);
    oled.print("-");
    oled.print(addr,BIN);
    oled.print("-b    ");
    oled.set1X();
    oled.setCursor(0,3); 
    oled.print("0x");
    if(addr<16) oled.print("0");
    oled.print(addr,HEX);
    oled.print(" HEX  --  ");
    if(addr<10) oled.print("0");
    oled.print(addr,DEC);
    oled.print(" DEC ");
    delay(100);
    break;
    }  // Device Found END   
  }  // Scan Round END
  
if (!detected) 
  {
  I2C_Device_found_at_Address = 0;
  Round++; 
  } 
return detected;
} // Function END 

void loop() 
{
 if ((!scanI2C()) && (Round > 2))
  {
   if (!AlreadyScan)
    {
     AlreadyScan = true;
     oled.clear();
     oled.setCursor(0, 0);
     oled.print("Scanning I2C Bus..."); 
    }
   Round = 0;
   I2C_Device_found_at_Address = 0;
  }
 delay(200);  
}
