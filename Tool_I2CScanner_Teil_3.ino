// Tobias Kuch 2020 GPL 3.0  tobias.kuch@googlemail.com
// https://github.com/kuchto

#include <Wire.h>
#include <SoftWire.h>
//#include <AsyncDelay.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#define oLed_I2C_ADDRESS 0x3C

#define VSensor0 A0
#define VSensor1 A1 

#define UniIOA 4
#define UniIOB 5

#define POTAConfig 6
#define POTBConfig 7

#define UpSwitch 8                // Pin Up Taster
#define DownSwitch 9              // Pin Down Taster
#define ModeSwitch 2             // Pin Modus Taster

#define Prescaler 256            // Mögliche Werte: 1,8,64,256,1024
#define CrystalFreq 16000000     // Quarzgeschwindigkeit des Arduinos: Standard: 16 mhz
#define UReference 4.5             // Spannungsreferenz für den Analogwandler (hier: 4.5 Volt)
#define ADResolution 1023
#define interval1 500            // Zeitraum in ms zwischen 2 analog Messungen der Spannung an A0 und A1

const uint8_t firstAddr = 1;
const uint8_t lastAddr = 0x7F;

SSD1306AsciiWire oled;
SoftWire sw(UniIOA, UniIOB);
SoftWire sw_rev(UniIOB, UniIOA);

//------------------------------------------------------------------------------

uint8_t I2C_Device_found_at_Address = 0;
uint16_t Round = 0;
uint16_t IntFreqStepOld = 0;

bool AlreadyScan = false;
bool Alreadypressed1 = false;
bool Alreadypressed2 = false;
bool Alreadypressed3 = false;
bool Modeswitched = false;

byte SelectedMode = 0;

unsigned int IntFreqStep = 1;

unsigned long previousMillis1 = 0;
unsigned long Frequency = 0;

float VoltageMultiplicator = 0;

ISR(TIMER1_COMPA_vect) { //Timer1 interrupt.
  // Weitere Infos auf: https://www.mikrocontroller.net/articles/AVR-Tutorial:_Timer
  digitalWrite(UniIOA, !(digitalRead(UniIOA)));
  digitalWrite(UniIOB, !(digitalRead(UniIOB)));
}

void setup()
{
  pinMode(ModeSwitch, INPUT_PULLUP);
  pinMode(UpSwitch, INPUT_PULLUP);
  pinMode(DownSwitch, INPUT_PULLUP);
  pinMode(UniIOA, OUTPUT);
  pinMode(UniIOB, OUTPUT);
  pinMode(POTAConfig, OUTPUT);
  pinMode(POTBConfig, OUTPUT);
  digitalWrite (POTAConfig, HIGH);
  digitalWrite (POTBConfig, HIGH);
  cli();                                   // stoppe alle Interrupts
  TCCR1A = 0;                              // set entire TCCR1A register to 0 TCCR - Timer/Counter Control Register
  TCCR1B = 0;                              // Setze Timer/Counter Control Register TCCR1B auf 0
  TCCR1B |= (1 << WGM12);                  // Schalte Clear Timer on Compare (CTC) Modus ein
  if (Prescaler == 0)
  {
    TCCR1B |= (1 << CS10);                  // Setze Prescaler auf 0.
  }
  if (Prescaler == 8)
  {
    TCCR1B |= (1 << CS11);                  // Setze CS11 Bit auf 1 für den 8 Prescaler.
  }
  if (Prescaler == 64)
  {
    TCCR1B |= (1 << CS11) | (1 << CS10);    // Setze CS10 und CS11 Bit auf 1 für den 64 Prescaler.
  }
  if (Prescaler == 256)
  {
    TCCR1B |= (1 << CS12);                  // Setze CS12 Bit auf 1 für den 256 Prescaler.
  }
  if (Prescaler == 1024)
  {
    TCCR1B |= (1 << CS12) | (1 << CS10);    // Setze CS10 und CS12 Bit auf 1 für den 1024 Prescaler.
  }
  TCNT1  = 0;                              // Initialisiere Zähler/Zeitgeber Register Wert auf 0
  OCR1A = 130;                             // Aufruffrequenz Timer 1  241 Hz * 2
  TIMSK1 &= (0 << OCIE1A);                 // Sperre Timer Compare interrupt TIMSK - Timer/Counter Interrupt Mask Register
  sei();                                   // erlaube interrupts
  VoltageMultiplicator = float(UReference)/float(ADResolution);
  Wire.begin();
  Wire.setClock(400000L);
  sw.setTimeout_ms(200);                   // Software I2C Bus Objekt 1 TimeOut
  sw_rev.setTimeout_ms(200);               // Software I2C Bus Objekt 2 TimeOut
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
  bool detected = false;
  for (uint8_t addr = firstAddr; addr <= lastAddr; addr++)
  {
    uint8_t startResult = sw.llStart((addr << 1) + 1); // Signal a read
    sw.stop();
    if ((startResult == 0) & (I2C_Device_found_at_Address != addr)) // New I2C Device found
    {
      Round = 0;
      detected = true;
      I2C_Device_found_at_Address = addr;
      AlreadyScan = false;
      oled.set1X();
      oled.setCursor(0, 0);
      oled.print("I2C Device found at:  ");
      oled.setCursor(0, 1);
      oled.set2X();
      oled.setCursor(0, 1);
      oled.print("-");
      oled.print(addr, BIN);
      oled.print("-b    ");
      oled.set1X();
      oled.setCursor(0, 3);
      oled.print("0x");
      if (addr < 16) oled.print("0");
      oled.print(addr, HEX);
      oled.print(" HEX  --  ");
      if (addr < 10) oled.print("0");
      oled.print(addr, DEC);
      oled.print(" DEC ");
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

bool scanI2C_rev()
{
  bool detected = false;
  for (uint8_t addr = firstAddr; addr <= lastAddr; addr++)
  {
    uint8_t startResult = sw_rev.llStart((addr << 1) + 1); // Signal a read
    sw_rev.stop();
    if ((startResult == 0) & (I2C_Device_found_at_Address != addr)) // New I2C Device found
    {
      Round = 0;
      detected = true;
      I2C_Device_found_at_Address = addr;
      AlreadyScan = false;
      oled.set1X();
      oled.setCursor(0, 0);
      oled.print("I2C Device found at:  ");
      oled.setCursor(0, 1);
      oled.set2X();
      oled.setCursor(0, 1);
      oled.print("-");
      oled.print(addr, BIN);
      oled.print("-b    ");
      oled.set1X();
      oled.setCursor(0, 3);
      oled.print("0x");
      if (addr < 16) oled.print("0");
      oled.print(addr, HEX);
      oled.print(" HEX  --  ");
      if (addr < 10) oled.print("0");
      oled.print(addr, DEC);
      oled.print(" DEC ");
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

void CheckMode()
{
  bool PinStatus1 = digitalRead(UpSwitch);
  if ((PinStatus1 == LOW) && !(Alreadypressed1))
  {
    delay(200);
    IntFreqStep = IntFreqStep + 1;
    if  (IntFreqStep > 254) 
      {
      IntFreqStep = 254;
      }
    Alreadypressed1 = true;
  } else if ((PinStatus1 == HIGH) && (Alreadypressed1))
  {
    Alreadypressed1 = false;
  }

  bool PinStatus2 = digitalRead(DownSwitch);
  if ((PinStatus2 == LOW) && !(Alreadypressed2))
  {
    delay(200);
    IntFreqStep = IntFreqStep - 1;
    if  (IntFreqStep < 1) 
      {
      IntFreqStep = 1;
      }
    Alreadypressed2 = true;
  } else if ((PinStatus2 == HIGH) && (Alreadypressed2))
  {
    Alreadypressed2 = false;
  }

  bool PinStatus3 = digitalRead(ModeSwitch);
  if ((PinStatus3 == LOW) && !(Alreadypressed3))
  {
    delay(200);
    SelectedMode++;
    if  (SelectedMode > 2) {
      SelectedMode = 0;
    }
    Alreadypressed3 = true;
    Modeswitched = true;
  } else if ((PinStatus3 == HIGH) && (Alreadypressed3))
  {
    Alreadypressed3 = false;
  }
}

void loop()
{
  CheckMode();
  if (SelectedMode == 0) // I2C Scanner Modus
  {
    if (Modeswitched)
    {
      cli();  //disable interrupts
      TIMSK1 &= (0 << OCIE1A); // Sperre Timer Compare interrupt TIMSK - Timer/Counter Interrupt Mask Register
      sei();  //allow interrupts
      pinMode(POTAConfig, OUTPUT);
      pinMode(POTBConfig, OUTPUT);
      digitalWrite (POTAConfig, HIGH);
      digitalWrite (POTBConfig, HIGH);
      oled.clear();
      oled.setCursor(0, 0);
      oled.print("Scanning I2C Bus...");
    }
    Modeswitched = false;
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
  }
  CheckMode();
  if (SelectedMode == 0) // I2C Scanner Modus
  {
    if (Modeswitched)
    {
      cli();  //disable interrupts
      TIMSK1 &= (0 << OCIE1A); // Sperre Timer Compare interrupt TIMSK - Timer/Counter Interrupt Mask Register
      TCNT1  = 0;              // Lösche Counter Value
      sei();  //allow interrupts
      pinMode(POTAConfig, OUTPUT);
      pinMode(POTBConfig, OUTPUT);
      digitalWrite (POTAConfig, HIGH);
      digitalWrite (POTBConfig, HIGH);
      oled.clear();
      oled.setCursor(0, 0);
      oled.print("Scanning I2C Bus...");
    }
    Modeswitched = false;
    if ((!scanI2C_rev()) && (Round > 2))
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
  }
  CheckMode();
  if (SelectedMode == 1) // Square Wave Generator
  {
    if (Modeswitched)
    {
     
      digitalWrite(POTAConfig, LOW);
      digitalWrite(POTBConfig, LOW);
      pinMode(POTAConfig, INPUT);
      pinMode(POTBConfig, INPUT);
      pinMode(UniIOA, OUTPUT);
      pinMode(UniIOB, OUTPUT);
      digitalWrite(UniIOA, LOW);
      digitalWrite(UniIOB, HIGH);
      cli();  //disable interrupts
      TCNT1  = 0;              // Lösche Counter Value
      TIMSK1 |= (1 << OCIE1A); // Erlaube Timer compare interrupt TIMSK - Timer/Counter Interrupt Mask Register
      sei();  //allow interrupts
      oled.clear();
      oled.setCursor(0, 0);
      oled.print("Square Wave Generator");
      oled.set2X();
      oled.setCursor(0, 1);
      float Result = float(Frequency) / 1000;
      if (Result > 1)
      {
      oled.print(Result,3);
      oled.print(" kHz ");  
      } else
      {
      oled.print(float(Frequency),2);
      oled.print(" Hz   ");
      }
      oled.set1X();
      oled.setCursor(0, 3);
      oled.print("Step: ");
      oled.print(IntFreqStep);
      oled.print(" Prsc: ");  
      oled.println(Prescaler);    
    }
    Modeswitched = false;

    if (IntFreqStep != IntFreqStepOld)
    {
      IntFreqStepOld = IntFreqStep;
      cli();//stop interrupts
      OCR1A = IntFreqStep;
      if ( TCNT1 >= OCR1A )
      {
        TCNT1 = OCR1A - 1; //initialize counter value to 0
      }
      sei();//allow interrupts
      Frequency = (CrystalFreq / Prescaler) / (IntFreqStep + 1) / 2;
      oled.set2X();
      oled.setCursor(0, 1);
      double Result = float(Frequency) / 1000;
      Serial.println(Result);
      if (Result > 1)
      {
      oled.print(Result,3);
      oled.print(" kHz ");  
      } else
      {
      oled.print(Frequency);
      oled.print(" Hz   ");
      }
      oled.set1X();
      oled.setCursor(0, 3);
      oled.print("Step: ");
      oled.print(IntFreqStep);
      oled.print(" Prsc: ");  
      oled.println(Prescaler);
     // oled.print(" "); 
    }
  }
  CheckMode();
  if (SelectedMode == 2) // 2 Channel Voltmeter 0- 5 Volt 2 stellen genau. Vref ist mit 5 Volt angegeben.
  {
    if (Modeswitched)
    {
      cli();  //disable interrupts
      TIMSK1 &= (0 << OCIE1A); // Verbiete Timer compare interrupt TIMSK - Timer/Counter Interrupt Mask Register
      TCNT1  = 0;              // Lösche Counter Value
      sei();  //allow interrupts
      digitalWrite(POTAConfig, LOW);
      digitalWrite(POTBConfig, LOW);
      pinMode(POTAConfig, OUTPUT);
      pinMode(POTBConfig, OUTPUT);
      digitalWrite(UniIOB, LOW);
      digitalWrite(UniIOA, LOW);
      pinMode(UniIOB, INPUT);
      pinMode(UniIOA, INPUT);   
      oled.clear();
      oled.setCursor(0, 0);
      oled.print("2 Ch. Voltage Sensor");
      oled.setCursor(0, 1);
      oled.print("Channel 1  Channel 2 ");  
    }
    Modeswitched = false;
    if (millis() - previousMillis1 > interval1)
      {
      previousMillis1 = millis();   // aktuelle Zeit abspeichern 
      oled.set1X();
      int VS0 = analogRead(VSensor0);  // read the input pin
      int VS1 = analogRead(VSensor1);  // read the input pin   
      bool VSB0 = digitalRead(VSensor0);  // read the input pin
      bool VSB1 = digitalRead(VSensor1);  // read the input pin    
      
      float Voltage1 = float(VS0) * VoltageMultiplicator;
      float Voltage2 = float(VS1) * VoltageMultiplicator;
      oled.setCursor(0, 1);
      oled.print("Ch1        Ch2");
      oled.setCursor(0, 10);
      oled.print(VSB0);
      oled.setCursor(0, 80);
      oled.print(VSB1);
      oled.set2X();  
      oled.setCursor(0, 2);
      oled.print(Voltage1,2);
      oled.print("V");
      oled.setCursor(66, 2);
      oled.print(Voltage2,2);
      oled.print("V");
      oled.set1X();
      }
  }

  
  // END MainLoop
}
