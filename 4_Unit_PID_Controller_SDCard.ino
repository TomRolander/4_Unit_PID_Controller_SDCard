
/**************************************************************************
 4xTomPort PID Controller
 **************************************************************************/

#define VERSION "Ver 0.1 2019-05-03"

#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max[4] = {
  Adafruit_MAX31865(20, 21, 22, 23),
  Adafruit_MAX31865(49, 21, 22, 23),
  Adafruit_MAX31865(48, 21, 22, 23),
  Adafruit_MAX31865(47, 21, 22, 23)
};

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI  39
#define OLED_CLK   40
#define OLED_DC    41
#define OLED_CS    42
#define OLED_RESET 43
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

/* Comment out above, uncomment this block to use hardware SPI
#define OLED_DC     6
#define OLED_CS     7
#define OLED_RESET  8
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RESET, OLED_CS);
*/

int lineSpacing=12;
int xOffset = 8;
int yOffset = 4;



// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include "RTClib.h"

RTC_PCF8523 rtc;
DateTime now;

int counter = 0;

// SD Card used for data logging
#include <SD.h>

File fileSDCard;

// SD Shield
//
// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
#define chipSelectSDCard 10

bool bSDLogFail = false;
int  iToggle = 0;

#define HeaterUnit1 2
#define CoolerUnit1 3
#define HeaterUnit2 4
#define CoolerUnit2 5
#define HeaterUnit3 6
#define CoolerUnit3 7
#define HeaterUnit4 8
#define CoolerUnit4 9

void setup() {
  Serial.begin(9600);
  Serial.println(F("Serial Initialized..."));

  for (int i=0; i<4; i++) {
    max[i].begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  }

  pinMode(HeaterUnit1, OUTPUT);
  pinMode(CoolerUnit1, OUTPUT);
  pinMode(HeaterUnit2, OUTPUT);
  pinMode(CoolerUnit2, OUTPUT);
  pinMode(HeaterUnit3, OUTPUT);
  pinMode(CoolerUnit3, OUTPUT);
  pinMode(HeaterUnit4, OUTPUT);
  pinMode(CoolerUnit4, OUTPUT);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }

  displayFrame();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  display.setCursor(xOffset, yOffset+0);     // Start at top-left corner
  display.print("4 Unit PID");
  display.setCursor(xOffset, yOffset+lineSpacing);     // Start at top-left corner
  display.print("Hopkins 4xTomPort");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));     // Start at top-left corner
  display.print(VERSION);
  display.display();

  delay(2000);

  SetupSDCardOperations();    
}


void loop() {
  uint16_t rtd = max[0].readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(max[0].temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = max[0].readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    max[0].clearFault();
  }
  Serial.println();
  delay(1000);
}

void displayFrame()
{
  display.clearDisplay();
  for(int i=0; i<1; i++) {
  display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, WHITE);
  }    
}

// Initialize the SD for operations
// If the LOGGING.CSV file is not present create the file with the first line of column headings
void SetupSDCardOperations()
{   
  displayFrame();
  display.setCursor(xOffset, yOffset+(1*lineSpacing));
  display.print("*** STATUS ***  ");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));
  display.print("SD Init Start   ");
  display.display();

  if (!SD.begin(chipSelectSDCard)) {
    displayFrame();
    display.setCursor(xOffset, yOffset+(1*lineSpacing));
    display.print("*** ERROR ***   ");
    display.setCursor(xOffset, yOffset+(2*lineSpacing));
    display.print("SD Init Failed  ");
    display.display();
    while (1);
  }

  delay(2000);
  displayFrame();
  display.setCursor(xOffset, yOffset+(1*lineSpacing));
  display.print("* Test CLOCK.CSV");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));
  if (SD.exists("CLOCK.CSV")) 
  {
    //display.print("  Set Clock     ");

    fileSDCard = SD.open("CLOCK.CSV");
    if (fileSDCard) 
    {
      char *ptr3 = 0;
      
      if (fileSDCard.available())
      {
        char strClockSetting[256];
        char strClockSettingCopy[256];
        fileSDCard.read(strClockSetting, sizeof(strClockSetting));
        fileSDCard.close();
        strClockSetting[sizeof(strClockSetting)-1] = '\0';
        char *ptr1 = strchr(&strClockSetting[0],'\n');
        if (ptr1 != 0)
        {
            *ptr1++ = '\0';
        }
        Serial.println("Set Clock:");
        Serial.println(strClockSetting);

        ptr3 = strchr(ptr1,'\n');
        if (ptr3 != 0)
        {
            *ptr3++ = '\0';
            strcpy(strClockSettingCopy, ptr1);
        }
        Serial.println(ptr1);
        
        int iDateTime[6] = {0,0,0,0,0,0};
        for (int i=0; i<6; i++)
        {
          char *ptr2 = strchr(ptr1,',');
          if (ptr2 != 0)
          {
            *ptr2 = '\0';
            iDateTime[i] = atoi(ptr1);
            ptr1 = &ptr2[1];
          }
          else
          {
            if (i == 5)
              iDateTime[5] = atoi(ptr1);
            break;
          }
        }

        rtc.adjust(DateTime(iDateTime[0],iDateTime[1],iDateTime[2],iDateTime[3],iDateTime[4],iDateTime[5]));

        if(SD.exists("PRVCLOCK.CSV"))
        {
          SD.remove("PRVCLOCK.CSV");
        }       
        SD.remove("CLOCK.CSV");

        fileSDCard = SD.open("PRVCLOCK.CSV", FILE_WRITE);
        if (fileSDCard != 0) 
        {
          fileSDCard.println("\"Year\",\"Month\",\"Day\",\"Hour\",\"Minute\",\"Second\"");
          fileSDCard.println(strClockSettingCopy);
          fileSDCard.close();
        }

        //display.setCursor(xOffset, yOffset+(2*lineSpacing));
        display.print("* processed *");
        display.display();
        delay(2000);
      
      }
      else
      {
        fileSDCard.close();
      }
    } 
    
  }
  else 
  {
    display.print("* does not exist");
    display.display();
  }
  delay(2000);

// open the file for reading:
  fileSDCard = SD.open("LOGGING.CSV");
  if (fileSDCard) 
  {
    if (fileSDCard.available())
    {
    }
    fileSDCard.close();
  } 
  else
  {
    fileSDCard = SD.open("LOGGING.CSV", FILE_WRITE);
    if (fileSDCard) 
    {
      fileSDCard.println("\"Date\",\"Time\",\"Dout\",\"Din\",\"Status\"");
      fileSDCard.close();
    }
    else
    {
      displayFrame();
      display.setCursor(xOffset, yOffset+(1*lineSpacing));
      display.print("*** ERROR ***   ");
      display.setCursor(xOffset, yOffset+(2*lineSpacing));
      display.print("SD Write Failed ");
      display.display();
      while (1);
    }
  }
  SD.end();

  displayFrame();
  display.setCursor(xOffset, yOffset+(1*lineSpacing));
  display.print("*** STATUS ***  ");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));
  display.print("SD Init Finish  ");
  display.display();
  delay(2000);
  
//  OledDisplayStatusUpdate_SDLogging(F("Start Up  "));
}
