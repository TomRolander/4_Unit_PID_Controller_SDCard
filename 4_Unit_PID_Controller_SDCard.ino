
/**************************************************************************
 4xTomPort PID Controller
 **************************************************************************/

#define VERSION "Ver 0.2 2019-05-06"

#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max[4] = {
  Adafruit_MAX31865(44, 22, 23, 24),
  Adafruit_MAX31865(45, 22, 23, 24),
  Adafruit_MAX31865(46, 22, 23, 24),
  Adafruit_MAX31865(47, 22, 23, 24)
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

int prevHour = 0;
int prevMin = 0;

float prevTemp[4] = {0.0, 0.0, 0.0, 0.0};
float setPointTemp[4] = {24.00, 25.00, 26.00, 27.00};


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

#define menuPin   35
#define upPin     36
#define dnPin     37
#define enterPin  38

#define STATE_RUN                 0
#define STATE_SP1_WAIT            1
#define STATE_SP1                 2
#define STATE_SP1_ENTER_WAIT      3
#define STATE_SP1_ENTER           4
#define STATE_SP1_ENTER_UP_WAIT   5
#define STATE_SP1_ENTER_DN_WAIT   6
#define STATE_SP1_UPDATE_WAIT     7
#define STATE_SP2_WAIT            8
#define STATE_SP2                 9
#define STATE_SP2_ENTER_WAIT      10
#define STATE_SP2_ENTER           11
#define STATE_SP2_ENTER_UP_WAIT   12
#define STATE_SP2_ENTER_DN_WAIT   13
#define STATE_SP2_UPDATE_WAIT     14
#define STATE_CFG_WAIT            15
#define STATE_CFG                 16
#define STATE_CFG_ENTER_WAIT      17
#define STATE_CFG_ENTER           18
#define STATE_CFG_ENTER_UP_WAIT   19
#define STATE_CFG_ENTER_DN_WAIT   20
#define STATE_CFG_UPDATE_WAIT     21
#define STATE_RUN_WAIT            22
#define STATE_HI_PEAK_WAIT        23
#define STATE_HI_PEAK             24
#define STATE_HI_PEAK_FINISH      25
#define STATE_LO_VALY_WAIT        26
#define STATE_LO_VALY             27
#define STATE_LO_VALY_FINISH      28
#define STATE_STBY                29

int currentState = STATE_RUN;
int lastState = 0;
int savedState;

int updatingSetpoint = false;

int previousEnterButton = false;

void setup() 
{
  Wire.begin();
  
  Serial.begin(9600);
  Serial.println("Serial Initialized...");

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
  
  pinMode(menuPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);
  pinMode(dnPin, INPUT_PULLUP);
  pinMode(enterPin, INPUT_PULLUP);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("SSD1306 allocation failed");
    for(;;); // Don't proceed, loop forever
  }

  displayFrame();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  display.setCursor(xOffset, yOffset+0);     
  display.print("4 Unit PID");
  display.setCursor(xOffset, yOffset+lineSpacing);     
  display.print("Hopkins 4xTomPort");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));     
  display.print(VERSION);
  display.display();

  delay(2000);

  SetupSDCardOperations();    

// Initialize the Real Time Clock
  if (! rtc.begin()) 
  {
    displayFrame();
    display.setCursor(xOffset, yOffset+(1*lineSpacing));
    display.print("*** ERROR ***   ");
    display.setCursor(xOffset, yOffset+(2*lineSpacing));
    display.print("Couldnt find RTC");
    while (1);
  } 
  if (! rtc.initialized()) 
  {
    displayFrame();
    display.setCursor(xOffset, yOffset+(1*lineSpacing));
    display.print("*** WARN ***    ");
    display.setCursor(xOffset, yOffset+(2*lineSpacing));
    display.print("RTC isnt running");
    
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  now = rtc.now();

  displayFrame();
  display.setCursor(xOffset, yOffset+(1*lineSpacing));
  display.print("*** DATE ***    ");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));
  display.print(now.year(), DEC);
  display.print("/");
  OledDisplayPrintTwoDigits(now.month());
  display.print("/");
  OledDisplayPrintTwoDigits(now.day());
  display.print(" ");
  OledDisplayPrintTwoDigits(now.hour());
  display.print(":");
  OledDisplayPrintTwoDigits(now.minute());
  display.display();
  delay(2000);

  displayRun();
}

void displayRun()
{
  displayFrame();
  for(int i=0; i<1; i++) {
  display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, WHITE);
  }  

  display.setCursor(xOffset, yOffset+0);     
  display.print("# Temp  SetPt");
 
  for (int i=1; i<5; i++)
  {
    display.setCursor(xOffset, yOffset+(i*lineSpacing));
    display.print(i);
    display.print("       ");
    display.print(setPointTemp[i-1]);
  }
  display.display();  
}

void loop() 
{
  now = rtc.now();
  int currentHour = now.hour();
  int currentMin  = now.minute();
  float temp[4] = {0.0, 0.0, 0.0, 0.0};
  float delta[4] = {0.0, 0.0, 0.0, 0.0};
  uint8_t fault[4] = {false, false, false, false};
  char separator;

  int menuButton = digitalRead(menuPin);
  int upButton = digitalRead(upPin);
  int dnButton = digitalRead(dnPin);
  int enterButton = digitalRead(enterPin);  


  switch (currentState)  
  {
    case STATE_STBY:
      break;
      
    case STATE_RUN:
      if (menuButton == LOW)
      {
        previousEnterButton = false;
        currentState = STATE_SP1_WAIT;
        break;
      }
    
      for (int i=0; i<4; i++)
      {
        uint16_t rtd = max[i].readRTD();
      
      //  Serial.print("RTD value: "); Serial.println(rtd);
        float ratio = rtd;
        ratio /= 32768;
      //  Serial.print("Ratio = "); Serial.println(ratio,8);
      //  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
    
        temp[i] = max[i].temperature(RNOMINAL, RREF);
        // Check and print any faults
        fault[i] = max[i].readFault();
        if (fault[i]) 
        {
          Serial.print(i+1); Serial.print(" Fault 0x"); Serial.print(fault[i], HEX);
          if (fault[i] & MAX31865_FAULT_HIGHTHRESH) {
            Serial.println(" RTD High Threshold"); 
          }
          if (fault[i] & MAX31865_FAULT_LOWTHRESH) {
            Serial.println(" RTD Low Threshold"); 
          }
          if (fault[i] & MAX31865_FAULT_REFINLOW) {
            Serial.println(" REFIN- > 0.85 x Bias"); 
          }
          if (fault[i] & MAX31865_FAULT_REFINHIGH) {
            Serial.println(" REFIN- < 0.85 x Bias - FORCE- open"); 
          }
          if (fault[i] & MAX31865_FAULT_RTDINLOW) {
            Serial.println(" RTDIN- < 0.85 x Bias - FORCE- open"); 
          }
          if (fault[i] & MAX31865_FAULT_OVUV) {
            Serial.println(" Under/Over voltage"); 
          }
          max[i].clearFault();
        }
        else
        {
          Serial.print(i+1); Serial.print( " Temp = "); Serial.print(temp[i]); 
          Serial.print(" Delta "); Serial.println(temp[i] - setPointTemp[i]);      
        }
      }
      
      delay(1000);
    
      counter++;
      if ((counter & 1) == 1)
      {
        digitalWrite(HeaterUnit1, HIGH);
        digitalWrite(CoolerUnit1, LOW);    
        digitalWrite(HeaterUnit2, HIGH);
        digitalWrite(CoolerUnit2, LOW);    
        digitalWrite(HeaterUnit3, HIGH);
        digitalWrite(CoolerUnit3, LOW);    
        digitalWrite(HeaterUnit4, HIGH);
        digitalWrite(CoolerUnit4, LOW);    
        separator = ':';      
      }
      else
      {
        digitalWrite(HeaterUnit1, LOW);
        digitalWrite(CoolerUnit1, HIGH);    
        digitalWrite(HeaterUnit2, LOW);
        digitalWrite(CoolerUnit2, HIGH);    
        digitalWrite(HeaterUnit3, LOW);
        digitalWrite(CoolerUnit3, HIGH);    
        digitalWrite(HeaterUnit4, LOW);
        digitalWrite(CoolerUnit4, HIGH);  
        separator = ' ';      
      }
    
      if (prevHour != currentHour || prevMin != currentMin)
      {
        display.fillRect(xOffset+(14*(5+1)), yOffset+0,5*(5+1),8,BLACK);
        display.setCursor(xOffset+(14*(5+1)), yOffset+0);     
        OledDisplayPrintTwoDigits(now.hour());
        display.print(separator);
        OledDisplayPrintTwoDigits(now.minute());
      }
      else
      {
        display.fillRect(xOffset+(16*(5+1)), yOffset+0,1*(5+1),8,BLACK);
        display.print(separator);    
      }
    
      for (int i=0; i<4; i++)
      {
        if (fault[i])
        {
            ;
        }
        else
        {
          if (temp[i] != prevTemp[i])
          {
            prevTemp[i] = temp[i];
            display.fillRect(xOffset+(2*(5+1)), yOffset+((i+1)*lineSpacing),5*(5+1),8,BLACK);
            display.setCursor(xOffset+(2*(5+1)), yOffset+((i+1)*lineSpacing));
            display.print(temp[i]);
    
            delta[i] = temp[i] - setPointTemp[i];
            display.fillRect(xOffset+(14*(5+1)), yOffset+((i+1)*lineSpacing),5*(5+1),8,BLACK);
            display.setCursor(xOffset+(14*(5+1)), yOffset+((i+1)*lineSpacing));
            if (delta[i] >= 0.0)
              display.print('+');
            display.print(delta[i]);      
          } 
       }
      } 
      display.display();  
      break;

    case STATE_SP1_WAIT:
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP 1");
      if (menuButton == HIGH)
        currentState = STATE_SP1;
      break;
    
    case STATE_SP1:
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP 1");
      if (menuButton == LOW)
        currentState = STATE_SP2_WAIT;
      else
      if (enterButton == LOW)
        currentState = STATE_SP1_ENTER_WAIT;
      break;
      
    case STATE_SP2_WAIT:
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     // Start at top-left corner
      display.print("SP 2");
      if (menuButton == HIGH)
        currentState = STATE_SP2;
      break;
    
    case STATE_SP2:
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     // Start at top-left corner
      display.print("SP 2");
      if (menuButton == LOW)
        currentState = STATE_CFG_WAIT;
      else
      if (enterButton == LOW)
        currentState = STATE_SP2_ENTER_WAIT;
      break;
      
    case STATE_CFG_WAIT:
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     // Start at top-left corner
      display.print("CNFG");    
      if (menuButton == HIGH)
        currentState = STATE_CFG;
      break;
    
    case STATE_CFG:
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     // Start at top-left corner
      display.print("CNFG");    
      if (menuButton == LOW)
        currentState = STATE_RUN_WAIT;
      else
      if (enterButton == LOW)
        currentState = STATE_CFG_ENTER_WAIT;
      break;

    case STATE_RUN_WAIT:
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     // Start at top-left corner
      display.print("RUN");    
      if (menuButton == HIGH)
      {
        currentState = STATE_RUN;
        displayRun();
      }
      break;

    default:
      break;
  }

  display.display();
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
  
  OledDisplayStatusUpdate_SDLogging(F("Start Up"));
}

void OledDisplayStatusUpdate_SDLogging(const __FlashStringHelper*status)
{
  displayFrame();
  display.setCursor(xOffset, yOffset+(1*lineSpacing));
  display.print("*** LOGGING ***");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));
  display.print(status);

  now = rtc.now();

  Serial.print("STATUS: ");
  Serial.print(status);
  Serial.print("  ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.println(now.second());

  display.setCursor(xOffset, yOffset+(3*lineSpacing));
  display.print(" ");
  OledDisplayPrintTwoDigits(now.hour());
  display.print(":");
  OledDisplayPrintTwoDigits(now.minute());   
  display.print(":");
  OledDisplayPrintTwoDigits(now.second());   
  display.display();
  delay(2000);
  
  if (!SD.begin(chipSelectSDCard)) 
  {
    bSDLogFail = true;
    iToggle++;
    if ((iToggle & B00000001) == 0)
    {
      display.setCursor(xOffset, yOffset+(2*lineSpacing));
      display.print("SD LogFail");
    }
    return;
  }
  bSDLogFail = false;
  iToggle = 0;

//  if ((strcmp((const char*) status, "") != 0) || bForceOneMinuteLogging)
  {
    fileSDCard = SD.open("LOGGING.CSV", FILE_WRITE);
  
    // if the file opened okay, write to it:
    if (fileSDCard) 
    {
      fileSDCard.print(now.year(), DEC);
      fileSDCard.print("/");
      fileSDCard.print(now.month(), DEC);
      fileSDCard.print("/");
      fileSDCard.print(now.day(), DEC);
      fileSDCard.print(",");
      fileSDCard.print(now.hour(), DEC);
      fileSDCard.print(":");
      fileSDCard.print(now.minute(), DEC);
      fileSDCard.print(":");
      fileSDCard.print(now.second(), DEC);
      fileSDCard.print(",");
//      SDPrintBinary(digitalOutputState,5);
      fileSDCard.print(",");
//      SDPrintBinary(digitalInputState_Saved,4);
      fileSDCard.print(",");
      fileSDCard.print(status);
      fileSDCard.println("");
      fileSDCard.close();
      SD.end();
    } 
    else 
    {
      // if the file didn't open, display an error:
      displayFrame();
      display.setCursor(xOffset, yOffset+(1*lineSpacing));
      display.print("*** ERROR ***   ");
      display.setCursor(xOffset, yOffset+(2*lineSpacing));
      display.print("Open LOGGING.CSV");
      display.display();
      delay(2000);
    }  
  }
}

void OledDisplayPrintTwoDigits(int iVal)
{
  if (iVal < 10)
    display.print("0");
  display.print(iVal, DEC);
}
