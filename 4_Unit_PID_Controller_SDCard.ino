
/**************************************************************************
 4xTomPort PID Controller
 **************************************************************************/

#define VERSION "Ver 0.8 2019-06-17"


int maxRTD=4;

#define DELAY_DIVISOR 16    // compensate for the change of frequency for Timer 0

#define DELAY_BETWEEN_UPDATES 10000
#define DELAY_BETWEEN_LOGGING 60000

#define MINIMUM_COOL  60000

int iCoolUpdates = 0;

unsigned long timeLastPID = 0;
unsigned long timeLastLog = 0;

/********************************************************
 * PID Proportional on measurement Example
 * Setting the PID to use Proportional on measurement will 
 * make the output move more smoothly when the setpoint 
 * is changed.  In addition, it can eliminate overshoot
 * in certain processes like sous-vides.
 ********************************************************/

#include <PID_v1.h>

//Define Variables we'll be connecting to with the PID library
double Setpoint[4] = {22.0, 22.0, 22.0, 22.0};
double Input[4] = {0.0, 0.0, 0.0, 0.0};
double Output[4] = {0.0, 0.0, 0.0, 0.0};

double SetpointNew;

double prevTemp[4] = {0.0, 0.0, 0.0, 0.0};
double prevSetpoint[4] = {31.0, 31.0, 31.0, 31.0};

double offsetTemp[4] = {-0.16, 0.06, -0.2, 0.12};

double Kp = 2;
double Ki = 5;
double Kd = 1;

// From: https://www.diva-portal.org/smash/get/diva2:678519/FULLTEXT01.pdf
#if 0
double Kp = 14.4;
double Ki = 6;
double Kd = 1.5;
#endif 

int POn = P_ON_E;
int Direction[4] = {DIRECT, DIRECT, DIRECT, DIRECT};

//Specify the links and initial tuning parameters
#if 0
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
                                                                   //P_ON_E (Proportional on Error) is the default behavior
#endif

PID myPID[4] = {
  PID(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, POn, Direction[0]),
  PID(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, POn, Direction[1]),
  PID(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, POn, Direction[2]),
  PID(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, POn, Direction[3])
};

unsigned int Setpoints_Thousandths[4][240];

double DeltaIncrement = 0.5;

#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max[4] = {
  Adafruit_MAX31865(44, 32, 33, 34),
  Adafruit_MAX31865(45, 32, 33, 34),
  Adafruit_MAX31865(46, 32, 33, 34),
  Adafruit_MAX31865(47, 32, 33, 34)
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
#define STATE_SP_WAIT             1
#define STATE_SP                  2
#define STATE_SP_ENTER_WAIT       3
#define STATE_SP_ENTER            4
#define STATE_SP_ENTER_UP_WAIT    5
#define STATE_SP_ENTER_DN_WAIT    6
#define STATE_SP_UPDATE_WAIT      7
#define STATE_CFG_WAIT            8
#define STATE_CFG                 9
#define STATE_CFG_ENTER_WAIT      10
#define STATE_CFG_ENTER           11
#define STATE_CFG_ENTER_UP_WAIT   12
#define STATE_CFG_ENTER_DN_WAIT   13
#define STATE_CFG_UPDATE_WAIT     14
#define STATE_RUN_WAIT            15
#define STATE_HI_PEAK_WAIT        16
#define STATE_HI_PEAK             17
#define STATE_HI_PEAK_FINISH      18
#define STATE_LO_VALY_WAIT        19
#define STATE_LO_VALY             20
#define STATE_LO_VALY_FINISH      21
#define STATE_STBY                22

int HeaterUnits[4] = {HeaterUnit1, HeaterUnit2, HeaterUnit3, HeaterUnit4};
int CoolerUnits[4] = {CoolerUnit1, CoolerUnit2, CoolerUnit3, CoolerUnit4};

int currentState = STATE_RUN;
int currentSetpoint = 0;
int lastState = 0;
int savedState;

int updatingSetpoint = false;

int previousEnterButton = false;

int menuButton;
int upButton;
int dnButton;
int enterButton;

int menuButtonLast = HIGH;
int upButtonLast = HIGH;
int dnButtonLast = HIGH;
int enterButtonLast = HIGH;

#define BUTTON_HOLD_TIME 3000
#define BUTTON_HOLD_DELTA 200

int buttonDownStartTime = 0;
int buttonDownCurrentTime = 0;
int buttonDownIncrementTime = 0;


void setup() 
{
  Wire.begin();
  
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Serial Initialized...");
  
//For Arduino Mega1280, Mega2560, MegaADK, Spider or any other board using ATmega1280 or ATmega2560

//---------------------------------------------- Set PWM frequency for D4 & D13 ------------------------------
  
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


//---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
  
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz


//---------------------------------------------- Set PWM frequency for D2, D3 & D5 ---------------------------
  
//TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR3B = TCCR3B & B11111000 | B00000010;    // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR3B = TCCR3B & B11111000 | B00000011;    // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR3B = TCCR3B & B11111000 | B00000100;    // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
  TCCR3B = TCCR3B & B11111000 | B00000101;    // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz

  
//---------------------------------------------- Set PWM frequency for D6, D7 & D8 ---------------------------
  
//TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR4B = TCCR4B & B11111000 | B00000010;    // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR4B = TCCR4B & B11111000 | B00000011;    // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR4B = TCCR4B & B11111000 | B00000100;    // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
  TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz


//---------------------------------------------- Set PWM frequency for D44, D45 & D46 ------------------------
  
//TCCR5B = TCCR5B & B11111000 | B00000001;    // set timer 5 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR5B = TCCR5B & B11111000 | B00000010;    // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR5B = TCCR5B & B11111000 | B00000011;    // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR5B = TCCR5B & B11111000 | B00000100;    // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR5B = TCCR5B & B11111000 | B00000101;    // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz
  

  for (int i=0; i<maxRTD; i++) {
    max[i].begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  }

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

  delay(5000/DELAY_DIVISOR);

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
  delay(2000/DELAY_DIVISOR);

  Serial.println("Tuning Parameters");
  Serial.print(" Kp = ");
  Serial.println(Kp);
  Serial.print(" Ki = ");
  Serial.println(Ki);
  Serial.print(" Kd = ");
  Serial.println(Kd);
  Serial.print(" Prop on ");
  if (POn == P_ON_E)
    Serial.println("Error");
  else
    Serial.println("Measure"); 

  displayFrame();
  display.setCursor(xOffset, yOffset+(0*lineSpacing));
  display.print("Tuning Parameters");
  display.setCursor(xOffset, yOffset+(1*lineSpacing));
  display.print(" Kp = ");
  display.print(Kp);
  display.setCursor(xOffset, yOffset+(2*lineSpacing));
  display.print(" Ki = ");
  display.print(Ki);
  display.setCursor(xOffset, yOffset+(3*lineSpacing));
  display.print(" Kd = ");
  display.print(Kd);
  display.setCursor(xOffset, yOffset+(4*lineSpacing));
  display.print(" Prop on ");
  if (POn == P_ON_E)
    display.print("Error");
  else
    display.print("Measure"); 
  display.display();
  delay(2000/DELAY_DIVISOR);

  displayRun();

  //turn the PID on
  for (int i=0; i<4; i++)
  {
    //initialize the variables we're linked to
    Input[i] = 22.0;
    Setpoint[i] = 22.0;
    myPID[i].SetMode(AUTOMATIC);
    myPID[i].SetOutputLimits(25.0,127.5);
    Direction[i] = DIRECT;
    myPID[i].SetControllerDirection(Direction[i]);
  }
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
    display.print(Setpoint[i-1]);
  }
  display.display();  
}

void loop() 
{  
  now = rtc.now();
  int currentHour = now.hour();
  int currentMin  = now.minute();
  int currentSec  = now.second();

  unsigned long timeCurrent = millis() * DELAY_DIVISOR;
    
  double temp[4] = {0.0, 0.0, 0.0, 0.0};
  double delta[4] = {0.0, 0.0, 0.0, 0.0};
  uint8_t fault[4] = {false, false, false, false};

  menuButton = digitalRead(menuPin);
  upButton = digitalRead(upPin);
  dnButton = digitalRead(dnPin);
  enterButton = digitalRead(enterPin);  

  switch (currentState)  
  {
    case STATE_STBY:
      break;
      
    case STATE_RUN:
      if (menuButton == LOW)
      {
        previousEnterButton = false;
        currentState = STATE_SP_WAIT;
        currentSetpoint = 0;
        break;
      }

//Serial.print(timeCurrent); Serial.print(" "); Serial.print(timeLastPID); Serial.print(" "); Serial.print(timeCurrent-timeLastPID); Serial.print(" "); Serial.println(DELAY_BETWEEN_UPDATES); 
      if ((timeCurrent - timeLastPID) < DELAY_BETWEEN_UPDATES)
      {
        char separator;
        counter++;
        if ((counter & 1) == 1)
          separator = ':';      
        else
          separator = ' '; 
               
        if (prevHour != currentHour || prevMin != currentMin)
        {
          display.fillRect(xOffset+(14*(5+1)), yOffset+0,5*(5+1),8,BLACK);
          display.setCursor(xOffset+(14*(5+1)), yOffset+0);     
          OledDisplayPrintTwoDigits(now.hour());
          display.print(separator);
          OledDisplayPrintTwoDigits(now.minute());
          prevHour = currentHour;
          prevMin = currentMin;
        }
        else
        {
          display.fillRect(xOffset+(16*(5+1)), yOffset+0,1*(5+1),8,BLACK);
          display.setCursor(xOffset+(16*(5+1)), yOffset+0);     
          display.print(separator);    
        }
        display.display();
          
        delay(1000/DELAY_DIVISOR);
        return;    
      }
    
      timeLastPID = timeCurrent;

      for (int i=0; i<maxRTD; i++)
      {
        char szUnit[] = "Unit?";
        szUnit[4] = char ('1'+i);
        
        // Update the Setpoint from the table
        int index = (currentHour*10) + ((currentMin*10)/60);
        Setpoint[i] = double(Setpoints_Thousandths[i][index]) / 1000.0;
        //Serial.print("Setpoint: ");
        //Serial.println(Setpoint[i],3);

        if (Setpoint[i] != prevSetpoint[i])
        {
          display.fillRect(xOffset+(8*(5+1)), yOffset+((i+1)*lineSpacing),5*(5+1),8,BLACK);
          display.setCursor(xOffset+(8*(5+1)), yOffset+((i+1)*lineSpacing));
          display.print(Setpoint[i]);
          prevTemp[i] = 0.0;  // force temp update 
          prevSetpoint[i] = Setpoint[i]; 
        }
        
        uint16_t rtd = max[i].readRTD();
      
      //  Serial.print("RTD value: "); Serial.println(rtd);
        float ratio = rtd;
        ratio /= 32768;
      //  Serial.print("Ratio = "); Serial.println(ratio,8);
      //  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
    
        Serial.print(now.year(), DEC);
        Serial.print("/");
        Serial.print(now.month(), DEC);
        Serial.print("/");
        Serial.print(now.day(), DEC);
        Serial.print(" ");
        if (now.hour() < 10) Serial.print("0");
        Serial.print(now.hour(), DEC);
        Serial.print(":");
        if (now.minute() < 10) Serial.print("0");
        Serial.print(now.minute(), DEC);
        Serial.print(":");
        if (now.second() < 10) Serial.print("0");
        Serial.print(now.second(), DEC);
        Serial.print(" ");

        temp[i] = max[i].temperature(RNOMINAL, RREF);

        temp[i] += offsetTemp[i];
        
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
          Serial.print(" Delta "); Serial.print(temp[i] - Setpoint[i]);      
        }

        if (fault[i])
        {
          if ((timeCurrent - timeLastLog) >= DELAY_BETWEEN_LOGGING)
          {
//            timeLastLog = timeCurrent;          
            SDLogging(szUnit, Setpoint[i], 0, fault[i], 0, "FAULT");
          }
        }
        else
        {
          Input[i] = temp[i];

          char *strHeatingOrCooling;
          double gap = abs(Setpoint[i]-Input[i]); //distance away from setpoint
          int bOff = 0; 

#if 0
//////////////////  NEW ALGORITM
          if (Direction[i] == DIRECT &&
              Input[i] < Setpoint[i])
          {
              myPID[i].SetControllerDirection(Direction[i]);
              analogWrite(CoolerUnits[i],0); 
              strHeatingOrCooling = "Heat";           
          }
#endif

          if (gap <= 0.1 &&
              ((Direction[i] == DIRECT && Input[i] >= Setpoint[i]) ||
               (Direction[i] == REVERSE && iCoolUpdates >= (MINIMUM_COOL/DELAY_BETWEEN_UPDATES))))
          {
            // PWM of 0 when within 0.1C
            Output[i] = 0.0;
            analogWrite(CoolerUnits[i],0.0); 
            analogWrite(HeaterUnits[i],0.0);                         
            strHeatingOrCooling = "OFF ";
            bOff = 1;
            iCoolUpdates = 0;
          }
          else
          {            
            if (Input[i] <= Setpoint[i])
            {
              Direction[i] = DIRECT;
              myPID[i].SetControllerDirection(Direction[i]);
              strHeatingOrCooling = "Heat";
            }
            else
            {            
              Direction[i] = REVERSE;
              myPID[i].SetControllerDirection(Direction[i]);
              strHeatingOrCooling = "Cool";
            }
            myPID[i].Compute();
          }
          Serial.print("  Setpoint = "); Serial.print(Setpoint[i]); Serial.print(", "); 
          Serial.print(strHeatingOrCooling); 
          double DutyCycle = (Output[i]/255.0)*100;
          Serial.print(", DutyCycle = "); Serial.print(DutyCycle); Serial.println("%");
          
//          if (gap > 0.1)
          if (bOff == 0)
          {
            if (Input[i] <= Setpoint[i])
            {
              analogWrite(HeaterUnits[i],Output[i]); 
              analogWrite(CoolerUnits[i],0.0); 
              iCoolUpdates = 0;
            }
            else
            {
#if 1 // Force 100% Duty Cycle for cooling
              Output[i] = 255.0;
#endif
              analogWrite(CoolerUnits[i],Output[i]); 
              analogWrite(HeaterUnits[i],0.0);                         
              iCoolUpdates++;
            }
          }

          if ((timeCurrent - timeLastLog) >= DELAY_BETWEEN_LOGGING)
          {
            if (i+1 >= maxRTD)
              timeLastLog = timeCurrent;          
            SDLogging(szUnit, Setpoint[i], temp[i], (temp[i] - Setpoint[i]), Output[i], strHeatingOrCooling);
          }
          
          if (temp[i] != prevTemp[i])
          {
            prevTemp[i] = temp[i];
            display.fillRect(xOffset+(2*(5+1)), yOffset+((i+1)*lineSpacing),5*(5+1),8,BLACK);
            display.setCursor(xOffset+(2*(5+1)), yOffset+((i+1)*lineSpacing));
            display.print(temp[i]);
    
            delta[i] = temp[i] - Setpoint[i];
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

    case STATE_SP_WAIT:
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP ");
      display.print(currentSetpoint+1);
      display.display();  
      if (menuButton == HIGH)
        currentState = STATE_SP;
      break;
    
    case STATE_SP:
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP ");
      display.print(currentSetpoint+1);
      display.display();  
      if (menuButton == LOW)
      {
        currentState = STATE_SP_WAIT;
        currentSetpoint++;
        if (currentSetpoint >= 4)
          currentState = STATE_CFG_WAIT;         
      }
      else
      if (enterButton == LOW)
        currentState = STATE_SP_ENTER_WAIT;
      break;
      
    case STATE_SP_ENTER_WAIT:
      if (enterButton == HIGH)
      {
        currentState = STATE_SP_ENTER;
        SetpointNew = Setpoint[currentSetpoint];
      }
      break;
    
    case STATE_SP_ENTER:
      if (menuButton == LOW)
        currentState = STATE_SP_WAIT;
      else
      if (upButton == LOW)
      {
        SetpointNew += 0.1;
        buttonDownIncrementTime = 0;
        currentState = STATE_SP_ENTER_UP_WAIT;
      }
      else
      if (dnButton == LOW)
      {
        SetpointNew -= 0.1;
        buttonDownStartTime = millis() * DELAY_DIVISOR;
        buttonDownIncrementTime = 0;
        currentState = STATE_SP_ENTER_DN_WAIT;
      }
      else
      if (enterButton == LOW)
        currentState = STATE_SP_UPDATE_WAIT;
        
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP ");
      display.print(currentSetpoint+1);
      display.setCursor(xOffset+(2*(5+1)), yOffset+(2*lineSpacing));     
      display.print(SetpointNew);
      display.display();  
      break;
    
    case STATE_SP_ENTER_UP_WAIT:
      if (upButton == HIGH)
        currentState = STATE_SP_ENTER;
      else
      {
        buttonDownCurrentTime = millis() * DELAY_DIVISOR;
        if (buttonDownCurrentTime > (buttonDownStartTime + BUTTON_HOLD_TIME))
        {
          if (buttonDownIncrementTime == 0 ||
              buttonDownCurrentTime > (buttonDownIncrementTime + BUTTON_HOLD_DELTA))
          {
            SetpointNew += 0.1;
            buttonDownIncrementTime = buttonDownCurrentTime;
          }
        }
      }
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP ");
      display.print(currentSetpoint+1);
      display.setCursor(xOffset+(2*(5+1)), yOffset+(2*lineSpacing));     
      display.print(SetpointNew);
      display.display();  
      break;
    
    case STATE_SP_ENTER_DN_WAIT:
      if (dnButton == HIGH)
        currentState = STATE_SP_ENTER;
      else
      {
        buttonDownCurrentTime = millis() * DELAY_DIVISOR;
        if (buttonDownCurrentTime > (buttonDownStartTime + BUTTON_HOLD_TIME))
        {
          if (buttonDownIncrementTime == 0 ||
              buttonDownCurrentTime > (buttonDownIncrementTime + BUTTON_HOLD_DELTA))
          {
            SetpointNew -= 0.1;
            buttonDownIncrementTime = buttonDownCurrentTime;
          }
        }
      }
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP ");
      display.print(currentSetpoint+1);
      display.setCursor(xOffset+(2*(5+1)), yOffset+(2*lineSpacing));     
      display.print(SetpointNew);
      display.display();  
      break;
    
    case STATE_SP_UPDATE_WAIT:
      displayFrame();
      display.setCursor(xOffset+(2*(5+1)), yOffset+(1*lineSpacing));     
      display.print("SP ");
      display.print(currentSetpoint+1);
      display.setCursor(xOffset+(2*(5+1)), yOffset+(2*lineSpacing));     
      display.print("StRd");
      display.display();  
      if (enterButton == HIGH)
      {
        Serial.println("Updating setpoint table.");
        Serial.print("currentSetpoint = "); Serial.println(currentSetpoint);
        for (int iSetpoint=0; iSetpoint<240; iSetpoint++)
        {
          Setpoints_Thousandths[currentSetpoint][iSetpoint] = (unsigned int) (SetpointNew*1000.0);
        }
        
        Setpoint[currentSetpoint] = SetpointNew;
        currentState = STATE_SP;
        currentSetpoint++;
        if (currentSetpoint >= 4)
          currentState = STATE_CFG_WAIT;         
      }
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
        for (int i=0; i<maxRTD; i++)
          prevTemp[i] = 0.0;       
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
  for(int i=0; i<maxRTD; i++) {
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
    display.setCursor(xOffset, yOffset+(2*lineSpacing));
    display.print("System HALTED!");
    display.display();
    while (1);
  }
  delay(2000/DELAY_DIVISOR);

  
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
        delay(2000/DELAY_DIVISOR);
      
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
  delay(2000/DELAY_DIVISOR);

// CONTROL.CSV Processing
  displayFrame();
  display.setCursor(xOffset, yOffset+(1*lineSpacing));
  display.print("* Test CONTROL.CSV");
  display.setCursor(xOffset, yOffset+(2*lineSpacing));
  if (SD.exists("CONTROL.CSV")) 
  {
    fileSDCard = SD.open("CONTROL.CSV");
    if (fileSDCard) 
    {
      char *ptr3 = 0;
      
      if (fileSDCard.available())
      {
        char strControlSetting[256];
        char strControlSettingCopy[256];
        fileSDCard.read(strControlSetting, sizeof(strControlSetting));
        fileSDCard.close();
        strControlSetting[sizeof(strControlSetting)-1] = '\0';
        char *ptr1 = strchr(&strControlSetting[0],'\n');
        if (ptr1 != 0)
        {
            *ptr1++ = '\0';
        }
        Serial.println("Set Control:");
        Serial.println(strControlSetting);

        ptr3 = strchr(ptr1,'\n');
        if (ptr3 != 0)
        {
            *ptr3++ = '\0';
            strcpy(strControlSettingCopy, ptr1);
        }
        Serial.println(ptr1);
        
        int iControl[4] = {0,0,0,0};
        for (int i=0; i<4; i++)
        {
          char *ptr2 = strchr(ptr1,',');
          if (ptr2 != 0)
          {
            *ptr2 = '\0';
            iControl[i] = atoi(ptr1);
            ptr1 = &ptr2[1];
          }
          else
          {
            if (i == 3)
              iControl[3] = atoi(ptr1);
            break;
          }
        }
        Kp = iControl[0];
        Ki = iControl[1];
        Kd = iControl[2];
        POn = iControl[3];
        
        display.print("* processed *");
        display.display();
        delay(2000/DELAY_DIVISOR);      
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
  delay(2000/DELAY_DIVISOR);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// SETPNTx.CSV Processing
  for (int iUnit=1; iUnit<=maxRTD; iUnit++)
  {
    char szSETPNTx[] = "SETPNTx.CSV";
    szSETPNTx[6] = '0'+iUnit;
    displayFrame();
    display.setCursor(xOffset, yOffset+(1*lineSpacing));
    display.print("* Test ");
    display.print(szSETPNTx);
    display.setCursor(xOffset, yOffset+(2*lineSpacing));
    if (SD.exists(szSETPNTx)) 
    {
      fileSDCard = SD.open(szSETPNTx);
      if (fileSDCard) 
      {
        char *ptr3 = 0;
        
        if (fileSDCard.available())
        {
          ReadSetpoints(&Setpoints_Thousandths[iUnit-1][0]);   

          now = rtc.now();
          int currentHour = now.hour();
          int currentMin  = now.minute();
          int index = (currentHour*10) + ((currentMin*10)/60);
          Setpoint[iUnit-1] = double(Setpoints_Thousandths[iUnit-1][index]) / 1000.0;
                       
          display.print("* processed *");
          display.display();
          delay(2000/DELAY_DIVISOR);      
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
    delay(2000/DELAY_DIVISOR);
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // LOGGING.CSV file processing
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
      fileSDCard.println("\"Date\",\"Time\",\"Unit\",\"Setpt\",\"Temp\",\"Delta\",\"Output\",\"DutyCycle\",\"Dir\"");
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
  delay(2000/DELAY_DIVISOR);
  
  SDLogging("Start Up", 0.0, 0.0, 0.0, 0.0, "");
}

void SDLogging(char *szUnit, double setpoint, double temp, double delta, double output, char *strDir)
{
  if (!SD.begin(chipSelectSDCard)) 
  {
#if 0
    bSDLogFail = true;
    iToggle++;
    if ((iToggle & B00000001) == 0)
    {
      display.setCursor(xOffset, yOffset+(2*lineSpacing));
      display.print("SD LogFail");
    }
#endif
    return;
  }
  bSDLogFail = false;
  iToggle = 0;

//  if ((strcmp((const char*) szUnit, "") != 0) || bForceOneMinuteLogging)
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
      fileSDCard.print(szUnit);
      fileSDCard.print(",");
      fileSDCard.print(setpoint);
      fileSDCard.print(",");
      fileSDCard.print(temp);
      fileSDCard.print(",");
      fileSDCard.print(delta);
      fileSDCard.print(",");
      fileSDCard.print(output);
      fileSDCard.print(",");
      double DutyCycle = (output/255.0)*100.0;
      fileSDCard.print(DutyCycle);
      fileSDCard.print("%,");
      fileSDCard.print(strDir);
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
      delay(2000/DELAY_DIVISOR);
    }  
  }
}

void OledDisplayPrintTwoDigits(int iVal)
{
  if (iVal < 10)
    display.print("0");
  display.print(iVal, DEC);
}

bool readLine(char* line, size_t maxLen) {
  for (size_t n = 0; n < maxLen; n++) {
    int c = fileSDCard.read();
    if ( c < 0 && n == 0) return false;  // EOF
    if (c < 0 || c == '\n') {
      line[n] = 0;
      return true;
    }
    line[n] = c;
  }
  return false; // line too long
}

bool readVals(int* iHour, int* iMinute, double* temp) {
  char line[40], *ptr, *str;
  if (!readLine(line, sizeof(line))) {
    return false;  // EOF or too long
  }
  ptr = &line[0];
  *iHour = atoi(ptr);
  while (*ptr) {
    if (*ptr++ == ':') break;
  }
  *iMinute = atoi(ptr);
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *temp = strtod(ptr, &str);
  return str != ptr;  // true if number found
}

void ReadSetpoints(unsigned int* Setpoints_Thousandths)
{
  int iHour, iMinute;
  int index;
  double temp;
  long iThousandths;

  while (readVals(&iHour, &iMinute, &temp)) {
    iThousandths = (unsigned int) (temp*1000.0);
    index = (iHour*10) + ((iMinute*10)/60);
    Setpoints_Thousandths[index] = iThousandths; 
#if 0
    Serial.print("Hr: ");
    Serial.print(iHour);
    Serial.print(" Mn: ");
    Serial.print(iMinute);
    Serial.print(" Temp: ");
    Serial.print(temp,3);
    Serial.print(" iThousandths: ");
    Serial.print(iThousandths);
    Serial.print(" Index: ");
    Serial.println(index);
#endif
  }
}
