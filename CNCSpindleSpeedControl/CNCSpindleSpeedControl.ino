/*
This Program when uploaded to a Arduino/Leonardo (or Uno) is a Tach based speed controller.
It was originally written to operate a 400W/48VDC CNC spindle motor through
a simple electronic controller, readily found on E-Bay
This program assumes that the +PWM lead of the controller is connected to
pin D13 of the Leonardo (or Pin D11 on the UNO), and the collector output of the speed sensor 
[OPB704 is a direct pin-for-pin replacement for the once popular,
but now discontinued, QRB1114; the Parallax Inc 550-27401 or FairChild qrd1114
might also maybe adapted to work] is connected to pin D3. 
Note this collector is also tied +5Volts through a 20K pull-up resistor.
It Furhter assumes that a Sansmart 16x2 LCD/KeyPad sheild has been installed.
This display & keypad provides a simple user interface, where the Up/Down buttons
can be used to ramp the Set Speed from 1400 to 10,000 RPM, in steps of 200 RPM, 
while the Left/Right Buttons will select the display mode.

This version of the speed control sketch incudes a "Scope" display mode. This mode
is new to this release. To access the Scope mode, press either the "Left" or "Right"
buttons on the display's key pad. Hint: using the "Left" button, it will be the first key press.
The purpose of this mode is to allow the user, to use the 16 x 2 display as a rudimentary 
Oscilloscope. The Arduino's "A1" pin acts as the Scope's input. Typically either the IR's Output signal
which is normally connected to the Arduino's interrupt pin (D3) can be also connected to A1.
This gives the user a practical way to observe the spindle's speed signal. And through this view,
the user can adjust the IR sensor's position for maximum effectiveness.
The ideal pulse should look something like what is shown here:
 /--------------------\   /-----
/                      \-/   
It may be helpful to make the initial position adjustments with the IR's output only connected to
A1 (interrupt pin D3 not connected). With this arrangement the spindle should run at a steady
speed regardless of what the sensor is outputing, and will allow the user to explore a range of positions
without the spindle speed going crazy. Once the best spot it found, add the interrupt connection, and 
then the user can "fine tune" this position by continuing to monitor the speed pulse on A1.

Note: The LCDKeypadR1 Library, contained in this file set, 
is a derivative of the files found here:

http://sainsmart.com/zen/documents/20-011-901/keypad_lcd.zip

To install this sketch, when down loaded from GitHub all the files found under the sub-directory 
"CNCSpindleSpeedControl" should be left in that directory. and the entire directory
[CNCSpindleSpeedControl] moved to the same directory as the Uer's other sketches
On a windows based computer, this is usually the /Documents/Arduino/ directory.
If setup correctly, when you open the sketch, you should see three tabs.
"CNCSpindleSpeedControl", "LCDKeypadR1.cpp", and "LCDKeypadR1.h"


Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
 */
#include <LiquidCrystal.h>
#include "LCDKeypadR1.h"

#if defined(__AVR_ATmega32U4__)
  //Code in here will only be compiled if an Arduino Leonardo is used.
  #define TimerReg   TCCR4B
  #define BoardType 0
  #define InterruptId 0
#endif
#if defined(__AVR_ATmega16U4__)
  //Code in here will only be compiled if an Arduino Uno is used.
  #define TimerReg  TCCR2B
  #define BoardType 1
  #define InterruptId 1
#endif

#if defined(__AVR_ATmega328P__)
  //Code in here will only be compiled if an Arduino Uno is used.
  #define TimerReg  TCCR2B
  #define BoardType 1
  #define Interrupt 1
  #define InterruptId 1
#endif

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); //initialize Sansmart LCD Display
LCDKeypad Keypad;
int trgtRPM = 5000;
unsigned long timeval = 0; 
unsigned long Start = 0; 
unsigned long Stop; 
unsigned long WaitInterval =0;
unsigned long startPrgm = millis()/1000; 
unsigned long now; 
int SampleCnt = 10; // Number Spindle Relolutions needed to make a new RPM calculation
unsigned long period;
unsigned long Maxperiod;

int PWMpin; // Digital Pin that PWM signal appears on (Pin 13 = Leonardo; Pin 11 = Uno)
int calcrpm = 0;
int sensorPin = A1; // Used for Oscope Mode (mode 3)
int IRAnalogBuf[80];// Buffer/Array to hold Scaled analog data Used for Oscope Mode (mode 3)
// -----------------PID/PWM Constants -----------------
unsigned long lastTime;
double Input, SetPoint;
double ITerm, lastInput;
double kp, ki, kd;
double Kp = 0.02;//0.06;
double Ki = 0.05;//0.05;
double Kd = 0.0005;//0.0005;
double PIDOut = 0.0;
double Max = 254.0; //max allowed PWM value
double Min = 1.0; //min allowed PWM value
int SampleTime = 60;// measured in milliseconds [FWIW: 10,000 RPM measured over 10 revolutions = 60 ms]
int MstrSampleTime = SampleTime;

// -----------------End PID Constants -----------------
long int TotalRunTime = 0;
int MaxRunTime = 0;//60*60*10;// number of seconds the program is allowed to run be
int SpndlPWM;
int newdutycycle = 0;
int dutycycle = 30; // percentage of time clock is high
//int StrtDC = 0; // the calculated Start dutycycle needed to hit the target rpm $

//int LastRPM = 0;
//int AltRPM = 0;
//int SpeedChange = 0;
//int SuspectRdCnt = 0;
//bool increaseSpeed = true;
bool NewTrgtVal = true; //used to determine if display needs updating
bool ExitNow = false;
bool NuInput = true;
//char *intFlgStat;
//int ThrtlLpCnt= 0;
//int Startup = 0; // Locks out pid calcs on startup to give the default throttle setting a chance to take effect before correcting it
//int CalcIntrvl = 0;
int BackLight = 6;  // set Display BackGround color to 1=Red; 2=Green; 4=Blue
int Mode = 7;

// the event counter
volatile int eventCounter = 0;
volatile int LoopCounter = 0;

char buf [16]; // used to concatenate strings & numerical data for display on SanSmart LCD
         
// Display Custom characters for Throttle Bar Graph Mode
static unsigned char Char0 [8] =
{
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
} ;

static unsigned char Char1 [8] =
{
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000,
  0b10000
} ;

static unsigned char Char2 [8] =
{
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000,
  0b11000
} ;

static unsigned char Char3 [8] =
{
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100,
  0b11100
} ;

static unsigned char Char4 [8] =
{
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110,
  0b11110,
} ;

static unsigned char Char5 [8] =
{
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
} ;


// O-Scope bitmap
static unsigned char Char0101 [16];
static unsigned char Char0102 [16];
static unsigned char Char0103 [16];
static unsigned char Char0104 [16];
static unsigned char Char0105 [16];
static unsigned char Char0106 [16];
static unsigned char Char0107 [16];
static unsigned char Char0108 [16];
static unsigned char Char0109 [16];
static unsigned char Char0110 [16];
static unsigned char Char0111 [16];
static unsigned char Char0112 [16];
static unsigned char Char0113 [16];
static unsigned char Char0114 [16];
static unsigned char Char0115 [16];
static unsigned char Char0116 [16];
static unsigned char Trace1 [8];
static unsigned char Trace2 [8];
static unsigned char Trace3 [8];
static unsigned char Trace4 [8];
static unsigned char Trace5 [8];
static unsigned char Trace6 [8];
static unsigned char Trace7 [8];
static unsigned char Trace8 [8];
static unsigned char TstBtMap [8];


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// SpindleTachInterrupt:  called every time an event occurs
void SpindleTachInterrupt(void)
{
//   detachInterrupt(InterruptId);
   unsigned long ThisIntTime = micros();
   if (ThisIntTime < WaitInterval) return;
   WaitInterval = ThisIntTime+3000; //move the next valid interrupt out by 3 milliSeconds (the squivalent of the spindle turning @ 20K RPM )
   if (eventCounter == 1)
     {
      Start = ThisIntTime;
     }
   if (eventCounter == SampleCnt+1)
     {
      period = ThisIntTime-Start;
     }
   eventCounter +=1;
   if (eventCounter >= SampleCnt+2)
    {
    if (period ==0) sprintf (buf,"Too Small");
    else
     {
      //int Freq  = (SampleCnt*1000000)/period; //Simple Frequency Counter (Hz) code
      //sprintf (buf,"Hz: %d  ", Freq); //Simple Frequency Counter code
     calcrpm =  (SampleCnt*60000000)/period;
     if (calcrpm <=2800.0) SampleCnt = 2;
     else SampleCnt = 10;
      
     }
     SampleTime = MstrSampleTime; 
     SetTunings(Kp, Ki, Kd);
     SpndlPWM = CalcMtrPID(calcrpm);//Max+CalcMtrPID(calcrpm);
     analogWrite(PWMpin, SpndlPWM);
     dutycycle = (int)((SpndlPWM*100)/(Max));//(int)((SpndlPWM*100)/(2*Max));// convert PWM signal to a precentage
    LoopCounter = 1;//reset loopcounter so main loop will know that Motor is still running
    eventCounter = 0;
    }
//   delay(3); //wait at least the equivalent of 20K rpm before reacting to another spindle interrupt
//    attachInterrupt(InterruptId, SpindleTachInterrupt, FALLING);
}

// ================================================================
// ===               END INTERRUPT DETECTION ROUTINE            ===
// ================================================================

// -------------------------------------------------------------------------

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  byte Divisor;
  //Serial1.begin(9600); // enable when [Leonardo] bluetooth diagnostic is needed
  Serial.begin(9600);  // enable when/if diagnostic is needed
   // set up the lcd's number of columns and rows: 
  lcd.begin(16, 2);
  //calcrpm =  (SampleCnt*1000000*60)/period;//Formula used in interrupt routine to estimate current Spindle RMPM
  calcrpm = 500; //minimum expected RPM
  Maxperiod =(SampleCnt*1000000*60)/calcrpm;
 
  pinMode(3, INPUT_PULLUP);//Set Interrupt input pin to have an active pullupresistor
 if (BoardType == 0) // Leonardo
  {
   PWMpin = 13; 
   Divisor = 0x06;//0x06 original divisor used on Leonardo; new faster pwm divisor 0x04
   //Serial.println("Leonardo");
  }
  else //Uno
  {
   PWMpin = 11;
   Divisor = 0x03;
   //Serial.println("Uno");
  }  
  attachInterrupt(InterruptId, SpindleTachInterrupt, FALLING); //UNO digital pin 3;
   
  TimerReg = TimerReg & 0b11111000 | Divisor;
  pinMode(PWMpin, OUTPUT);

  //lcd.setCursor(0,0);
  //lcd.print("SetUp Complete");  
  //startPrgm = 1000*millis();
  long MaxRunTime = 60*60*10;// number of seconds the program is allowed to run be
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() 
{
  int UsrInput = 0;
  int stopCnt = 0;
  lcd.setCursor(0,1);
  lcd.print("Speed Cntl Ready");
  lcd.setCursor(0,0);
  sprintf (buf,"Set RPM: %d", trgtRPM);
  lcd.print(buf);
  //If you're here [and can see, "Speed Cntl Ready" on the Display]
  //then LCD setup seemed to go OK, & you're ready to start the speed control stuff

  // Calc initial dutycycle based on trgtRPM
   SpndlPWM = (int) 95.0*(trgtRPM/9800.0)*(trgtRPM/9800.0) ; //190.5*(trgtRPM/9800.0)*(trgtRPM/9800.0)

   // display counter value every 1/4 second.
  //sprintf (buf, "Time %d Max %d\n", TotalRunTime, MaxRunTime);
  //Serial1.println(buf);
  while ( TotalRunTime <= MaxRunTime && !ExitNow )
  {
    
    if(micros()-Start>=Maxperiod ){// if true, it looks like the motor isn't turning
      eventCounter = 0;
      LoopCounter = 0;
      Start = micros();
      NewTrgtVal = true;
    }
    while (eventCounter == 0 && LoopCounter == 0 &&  TotalRunTime <= MaxRunTime && !ExitNow)
     {
      //Motor is not running, So initialize PID for next startup
      //SpndlPWM = (int) 190.5*(trgtRPM/9800.0)*(trgtRPM/9800.0) ;
      ResetSpindle();
      SpndlPWM = 15;
//      SetPoint = 2000;
//      ITerm = 0.0; //kill any residual Integral component that might be left over from a previous run
//      lastTime = millis();// reset/establish time mark for initial PID calculation   
//      analogWrite(PWMpin, SpndlPWM); //start the PWM output
      //sprintf (buf, "PWMpin:%02i PWM:%02i", PWMpin, SpndlPWM);
      //Serial.println(buf);
      //Startup, in this version of the code is not used 
      //Startup = 60; // Locks out pid calcs on startup to give the default throttle setting a chance to take effect before correcting it
      delay(100);
      if (NewTrgtVal)
       {
         lcd.setCursor(0,0); //lcd.setCursor(pos,line)
         sprintf (buf, "%s %04i ", "Trgt RPM:", trgtRPM);
         lcd.print(buf);
         lcd.setCursor(0,1);
         lcd.print("Speed Cntl Ready");
         NewTrgtVal = false;
       }
      UsrInput =  ScanButtons();
      UpDateSettings(UsrInput, Mode);
      while (Mode == 3){// Run Scope Mode Routine
      ScopeMode();
      UsrInput =  ScanButtons();
      UpDateSettings(UsrInput, Mode);
      }
     
      long TotalRunTime = calcruntime();
      //SuspectRdCnt = 3;
      stopCnt = stopCnt+1;
     } // end of 2nd inner While looop
  //   Serial1.print( "rpm: %d; trgtRPM: %d; Duty Cycle: %d; SpdChng: %d %s; EvntCnt: %d \n", calcrpm, trgtRPM,$
   if (NewTrgtVal)
    {
      lcd.setCursor(0,0); //lcd.setCursor(pos,line)
      sprintf (buf, "%s %04i ", "Trgt RPM:", trgtRPM);
      lcd.print(buf);
      NewTrgtVal = false;
    }
   lcd.setCursor(0,1);//lcdPosition (lcdHandle, 0, 1) ;
   if (Mode == 7) // Original Display Mode; Show Measured RPM, on the 2nd line
    {
     //lcdPrintf (lcdHandle, "Actl RPM: %d  ", calcrpm) ;
     sprintf (buf, "%s %04i  ", "Actl RPM:", calcrpm);
     lcd.print(buf);
    }
   if (Mode == 5) // Display Duty Cycle Mode
    {
     char DsplyStr[16];
     char BlnkChr [1];
     sprintf(DsplyStr, "Duty Cycle: %d ", dutycycle );// convert SpdEr to a String
     strncpy( BlnkChr, " ", 1);// now fill out the rest of the display message/line with Blanks
                               //[to ensure previous message has been erased
     while (strlen(DsplyStr)<16)
      {
       strcat(DsplyStr, BlnkChr);
      }
     //lcdPrintf (lcdHandle,"%s", DsplyStr);
     //sprintf (buf, "%s %04i ", "Actl RPM:", calcrpm);
     lcd.print(DsplyStr);
    }
   if (Mode == 6) // show Speed error Mode
    {
     char SpdErStr[6];
     char SgndErStr[8];
     char *pntr;
     int SpdEr =  calcrpm-trgtRPM;
     sprintf(SpdErStr, "%d",SpdEr );// convert SpdEr to a String
     pntr = strchr (SpdErStr, '-');
     if(pntr == NULL)
      {
       strncpy(SgndErStr, " +", 8);
       strcat(SgndErStr, SpdErStr);
      }
     else
      {
       strncpy(SgndErStr, " ", 8);
       strcat(SgndErStr, SpdErStr);
      }
     strncpy( SpdErStr, " ", 6);
     while (strlen(SgndErStr)<8)
      {
       strcat(SgndErStr, SpdErStr);
      }
     //lcdPrintf (lcdHandle, "Spd Err:%s", SgndErStr ) ;
     sprintf (buf, "%s%s ", "Spd Err:", SgndErStr);
     lcd.print(buf);
    }
// ############ Begin BarGraph Mode #################
   if (Mode == 4)// PWM Bar Graph Mode
    {
     int FullCell;
     int runngCnt = 0;
     int postn = 0;
     int remainder = 0;
     int rowWeight;
     FullCell =  100/16;//(Maxdutycycle - Mindutycycle)/16;
     rowWeight = FullCell/5;
//   First, fill in the whole spaces
     while (runngCnt +  FullCell<= dutycycle)
      {
       //lcdPutchar  (lcdHandle, 7) ;
       lcd.write(byte(5));
       runngCnt = runngCnt +  FullCell;
       postn =  postn + 1;
      }
//   Next, Deterime which partial charater to use
     remainder =  dutycycle- runngCnt;
    //Serial1.print("remainder: %d Dutycycle: %d\n", remainder, dutycycle-Mindutycycle);
     if(remainder>0)
      {
      postn =  postn + 1;
       if (remainder>= 4*rowWeight)
        {
          lcd.write(byte(4));//lcdPutchar  (lcdHandle, 6) ;
        }
       else
        {
         if (remainder>= 3*rowWeight)
          {
           lcd.write(byte(3));//lcdPutchar  (lcdHandle, 5) ;
          }
         else
          {
           if (remainder>= 2*rowWeight)
            {
              lcd.write(byte(2));//lcdPutchar  (lcdHandle, 4) ;
            }
           else
            {
             lcd.write(byte(1));//lcdPutchar  (lcdHandle, 3) ;
            }
          }
        }
      }

//   finally, Finish out the Bar Graph line with blanks
     while (postn < 16)
      {
        lcd.write(byte(0));//lcdPutchar  (lcdHandle, 2) ;
        postn =  postn + 1;
      }
    }
// ############ End BarGraph Mode #################
   if (Mode == 3){// Run Scope Mode Routine
    ScopeMode();
   }
// ############ End O-Scope Mode #################    
    
   UsrInput =  ScanButtons();
   UpDateSettings(UsrInput, Mode);
   delay( 250 ); // wait 1/4 second(s)
    LoopCounter = LoopCounter + 1;

    if (LoopCounter >= 8)
     {
      LoopCounter = 0;
      eventCounter = 0;
//      TooHighCnt = 0;
//      TooLowCnt = 0;
      NewTrgtVal = true;
      //Serial1.print("Looks as if Motor has Stopped\n");
     }
   long TotalRunTime = calcruntime();
  } //end of 1st while loop
 //kill the PWM signal
  detachInterrupt(InterruptId);
  digitalWrite (PWMpin, LOW);
 lcd.clear();
 lcd.setCursor(0,0); //lcd.setCursor(pos,line)
 sprintf (buf, "Program Stopped");
 lcd.print(buf);
 lcd.setCursor(0,1); //lcd.setCursor(pos,line)
 sprintf (buf, "CYCLE PWR 2 STRT");
 lcd.print(buf);
 delay(10000);
 return;
}
// ================================================================
// ===                   END MAIN PROGRAM LOOP                  ===
// ================================================================

//###########################################################################
// Begin PID computations
int CalcMtrPID(float Input)
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      //sprintf (buf, "DT %d", timeChange);
      //Serial1.println(buf);
      SetSampleTime(timeChange);
      /*Compute all the working error variables*/
      if (SetPoint < trgtRPM) SetPoint +=200;
      else if(SetPoint > trgtRPM) SetPoint -=200;
      double error = SetPoint - Input;
      double dInput = (Input - lastInput);
      double offset = 0.0; 
      double DtI = ki*error;
      double Cap = 10.0;
      if (DtI >Cap) DtI = Cap;
      else if (DtI <-Cap) DtI = -Cap;
      ITerm+= DtI;//ITerm+= (0.75*ki*error);
  
      /*Compute PID PIDOut*/
      PIDOut = kp*error + ITerm - kd * dInput;
      //sprintf (buf, "P %d\n", PIDOut);
      //Serial1.println(PIDOut);

      SetPIDOutLimits(Min, Max);
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
   return (int) PIDOut;
   
}   
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void SetPIDOutLimits(double Min, double Max)
{
   if(Min > Max) return;
//   double outMin = Min;
//   double outMax = Max;
    
//   if(PIDOut > outMax) PIDOut = outMax;
//   else if(PIDOut < outMin) PIDOut = outMin;
// 
//   if(ITerm> outMax) ITerm= outMax;
//   else if(ITerm< outMin) ITerm = outMin;

   if (PIDOut > Max)
    {
      ITerm -= PIDOut - Max;
      PIDOut = Max;
    }
   else if (PIDOut < Min)
    {
      ITerm  -=(PIDOut - Min);//+= Min - PIDOut;// -=(PIDOut - Min)
      PIDOut = Min;
    }
}
 

// End PID computations
//#######################################################################

// ================================================================
// ===              BEGIN LOCAL FUNCTION DEFINITIONS            ===
// ================================================================


static int ScanButtons()
{
  int buttons = 0;
  int buttonPressed = Keypad.button();
  if (buttonPressed==KEYPAD_UP) buttons += 1;
  if (buttonPressed==KEYPAD_DOWN) buttons += 2;
  if (buttonPressed==KEYPAD_LEFT) buttons += 4;
  if (buttonPressed==KEYPAD_RIGHT) buttons += 8;
  if (buttonPressed==KEYPAD_SELECT) buttons += 16;
  if (buttonPressed==KEYPAD_NONE) NuInput = true;
  return buttons;
}

void UpDateSettings(int UsrInput, int mode)
{
   if (UsrInput !=0)
    {
      if(UsrInput == 1 || UsrInput == 2) //[Display Up or Down Button]  Speed, $
       {
         if (mode == 3)
          {
           if(UsrInput == 1 )
            {
             if (BackLight <7)
              {
               BackLight = BackLight+1;
              }
             else
              {
               BackLight = 1;
              }
            }
           else
            {
             if (BackLight >1)
              {
               BackLight = BackLight-1;
              }
             else
              {
               BackLight = 7;
              }
            }
           //setBacklightColour (BackLight) ;
          }
         else
          {
           NewTrgtVal = true;
           if(UsrInput == 1 )
            {
             if (trgtRPM <10000)
              {
               trgtRPM = trgtRPM+200;
              }
            }
           else
            {
             if (trgtRPM >1500)
              {
                trgtRPM = trgtRPM-200;
              }
            }
          }
       }
      if((UsrInput == 4 || UsrInput == 8) && NuInput == true) //[Display Left o$
       {
//        Serial1.print("NuInput %s\n", NuInput ? "true" : "false");
        if (Mode == 3){ //user is leaving Scope mode so restore Display
          lcd.setCursor(0,0);
          lcd.clear();
          sprintf (buf,"Trgt RPM: %d", trgtRPM);
          lcd.print(buf);
        }


        NuInput = false;
        if (UsrInput == 4)
         {
           Mode = Mode +1;
           if (Mode == 8)
            {
              Mode = 3;
              //Mode = 4;
            }
         }
        else
         {
           Mode = Mode- 1;
           if (Mode == 2)
            {
              Mode = 7;
            }

         }
        //setBacklightColour (BackLight);
        if (Mode == 4) LoadBarGraphFont();//user just selected bargraph mode; load custom characters to support
       }
      if (UsrInput ==16)
       {
          ExitNow = true;
       }
    }
}


// -------------------------------------------------------------------------
// Function that, given a start time, will calculate an interger number of seconds
// that the progam has been running

long int calcruntime()
{
  now = millis()/1000;//gettimeofday(&now, NULL);
  long int RunTime = (now -startPrgm); //TotalRunTime = ((now.tv_sec - startPrgm.tv_sec));
  return RunTime;
}
// -------------------------------------------------------------------------
void ResetSpindle(){
  SpndlPWM = 15;
  SetPoint = 2000;
  ITerm = 0.0; //kill any residual Integral component that might be left over from a previous run
  lastTime = millis();// reset/establish time mark for initial PID calculation   
  analogWrite(PWMpin, SpndlPWM); //start the PWM output
}
// -------------------------------------------------------------------------

void LoadBarGraphFont(){
  // install  user defined special characters
  lcd.createChar(0, Char0);
  // create a new character
  lcd.createChar(1, Char1);
  // create a new character
  lcd.createChar(2, Char2);
  // create a new character
  lcd.createChar(3, Char3);  
  // create a new character
  lcd.createChar(4,Char4);  
  // create a new character
  lcd.createChar(5, Char5);
}
// -------------------------------------------------------------------------
void ScopeMode(){
  int AsmplCnt =0 ;
  int sensorValue;
  int RowWeight = 64;
  bool KeepWaiting = true;
  unsigned long LoopTime;
  unsigned long GoNow = micros();
  AsmplCnt =0;
  int uSWait = (int)((period/(SampleCnt*10))-900)/8; // returns the number of micro seconds to wait/sample to collect 80 data points given the current spindle speed
//      Serial.println("Mode3");
  while(eventCounter == 0 & KeepWaiting){
    LoopTime = micros()-GoNow;
    if (LoopTime > 300000){
      KeepWaiting = false;
    }
  } 
  while (eventCounter != 0 & KeepWaiting){
   
    LoopTime = micros()-GoNow;
    if (LoopTime > 300000){
      KeepWaiting = false;
    }
  }
  if(KeepWaiting){ // if true, we are receiving interrupts; So use the interrupt method to establish sample frequency
    GoNow = micros();
    unsigned long pause = (period/(SampleCnt))/2;
    while (KeepWaiting){
      LoopTime = micros()-GoNow;
      if (LoopTime > pause){
        KeepWaiting = false;
      }
    }
    //Serial.println("Interrupt Trigger");
    while (AsmplCnt<80){ //while (eventCounter < 2 & AsmplCnt<80){
       sensorValue = 1024-analogRead(sensorPin);
       int Drow = 0; 
       int RowVal=0;
       while (RowVal< sensorValue){
         RowVal = RowVal+RowWeight;
         Drow++;
       }
       IRAnalogBuf[AsmplCnt] = Drow;
       AsmplCnt++;
       GoNow = micros()+uSWait;//150;//
       while (micros()< GoNow);        
     
    }
  }
  else{ // no interrupts detected, try to look for a dip in the analog sense reading to sync/start sample timing
    //Serial.println("Slope Trigger");
    ResetSpindle();
    GoNow = micros();
    uSWait = 90;
    LoopTime = 000;
    while(analogRead(sensorPin)< 800 & LoopTime<60000){ // wait long enough for a motor turning at at least 1,000 rpm to generate a dip in the analog reading
      LoopTime = micros()-GoNow;
    }
    while(analogRead(sensorPin)> 600 & LoopTime<60000){ // wait long enough for a motor turning at at least 1,000 rpm to generate a dip in the analog reading
      LoopTime = micros()-GoNow;
    }
    while (AsmplCnt<80){
       sensorValue = 1024-analogRead(sensorPin);
       int Drow = 0; 
       int RowVal=0;
       while (RowVal< sensorValue){
         RowVal = RowVal+RowWeight;
         Drow++;
       }
       IRAnalogBuf[AsmplCnt] = Drow;
       AsmplCnt++;
       GoNow = micros()+uSWait;//150;//
       while (micros()< GoNow);        
     
    }

  }
//      sprintf (buf, "uSwait:%d   ",  uSWait);
//      Serial.println(buf);
  IRAnalogBuf[AsmplCnt] =-1;
  MapBuf2BitMap();
  LoadTrace();
//      for( int i=0; i< 16; i++){
//      sprintf (buf, "%d,",Char0101[i]);
//      Serial.println(buf);
//      }
  
  
//      for( int i=0; i< 80; i++){
//       sprintf (buf, "%d, ",IRAnalogBuf[i]); 
//       Serial.print(buf);
//       //Serial.print(", ");
//      }
//      Serial.println("");
  //sprintf (buf, "%s%d   ", "Period:", uSWait);
//      sprintf (buf, "%s%d ", "Smpl Cnt:", AsmplCnt);
//      lcd.print(buf);


}// End Scope Mode Function 
// -------------------------------------------------------------------------

void MapBuf2BitMap(){// takes the 80 analog samples just collected & creates 16 5x16 bitmaps [16 x 5= 80]
  int ChrPos = 1;
  unsigned char *BktPtr;
  unsigned char BitMask = 0b10000;
  ResetArrays();// clears out old data such that each of the 16 buckets have 5 bit zereos in each of their 16 rows 
  BktPtr =Rtn5x16BktPtr(ChrPos);
  for (int i = 0; IRAnalogBuf[i] != -1; i++){
//         sprintf(buf,"%d, %d\n\r",IRAnalogBuf[i]-1,BitMask);
//         Serial.print(buf);     
         *(BktPtr+(IRAnalogBuf[i]-1))= *(BktPtr+(IRAnalogBuf[i]-1)) | BitMask;
         //*(BktPtr+2)= *(BktPtr+2) | BitMask; // For testing synsthesize a straight trace
          BitMask = (unsigned int)BitMask>>1;
          if (BitMask == 0){
            BitMask = 0b10000;
            ChrPos++;
            BktPtr =Rtn5x16BktPtr(ChrPos);
//            Serial.print(i);
//            Serial.print(", ");
          }
    }// end "for" loop
}




void ResetArrays(){
  unsigned char *BktPtr;
  for(int DatAry = 1; DatAry < 17; DatAry++){
    BktPtr =Rtn5x16BktPtr(DatAry); 
    for (int i = 0 ; i < 16; i++){
      *(BktPtr+i) = 0b00000;
    }
  }
}


void ResetCstmChars(){
  unsigned char *BktPtr;
  for(int DataAry = 1; DataAry < 9; DataAry++){
    BktPtr =RtnSpclChrPtr(DataAry); 
    for (int i = 0 ; i < 8; i++){
      *(BktPtr+i) = 0b11111;
    }
  }
}

void LoadTrace(){// Draw Oscilloscope Trace (Using Scaled Analog just collected) on 16x2 SanSmart Display 
  int SpclChrCnt = 0;
  int Line1 [16]; // bucket to hold 1st line of 16 bitmaps to be displayed
  int Line2 [16]; // bucket to hold 2nd line of 16 bitmaps to be displayed
  unsigned char *TracePtr;  // create another Pointer label
  unsigned char *TstPtr;// create another Pointer label
  ResetCstmChars();
  // now sequence through the 5x16 bitmaps, & from the custom bitmap collection, assigm them as needed to each of the 16 Display grids/positions in line1 & line2 
  unsigned char *BktPtr;
  for (int ColPos = 1 ; ColPos < 17; ColPos++){
    BktPtr =Rtn5x16BktPtr(ColPos);
     // Now for the current character position, extract a 5x8 bitmap for the 1st line of the display
     // Then determine if it matches one of the exixting custom characters; If not, add it to the custom character collection (max of 8)
     for (int i = 0 ; i < 8; i++){
      TstBtMap[i]= *(BktPtr+i);
     }
     // next check the existing special character bitmaps to see if this TstBtMap has a match
     TstPtr = &TstBtMap[0];
     int MatchingChr = TstBtMp4Match(SpclChrCnt, TstPtr);
     if (MatchingChr != -1){ Line1[ColPos-1] = MatchingChr;
//     Serial.println("Yes");
     }
     else {// no match; it's a new bitmap
       SpclChrCnt++;
//       sprintf (buf, "NO %d",SpclChrCnt);
//      Serial.println(buf);
       if(SpclChrCnt<9){// we haven't yet maxed out the number of custom characters we can define; So add this one to the collection 
         TracePtr = RtnSpclChrPtr(SpclChrCnt);// get a pointer to the next available custom bitmap bucket
         for (int i = 0 ; i < 8; i++){// transfer the contents of this 5x8 bitmap to our newly select custom bitmap bucket 
            *(TracePtr+i)= *(TstPtr+i);
          }
         Line1[ColPos-1] = SpclChrCnt-1; //update the line 1 bucket so that it can later tell the display where to find this custom bitmap 
       }
       else {Line1[ColPos-1] = -1;
 //      Serial.println("OverFlow");
       }
     }
     // Now for the current character position, repeat the above process but use the lower 1/2 of the 5x16 bitmapbucket for the 2nd line of the display
     for (int i = 0 ; i < 8; i++){
      TstBtMap[i]= *(BktPtr+(i+8));
     }
     // now check the existing special character bitmaps to see if this TstBtMap for a match
     TstPtr = &TstBtMap[0];
     MatchingChr = TstBtMp4Match(SpclChrCnt, TstPtr);
     if (MatchingChr != -1) Line2[ColPos-1] = MatchingChr;
     else {
       SpclChrCnt++;
       if(SpclChrCnt<9){
         TracePtr = RtnSpclChrPtr(SpclChrCnt);
         for (int i = 0 ; i < 8; i++){
            *(TracePtr+i)= *(TstPtr+i);
          }
         Line2[ColPos-1] = SpclChrCnt-1; 
       }
       else Line2[ColPos-1] = -1;
     }
  } //End for Loop
  // now load this round's custom charater Set into the 16x2's user defined character ram/registers (max limit of 8) 
  for (int i = 0 ; i < SpclChrCnt & i < 8; i++){
    TracePtr = RtnSpclChrPtr(i+1);
    lcd.createChar(i, TracePtr);
  }
  // now write out the character assignments to the first line
  lcd.setCursor(0,0); //lcd.setCursor(pos,line)
  for (int i = 0 ; i < 16; i++){
    if(Line1[i]!= -1) lcd.write(byte(Line1[i]));
    else lcd.print(" ");//lcd.write(byte(3));
//    sprintf (buf, "%d, ",Line1[i]);
//    Serial.print(buf);
  }
//  Serial.println("");
  // now write out the character assignments to the 2nd line
  lcd.setCursor(0,1); //lcd.setCursor(pos,line)
  for (int i = 0 ; i < 16; i++){
    if(Line2[i]!= -1 ) lcd.write(byte(Line2[i]));
    else lcd.print(" ");//lcd.write(byte(3));
//    sprintf (buf, "%d, ",Line2[i]);
//    Serial.print(buf);
  }
//  Serial.println("");
}//end of LoadTrace() function



int TstBtMp4Match(int SpclChrCnt, unsigned char *TstPtr){
  unsigned char *TracePtr;
  bool IsSame = true;
  int RtnPtr = -1;
  if (SpclChrCnt == 0) return RtnPtr;  
  for (int CurSpclBtMp = 1; CurSpclBtMp < SpclChrCnt+1 & CurSpclBtMp < 9 ; CurSpclBtMp++){
    IsSame = true;
    TracePtr = RtnSpclChrPtr(CurSpclBtMp);
    for (int i = 0 ; i < 8; i++){
        if(*(TstPtr+i)!= *(TracePtr+i)) IsSame = false;
      }
    if (IsSame) return CurSpclBtMp-1;  
  }
  return RtnPtr;
} 


unsigned char* RtnSpclChrPtr(int SpclChrCnt){ // returns a pointer to the designated bitmap array
  unsigned char *TracePtr;
  switch (SpclChrCnt) {
   case 1:
    TracePtr = &Trace1[0];// assosciate the pointer to the Sat1 array
     break;
   case 2:
     TracePtr = &Trace2[0];// assosciate the pointer to the Sat2 array
     break;
   case 3:
     TracePtr = &Trace3[0];// assosciate the pointer to the Sat3 array
     break;
   case 4:
     TracePtr = &Trace4[0];// assosciate the pointer to the Sat4 array
     break;
   case 5:
    TracePtr = &Trace5[0];// assosciate the pointer to the Sat1 array
     break;
   case 6:
     TracePtr = &Trace6[0];// assosciate the pointer to the Sat2 array
     break;
   case 7:
     TracePtr = &Trace7[0];// assosciate the pointer to the Sat3 array
     break;
   case 8:
     TracePtr = &Trace8[0];// assosciate the pointer to the Sat4 array
     break;
 }
 return TracePtr; 
}



unsigned char* Rtn5x16BktPtr(int ChrPos){ // returns a pointer to the designated bitmap array
  unsigned char *BktPtr;
 switch (ChrPos) {
       case 1:
        BktPtr = &Char0101[0];// assosciate the pointer to the Sat1 array
         break;
       case 2:
         BktPtr = &Char0102[0];// assosciate the pointer to the Sat2 array
         break;
       case 3:
         BktPtr = &Char0103[0];// assosciate the pointer to the Sat3 array
         break;
       case 4:
         BktPtr = &Char0104[0];// assosciate the pointer to the Sat4 array
         break;
       case 5:
        BktPtr = &Char0105[0];// assosciate the pointer to the Sat1 array
         break;
       case 6:
         BktPtr = &Char0106[0];// assosciate the pointer to the Sat2 array
         break;
       case 7:
         BktPtr = &Char0107[0];// assosciate the pointer to the Sat3 array
         break;
       case 8:
         BktPtr = &Char0108[0];// assosciate the pointer to the Sat4 array
         break;
       case 9:
        BktPtr = &Char0109[0];// assosciate the pointer to the Sat1 array
         break;
       case 10:
         BktPtr = &Char0110[0];// assosciate the pointer to the Sat2 array
         break;
       case 11:
         BktPtr = &Char0111[0];// assosciate the pointer to the Sat3 array
         break;
       case 12:
         BktPtr = &Char0112[0];// assosciate the pointer to the Sat4 array
         break;
       case 13:
        BktPtr = &Char0113[0];// assosciate the pointer to the Sat1 array
         break;
       case 14:
         BktPtr = &Char0114[0];// assosciate the pointer to the Sat2 array
         break;
       case 15:
         BktPtr = &Char0115[0];// assosciate the pointer to the Sat3 array
         break;
       case 16:
         BktPtr = &Char0116[0];// assosciate the pointer to the Sat4 array
         break;  
     }
 return BktPtr; 
}




