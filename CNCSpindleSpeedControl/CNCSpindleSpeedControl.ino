/*
This Program when loaded on an Arduino/Leonardo is a Tach based speed controller.
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

Note: The LCDKeypadR1 Library, contained in this file set, 
is a derivative of the files found here:

http://sainsmart.com/zen/documents/20-011-901/keypad_lcd.zip

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
int GPIOpin = 1; // Use pin "0" [Pi IO header pin 11] to simulate a Clk pulse s$
//int GPIOpinSlow = 4; //Under Speed LED
//int GPIOpinOK = 5; //Speed Ok LED
//int GPIOpinFast = 6; //Over Speed LED

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
  //Serial.begin(9600);  // enable when/if diagnostic is needed
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
      SpndlPWM = 15;
      SetPoint = 2000;
      ITerm = 0.0; //kill any residual Integral component that might be left over from a previous run
      lastTime = millis();// reset/establish time mark for initial PID calculation   
      analogWrite(PWMpin, SpndlPWM); //start the PWM output
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
   if (Mode == 3)// Set Back Light Color Mode
    {
      //lcdPrintf (lcdHandle,"Set Display Clr ");
      //sprintf (buf, "%s%s ", "Spd Err:", calcrpm);
     lcd.print("Set Display Clr ");
    }
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
// pinMode (GPIOpin, OUTPUT) ; //configure pin 1 as a simple output pin
// digitalWrite (GPIOpin, LOW); //set/force pin 1 output to ground [low]
// digitalWrite (GPIOpinSlow, LOW); //set/force pin 4 output High[Under Speed LED off]
// digitalWrite (GPIOpinOK, LOW); //set/force pin 5 output  High[Speed Ok LED off]
// digitalWrite (GPIOpinFast, LOW); //set/force pin 6 output  High[Over Speed LED off]


 //Serial1.print("All Done!\n");
 //setBacklightColour (0);
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
        NuInput = false;
        if (UsrInput == 4)
         {
           Mode = Mode +1;
           if (Mode == 8)
            {
              //Mode = 3;
              Mode = 4;
            }
         }
        else
         {
           Mode = Mode- 1;
           if (Mode == 3)//if (Mode == 2)
            {
              Mode = 7;
            }

         }
        //setBacklightColour (BackLight);
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














