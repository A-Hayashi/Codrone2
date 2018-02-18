#if defined (__AVR_ATmega32__)
//#error "AVR_ATmega32"
#endif
/*****************************************************************
  Control - Analog Joystick Control
  This is a basic remote control program for the drone in flight mode
*******************************************************************/
#include <CoDrone.h> // The codrone library that holds all the background files for thiss
#include "SoftwareSerial.h"
#include "Metro.h"
#include "EEPROM.h"

SoftwareSerial mySerial(27, 28); /* Rx,Tx */
Metro interval250 = Metro(250);

void setup()
{
  CoDrone.begin(115200);  // sets up the connection to the drone using the bluetooth module at 115200bps (bits per second)
  CoDrone.AutoConnect(NearbyDrone);    // finds and connects to a drone that is nearby
  CoDrone.DroneModeChange(Flight);    // Changes the drone so that it is now in flight mode
  mySerial.begin(9600);
  trim_read();
}

#define STOP        0
#define TRIM_RESET  1
#define CONTROL     2
#define TRIM        3
#define TRIM_STORE  4
const String TRIM_STATE[] = {
  "STOP",
  "TRIM_RESET",
  "CONTROL",
  "TRIM",
  "TRIM_STORE"
};
byte TrimState = STOP;

//Set your increment size here
int Increment = 20;
int Yaw, Throttle, Pitch, Roll;
int RefreshFlag = 0;

typedef struct _TRIMSTRUCT {
  int YawTrim;
  int ThrottleTrim;
  int PitchTrim;
  int RollTrim;
} TRIMSTRUCT;
TRIMSTRUCT Trim;

void loop()
{
  trimming();
  if (interval250.check()) {
    disp_serial();
  }
}

byte bt1, bt2, bt4, bt8;
void trimming()
{
  bt1 = digitalRead(11); // ■ □ □ □ □ □ □ Initialize the bottom left IR sensor
  bt2 = digitalRead(12); // □ ■ □ □ □ □ □ Initialize the bottom left IR sensor
  bt4 = digitalRead(14); // □ □ □ ■ □ □ □ Initialize the bottom center IR sensor
  bt8 = digitalRead(18); // □ □ □ □ □ □ ■ Initialize the bottom right IR sensor

  Yaw = -1 * CoDrone.AnalogScaleChange(analogRead(A3)); // YAW (turn left or right), port A3, reversed
  Throttle = CoDrone.AnalogScaleChange(analogRead(A4)); // THROTTLE (height), port A4, not reversed
  Roll = -1 * CoDrone.AnalogScaleChange(analogRead(A5)); // ROLL (tilt left or right), port A5, reversed
  Pitch = CoDrone.AnalogScaleChange(analogRead(A6)); // PITCH (tilt front or back), port A6, not reversed

  //■ □ □ □ □ □ □ □
  if (bt1 && !bt4 && !bt8) { // If the left sensor is triggered
    TrimState = STOP;
    //■ □ □ □ □ □ □ ■
  } else if (bt1 && !bt4 && bt8) { // If both the left and right sensors are triggered
    TrimState = TRIM_RESET;
    //□ ■ □ □ □ □ □ □
  } else if (bt2) {
    TrimState = TRIM_STORE;
    //□ □ □ □ □ □ □ □
  } else if (!bt8) {  //If the right sensor is NOT triggered
    TrimState = CONTROL;
    //□ □ □ □ □ □ □ ■
  } else if (bt8) {// If the right sensor IS triggered
    TrimState = TRIM;
  }

  //**************************** STOP ******************************//
  if (TrimState == STOP)
  {
    CoDrone.FlightEvent(Stop); // Stop the CoDrone
    // CoDrone.FlightEvent(Landing); // Land the CoDrone. You can choose either of these.
  }
  //*************************** Trim Reset ***********************//
  else if (TrimState == TRIM_RESET)
  {
    trim_read();
    CoDrone.Set_TrimAll(Trim.RollTrim, Trim.PitchTrim, Trim.YawTrim, Trim.ThrottleTrim, 0);
    CoDrone.Buzz(7000, 4);
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(5000, 4);
    delay(100);
  }
  //*************************** Control & Trim ***********************//
  else if (TrimState == CONTROL)
  {
    digitalWrite(16, HIGH);
    YAW       = Yaw;      // Set the A3 analog pin to control the Yaw
    THROTTLE  = Throttle; // Set the A4 analog pin to control the Throttle
    ROLL      = Roll;     // Set the A5 analog pin to control the Roll
    PITCH     = Pitch;    // Set the A6 analog pin to control the Pitch
    CoDrone.Control(SEND_INTERVAL); // Send the new flight commands at the SEND_INTERVAL (50ms)
  }
  else if (TrimState == TRIM)
  {
    digitalWrite(16, LOW);
    if (Yaw > 50) {
      Trim.YawTrim = Trim.YawTrim + Increment;
      CoDrone.Buzz(2000, 4);
      CoDrone.Buzz(4000, 4);
      RefreshFlag = 1;
    }
    if (Yaw < -50) {
      Trim.YawTrim = Trim.YawTrim - Increment;
      CoDrone.Buzz(4000, 4);
      CoDrone.Buzz(2000, 4);
      RefreshFlag = 1;
    }
    if (Throttle > 50) {
      Trim.ThrottleTrim = Trim.ThrottleTrim + Increment;
      CoDrone.Buzz(2000, 4);
      CoDrone.Buzz(4000, 4);
      RefreshFlag = 1;
    }
    if (Throttle < -50) {
      Trim.ThrottleTrim = Trim.ThrottleTrim - Increment;
      CoDrone.Buzz(4000, 4);
      CoDrone.Buzz(2000, 4);
      RefreshFlag = 1;
    }
    if (Pitch > 50) {
      Trim.PitchTrim = Trim.PitchTrim + Increment;
      CoDrone.Buzz(2000, 4);
      CoDrone.Buzz(4000, 4);
      RefreshFlag = 1;
    }
    if (Pitch < -50) {
      Trim.PitchTrim = Trim.PitchTrim - Increment;
      CoDrone.Buzz(4000, 4);
      CoDrone.Buzz(2000, 4);
      RefreshFlag = 1;
    }
    if (Roll > 50) {
      Trim.RollTrim = Trim.RollTrim + Increment;
      CoDrone.Buzz(2000, 4);
      CoDrone.Buzz(4000, 4);
      RefreshFlag = 1;
    }
    if (Roll < -50) {
      Trim.RollTrim = Trim.RollTrim - Increment;
      CoDrone.Buzz(4000, 4);
      CoDrone.Buzz(2000, 4);
      RefreshFlag = 1;
    }

    if (Trim.YawTrim >= 600) {
      Trim.YawTrim = 600;
    }
    else if (Trim.YawTrim <= -600) {
      Trim.YawTrim = -600;
    }
    if (Trim.RollTrim >= 600) {
      Trim.RollTrim = 600;
    }
    else if (Trim.RollTrim <= -600) {
      Trim.RollTrim = -600;
    }
    if (Trim.PitchTrim >= 600) {
      Trim.PitchTrim = 600;
    }
    else if (Trim.PitchTrim <= -600) {
      Trim.PitchTrim = -600;
    }
    if (Trim.ThrottleTrim >= 600) {
      Trim.ThrottleTrim = 600;
    }
    else if (Trim.ThrottleTrim <= -600) {
      Trim.ThrottleTrim = -600;
    }
    if (RefreshFlag == 1) {
      CoDrone.Set_TrimAll(Trim.RollTrim, Trim.PitchTrim, Trim.YawTrim, Trim.ThrottleTrim, 0);
      delay(150);
      RefreshFlag = 0;
    }
  } else if (TrimState == TRIM_STORE) {
    trim_store();
    CoDrone.Buzz(7000, 4);
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(5000, 4);
    delay(100);
  }
}



void disp_serial()
{
  char s[32];
  clearAndHome();

  mySerial.print("bt1: ");
  mySerial.print(bt1);
  mySerial.print(" bt2: ");
  mySerial.print(bt2);
  mySerial.print(" bt4: ");
  mySerial.print(bt4);
  mySerial.print(" bt8: ");
  mySerial.print(bt8);

  mySerial.print(" TrimState: ");
  mySerial.print(TRIM_STATE[TrimState]);
  mySerial.println();

  /////////////////////////////////////////

  mySerial.print("[TRIM]\n");
  mySerial.print(" YAW: ");
  sprintf(s, "%+5d", Trim.YawTrim);
  mySerial.print(s);

  mySerial.print(" THROTTLE: ");
  sprintf(s, "%+5d", Trim.ThrottleTrim);
  mySerial.print(s);

  mySerial.print(" ROLL: ");
  sprintf(s, "%+5d", Trim.RollTrim);
  mySerial.print(s);

  mySerial.print(" PITCH: ");
  sprintf(s, "%+5d", Trim.PitchTrim);
  mySerial.print(s);

  mySerial.println();
  //////////////////////////////////////////
  mySerial.print("[JOYSTICK]\n");
  mySerial.print(" YAW: ");
  sprintf(s, "%+5d", Yaw);
  mySerial.print(s);

  mySerial.print(" THROTTLE: ");
  sprintf(s, "%+5d", Throttle);
  mySerial.print(s);

  mySerial.print(" ROLL: ");
  sprintf(s, "%+5d", Roll);
  mySerial.print(s);

  mySerial.print(" PITCH: ");
  sprintf(s, "%+5d", Pitch);
  mySerial.print(s);

  mySerial.println();
}


void clearAndHome()
{
  mySerial.write(27); // ESC
  mySerial.print("[2J"); // clear screen
  mySerial.write(27); // ESC
  mySerial.print("[H"); // cursor to home
}


void trim_store()
{
  byte* p = (byte*)&Trim;
  for (int j = 0; j < sizeof(TRIMSTRUCT); j++) {
    EEPROM.write(j, *p);
    p++;
  }
}

void trim_read()
{
  byte* p = (byte*)&Trim;
  for (int j = 0; j < sizeof(TRIMSTRUCT); j++) {
    *p = EEPROM.read(j);
    p++;
  }
}

