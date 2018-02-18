#if defined (__AVR_ATmega32__)
//#error "AVR_ATmega32"
#endif
/*****************************************************************
  Control - Analog Joystick Control
  This is a basic remote control program for the drone in flight mode
*******************************************************************/
#include "CoDrone.h" // The codrone library that holds all the background files for thiss
#include "SoftwareSerial.h"
#include "Metro.h"
#include "EEPROM.h"
#include "Controller_Flight_3.h"

SoftwareSerial mySerial(27, 28); /* Rx,Tx */
Metro interval250 = Metro(250);

void setup()
{
  CoDrone.begin(115200);  // sets up the connection to the drone using the bluetooth module at 115200bps (bits per second)
  CoDrone.AutoConnect(NearbyDrone);    // finds and connects to a drone that is nearby
  CoDrone.DroneModeChange(Flight);    // Changes the drone so that it is now in flight mode
  mySerial.begin(9600);
  trim_read();

  for (int i = 0; i < 8; i++) {
    pinMode(11 + i, INPUT);
  }
}

byte bt[8];
byte bt_state;
byte TrimState = STOP;

int Yaw, Throttle, Pitch, Roll;
TRIMSTRUCT Trim;

void loop()
{
  CoDrone.Request_DroneAttitude();
  CoDrone.Receive();

  control_drone();

  if (interval250.check()) {
    disp_serial();
  }
}



void control_drone()
{
  trans_bt_state();
  Yaw = -1 * CoDrone.AnalogScaleChange(analogRead(A3)); // YAW (turn left or right), port A3, reversed
  Throttle = CoDrone.AnalogScaleChange(analogRead(A4)); // THROTTLE (height), port A4, not reversed
  Roll = -1 * CoDrone.AnalogScaleChange(analogRead(A5)); // ROLL (tilt left or right), port A5, reversed
  Pitch = CoDrone.AnalogScaleChange(analogRead(A6)); // PITCH (tilt front or back), port A6, not reversed

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
  else if (TrimState == CONTROL) {
    joystick_control();
  }
  else if (TrimState == TRIM) {
    trimming();
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

  mySerial.print("bt_state: ");
  for (int i = 0; i < 8; i++) {
    mySerial.print(bt[i]);
    mySerial.print(" ");
  }

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

  ////////////////////////////////////////
  mySerial.print("[ATTITUDE]\n");
  mySerial.print(" YAW: ");
  sprintf(s, "%+5d", AttitudeYAW);
  mySerial.print(s);

  mySerial.print(" ROLL: ");
  sprintf(s, "%+5d", AttitudeROLL);
  mySerial.print(s);

  mySerial.print(" PITCH: ");
  sprintf(s, "%+5d", AttitudePITCH);
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


void trimming()
{
  static int RefreshFlag = 0;
  
  if (Yaw > 50) {
    Trim.YawTrim = Trim.YawTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Yaw < -50) {
    Trim.YawTrim = Trim.YawTrim - INCREMENT;
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    RefreshFlag = 1;
  }
  if (Throttle > 50) {
    Trim.ThrottleTrim = Trim.ThrottleTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Throttle < -50) {
    Trim.ThrottleTrim = Trim.ThrottleTrim - INCREMENT;
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    RefreshFlag = 1;
  }
  if (Pitch > 50) {
    Trim.PitchTrim = Trim.PitchTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Pitch < -50) {
    Trim.PitchTrim = Trim.PitchTrim - INCREMENT;
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    RefreshFlag = 1;
  }
  if (Roll > 50) {
    Trim.RollTrim = Trim.RollTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Roll < -50) {
    Trim.RollTrim = Trim.RollTrim - INCREMENT;
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
}


void joystick_control() {
  YAW       = Yaw;      // Set the A3 analog pin to control the Yaw
  THROTTLE  = Throttle; // Set the A4 analog pin to control the Throttle
  ROLL      = Roll;     // Set the A5 analog pin to control the Roll
  PITCH     = Pitch;    // Set the A6 analog pin to control the Pitch
  CoDrone.Control(SEND_INTERVAL); // Send the new flight commands at the SEND_INTERVAL (50ms)
}



void trans_bt_state(){
  
  for (int i = 0; i < 8; i++) {
    bt[i] = digitalRead(11 + i);
  }
  bt_state = 0;
  for (int i = 0; i < 8; i++) {
    bt_state |= (bt[i] << (7 - i));
  }
    switch (bt_state) {
    case 0b00000000:
      TrimState = CONTROL; break;
    case 0b10000000:
      TrimState = STOP; break;
    case 0b01000000:
      TrimState = TRIM_STORE; break;
    case 0b00000010:
      TrimState = HOVER ; break;
    case 0b00000001:
      TrimState = TRIM; break;
    case 0b10000001:
      TrimState = TRIM_RESET; break;
    default:
      TrimState = CONTROL; break;
  }
}
