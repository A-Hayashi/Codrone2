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
Metro interval100 = Metro(100);
EEPSTRUCT EEP;
void setup()
{
  CoDrone.begin(115200);  // sets up the connection to the drone using the bluetooth module at 115200bps (bits per second)
  CoDrone.AutoConnect(NearbyDrone);    // finds and connects to a drone that is nearby
  CoDrone.DroneModeChange(Flight);    // Changes the drone so that it is now in flight mode
  mySerial.begin(9600);
  eep_read();
  EEP.k_p  = 0;
  EEP.k_p2 = 0;
  for (int i = 0; i < 8; i++) {
    pinMode(11 + i, INPUT);
  }
}

byte bt[8];
byte bt_state;
byte TrimState = STOP;

int Yaw, Throttle, Pitch, Roll;


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
  else if (TrimState == EEP_READ)
  {
    eep_read();
    CoDrone.Set_TrimAll(EEP.RollTrim, EEP.PitchTrim, EEP.YawTrim, EEP.ThrottleTrim, 0);
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
  } else if (TrimState == EEP_WRITE) {
    eep_write();
    CoDrone.Buzz(7000, 4);
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(5000, 4);
    delay(100);
  } else if (TrimState == HOVER) {
    hovering();
  }else if(TrimState == GAIN_TUNE){
    gain_tune();
  }
}

double error_now;       //今回偏差
double error_before;    //前回偏差
double D_now;           //今回操作量
double D_before;        //前回操作量
double V_ref = 0;       //目標値
double V_now;           //現在値

double error_now2;       //今回偏差
double error_before2;    //前回偏差
double D_now2;           //今回操作量
double D_before2;        //前回操作量
double V_ref2 = 0;       //目標値
double V_now2;           //現在値

void hovering()
{
  if (interval100.check()) {
    V_now = AttitudePITCH;      //AD変換した値を現在値に代入
    error_now = V_ref - V_now;  //目標値と現在値の差から現在偏差を求める
    D_now = D_before + EEP.k_p * (error_now - error_before); 
    D_before = D_now;         //今回PI演算値を前回演算値として保存
    error_before = error_now; //今回偏差を前回偏差として保存
    
    V_now2 = AttitudeROLL;      //AD変換した値を現在値に代入
    error_now2 = V_ref2 - V_now2;  //目標値と現在値の差から現在偏差を求める
    D_now2 = D_before2 + EEP.k_p2 * (error_now2 - error_before2); 
    D_before2 = D_now2;         //今回PI演算値を前回演算値として保存
    error_before2 = error_now2; //今回偏差を前回偏差として保存
  }
  PITCH = D_now;
  ROLL  = D_now2;     // Set the A5 analog pin to control the Roll
  YAW   = Yaw;      // Set the A3 analog pin to control the Yaw
  THROTTLE  = Throttle; // Set the A4 analog pin to control the Throttle
  CoDrone.Control(SEND_INTERVAL); // Send the new flight commands at the SEND_INTERVAL (50ms)
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
  sprintf(s, "%+5d", EEP.YawTrim);
  mySerial.print(s);

  mySerial.print(" THROTTLE: ");
  sprintf(s, "%+5d", EEP.ThrottleTrim);
  mySerial.print(s);

  mySerial.print(" ROLL: ");
  sprintf(s, "%+5d", EEP.RollTrim);
  mySerial.print(s);

  mySerial.print(" PITCH: ");
  sprintf(s, "%+5d", EEP.PitchTrim);
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

  ////////////////////////////////////

  mySerial.print("[PI_PITCH]\n");
  
  mySerial.print(" k_p: ");
  mySerial.print(EEP.k_p,5);

  mySerial.print(" k_p2: ");
  mySerial.print(EEP.k_p2,5);
  mySerial.println();
  
  mySerial.print(" error_now: ");
  mySerial.print(error_now,5);

  mySerial.print(" D_now: ");
  mySerial.print(D_now,5);

  mySerial.print(" V_now: ");
  mySerial.print(V_now,5);
  mySerial.println();
  
  mySerial.print(" error_now2: ");
  mySerial.print(error_now2,5);

  mySerial.print(" D_now2: ");
  mySerial.print(D_now2,5);

  mySerial.print(" V_now2: ");
  mySerial.print(V_now2,5);
}

void clearAndHome()
{
  mySerial.write(27); // ESC
  mySerial.print("[2J"); // clear screen
  mySerial.write(27); // ESC
  mySerial.print("[H"); // cursor to home
}

void eep_write()
{
  byte* p = (byte*)&EEP;
  for (int j = 0; j < sizeof(EEPSTRUCT); j++) {
    EEPROM.write(j, *p);
    p++;
  }
}

void eep_read()
{
  byte* p = (byte*)&EEP;
  for (int j = 0; j < sizeof(EEPSTRUCT); j++) {
    *p = EEPROM.read(j);
    p++;
  }
}

void trimming()
{
  static int RefreshFlag = 0;

  if (Yaw > 50) {
    EEP.YawTrim = EEP.YawTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Yaw < -50) {
    EEP.YawTrim = EEP.YawTrim - INCREMENT;
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    RefreshFlag = 1;
  }
  if (Throttle > 50) {
    EEP.ThrottleTrim = EEP.ThrottleTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Throttle < -50) {
    EEP.ThrottleTrim = EEP.ThrottleTrim - INCREMENT;
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    RefreshFlag = 1;
  }
  if (Pitch > 50) {
    EEP.PitchTrim = EEP.PitchTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Pitch < -50) {
    EEP.PitchTrim = EEP.PitchTrim - INCREMENT;
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    RefreshFlag = 1;
  }
  if (Roll > 50) {
    EEP.RollTrim = EEP.RollTrim + INCREMENT;
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
    RefreshFlag = 1;
  }
  if (Roll < -50) {
    EEP.RollTrim = EEP.RollTrim - INCREMENT;
    CoDrone.Buzz(4000, 4);
    CoDrone.Buzz(2000, 4);
    RefreshFlag = 1;
  }

  if (EEP.YawTrim >= 600) {
    EEP.YawTrim = 600;
  }
  else if (EEP.YawTrim <= -600) {
    EEP.YawTrim = -600;
  }
  if (EEP.RollTrim >= 600) {
    EEP.RollTrim = 600;
  }
  else if (EEP.RollTrim <= -600) {
    EEP.RollTrim = -600;
  }
  if (EEP.PitchTrim >= 600) {
    EEP.PitchTrim = 600;
  }
  else if (EEP.PitchTrim <= -600) {
    EEP.PitchTrim = -600;
  }
  if (EEP.ThrottleTrim >= 600) {
    EEP.ThrottleTrim = 600;
  }
  else if (EEP.ThrottleTrim <= -600) {
    EEP.ThrottleTrim = -600;
  }
  if (RefreshFlag == 1) {
    CoDrone.Set_TrimAll(EEP.RollTrim, EEP.PitchTrim, EEP.YawTrim, EEP.ThrottleTrim, 0);
    delay(150);
    RefreshFlag = 0;
  }
}


void gain_tune()
{
  double inc;
  inc = (double)Throttle / 100;
  
  if (Pitch > 50) {
    EEP.k_p += inc;
  }
  if (Pitch < -50) {
    EEP.k_p2 += inc;
  }
}

void joystick_control() {
  YAW       = Yaw;      // Set the A3 analog pin to control the Yaw
  THROTTLE  = Throttle; // Set the A4 analog pin to control the Throttle
  ROLL      = Roll;     // Set the A5 analog pin to control the Roll
  PITCH     = Pitch;    // Set the A6 analog pin to control the Pitch
  CoDrone.Control(SEND_INTERVAL); // Send the new flight commands at the SEND_INTERVAL (50ms)
}



void trans_bt_state() {

  for (int i = 0; i < 8; i++) {
    bt[i] = digitalRead(11 + i);
  }
  bt_state = 0;
  for (int i = 0; i < 8; i++) {
    bt_state |= (bt[i] << (7 - i));
  }
  switch (bt_state) {
    case 0b00000000:
      TrimState = HOVER; break;
    case 0b10000000:
      TrimState = STOP; break;
    case 0b01000000:
      TrimState = EEP_WRITE; break;
    case 0b00000010:
      TrimState = EEP_READ; break;
    case 0b00000001:
      TrimState = GAIN_TUNE; break;
    case 0b10000001:
      TrimState = TRIM; break;
    default:
      TrimState = CONTROL; break;
  }
}




