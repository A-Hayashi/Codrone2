
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
SoftwareSerial mySerial(27, 28); /* Rx,Tx */
Metro interval250 = Metro(250);
void setup()
{
  CoDrone.begin(115200);  // sets up the connection to the drone using the bluetooth module at 115200bps (bits per second)

  CoDrone.AutoConnect(NearbyDrone);    // finds and connects to a drone that is nearby

  CoDrone.DroneModeChange(Flight);    // Changes the drone so that it is now in flight mode

  mySerial.begin(9600);
  pinMode(11, OUTPUT);
}

void loop()
{
  if (interval250.check()) {
    carib();
    disp_serial();
  }
}


#define TRIM_YAW        0
#define TRIM_THROTTLE   1
#define TRIM_PITCH      2
#define TRIM_ROLL       3
#define TRIM_APPLY      4
#define TRIM_CONTROL    5
#define TRIM_STOP       6

String TRIM_STATE[] = {
  "TRIM_YAW",
  "TRIM_THROTTLE",
  "TRIM_PITCH",
  "TRIM_ROLL",
  "TRIM_APPLY",
  "TRIM_CONTROL",
  "TRIM_STOP"
};

int YawTrim, ThrottleTrim, PitchTrim, RollTrim;
byte trim_state = TRIM_CONTROL;
byte cmd;

void carib(void)
{
  if (mySerial.available() > 0) {
    cmd = mySerial.read();
  } else {
    cmd = '\0';
  }


  switch (cmd) {
    case 'y':
      trim_state = TRIM_YAW;
      break;
    case 't':
      trim_state = TRIM_THROTTLE;
      break;
    case 'p':
      trim_state = TRIM_PITCH;
      break;
    case 'r':
      trim_state = TRIM_ROLL;
      break;
    case 'a':
      trim_state = TRIM_APPLY;
      break;
    case 'c':
      trim_state = TRIM_CONTROL;
      break;
    case 's':
      trim_state = TRIM_STOP;
      break;

  }

  switch (trim_state) {
    case TRIM_YAW:
      trim_carib(cmd, &YawTrim);
      break;
    case TRIM_THROTTLE:
      trim_carib(cmd, &ThrottleTrim);
      break;
    case TRIM_PITCH:
      trim_carib(cmd, &PitchTrim);
      break;
    case TRIM_ROLL:
      trim_carib(cmd, &RollTrim);
      break;
    case TRIM_APPLY:
      CoDrone.Set_TrimAll(RollTrim, PitchTrim, YawTrim, ThrottleTrim, 0);
      delay(150);
      trim_state = TRIM_CONTROL;
      break;
    case TRIM_CONTROL:
      joystick_control();
      break;
    case TRIM_STOP:
      drone_stop();
      break;
    default:
      break;
  }
}


void clip(int lower, int *val, int upper)
{
  if (*val > upper) {
    *val = upper;
  } else if (*val < lower) {
    *val = lower;
  }
}

void trim_carib(byte cmd, int *val)
{
  if (cmd == 'u') {
    *val++;
  } else if (cmd == 'd') {
    *val--;
  }
  //  clip(-600, val, 600);
}

void joystick_control(void)
{
  YAW = -1 * CoDrone.AnalogScaleChange(analogRead(A3)); // Set the A3 analog pin to control the Yaw
  THROTTLE = CoDrone.AnalogScaleChange(analogRead(A4)); // Set the A4 analog pin to control the Throttle
  ROLL = -1 * CoDrone.AnalogScaleChange(analogRead(A5)); // Set the A5 analog pin to control the Roll
  PITCH = CoDrone.AnalogScaleChange(analogRead(A6)); // Set the A6 analog pin to control the Pitch
  //  CoDrone.Control(SEND_INTERVAL); // Send the new flight commands at the SEND_INTERVAL (50ms)
}

void drone_stop(void)
{
  CoDrone.FlightEvent(Stop);
}

void disp_serial(void)
{
  char s[32];
  clearAndHome();

  mySerial.print(" cmd: 0x");
  mySerial.print(cmd, HEX);
  mySerial.print(" trim_state: ");
  mySerial.print(TRIM_STATE[trim_state]);
  mySerial.println();

  /////////////////////////////////////////

  mySerial.print("[TRIM]\n");
  mySerial.print(" YAW: ");
  sprintf(s, "%+5d", YawTrim);
  mySerial.print(s);

  mySerial.print(" THROTTLE: ");
  sprintf(s, "%+5d", ThrottleTrim);
  mySerial.print(s);

  mySerial.print(" ROLL: ");
  sprintf(s, "%+5d", RollTrim);
  mySerial.print(s);

  mySerial.print(" PITCH: ");
  sprintf(s, "%+5d", PitchTrim);
  mySerial.print(s);

  mySerial.println();
  //////////////////////////////////////////
  mySerial.print("[JOYSTICK]\n");
  mySerial.print(" YAW: ");
  sprintf(s, "%+5d", YAW);
  mySerial.print(s);

  mySerial.print(" THROTTLE: ");
  sprintf(s, "%+5d", THROTTLE);
  mySerial.print(s);

  mySerial.print(" ROLL: ");
  sprintf(s, "%+5d", ROLL);
  mySerial.print(s);

  mySerial.print(" PITCH: ");
  sprintf(s, "%+5d", PITCH);
  mySerial.print(s);

  mySerial.println();
}


void clearAndHome(void)
{
  mySerial.write(27); // ESC
  mySerial.print("[2J"); // clear screen
  mySerial.write(27); // ESC
  mySerial.print("[H"); // cursor to home
}



