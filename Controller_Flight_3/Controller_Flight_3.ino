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
EEPSTRUCT EEP;
int Yaw, Throttle, Pitch, Roll;
byte IR_Sensor;

void setup()
{
  CoDrone.begin(115200);  // sets up the connection to the drone using the bluetooth module at 115200bps (bits per second)
  CoDrone.AutoConnect(NearbyDrone);    // finds and connects to a drone that is nearby
  CoDrone.DroneModeChange(Flight);    // Changes the drone so that it is now in flight mode
  mySerial.begin(9600);
  eep_read();
  for (int i = 0; i < 8; i++) {
    pinMode(11 + i, INPUT);
  }
}

void get_analog_stick(void)
{
  Yaw = -1 * CoDrone.AnalogScaleChange(analogRead(A3)); // YAW (turn left or right), port A3, reversed
  Throttle = CoDrone.AnalogScaleChange(analogRead(A4)); // THROTTLE (height), port A4, not reversed
  Roll = -1 * CoDrone.AnalogScaleChange(analogRead(A5)); // ROLL (tilt left or right), port A5, reversed
  Pitch = CoDrone.AnalogScaleChange(analogRead(A6)); // PITCH (tilt front or back), port A6, not reversed
}


void get_ir_sensor()
{
  byte ir[8];

  for (int i = 0; i < 8; i++) {
    ir[i] = digitalRead(11 + i);
  }
  IR_Sensor = 0;
  for (int i = 0; i < 8; i++) {
    IR_Sensor |= (ir[i] << (7 - i));
  }
}

void loop()
{
  receive_pcdata();
  get_dronedata();
  get_ir_sensor();
  get_analog_stick();
  state_machine();
  send_pcdata();
}

void get_dronedata()
{
  Metro interval100 = Metro(100);
  Metro interval5000 = Metro(5000);

  if (interval100.check()) {
    CoDrone.Request_DroneAttitude();
    CoDrone.Request_ImuRawAndAngle();
    CoDrone.Request_Pressure();
  } else if (interval5000.check()) {
    CoDrone.Request_DroneGyroBias();
    CoDrone.Request_TrimAll();
    CoDrone.Request_Temperature();
  } else {
    CoDrone.Receive();
  }
}

void receive_pcdata()
{
}


void send_pcdata()
{
  Metro interval50 = Metro(50);
  Metro interval2500 = Metro(2500);

  if (interval50.check()) {
    Send_Attitude();
    Send_ImuRawAndAngl();
    Send_Pressure();
    Send_IrMessage();
  } else if (interval2500.check()) {
    Send_GyroBias();
    Send_TrimAll();
    Send_Temperature();
  }
}


void Send_Attitude()
{
  byte data[12];
  byte len = 6;
  data[0] = LowB(CoDrone.attitudeRoll);
  data[1] = HighB(CoDrone.attitudeRoll);
  data[2] = LowB(CoDrone.attitudePitch);
  data[3] = HighB(CoDrone.attitudePitch);
  data[4] = LowB(CoDrone.attitudeYaw);
  data[5] = HighB(CoDrone.attitudeYaw);
  Send_Processing(tType_Attitude, data, len);
}


void Send_GyroBias()
{
  byte data[12];
  byte len = 6;
  data[0] = CoDrone.droneGyroBias[0]; //Roll
  data[1] = CoDrone.droneGyroBias[1];
  data[2] = CoDrone.droneGyroBias[2]; //Pitch
  data[3] = CoDrone.droneGyroBias[3];
  data[4] = CoDrone.droneGyroBias[4]; //Yaw
  data[5] = CoDrone.droneGyroBias[5];
  Send_Processing(tType_GyroBias, data, len);
}


void Send_TrimAll()
{
  byte data[12];
  byte len = 10;
  data[0] = CoDrone.droneTrimAll[0];  //Roll
  data[1] = CoDrone.droneTrimAll[1];
  data[2] = CoDrone.droneTrimAll[2];  //Pitch
  data[3] = CoDrone.droneTrimAll[3];
  data[4] = CoDrone.droneTrimAll[4];  //Yaw
  data[5] = CoDrone.droneTrimAll[5];
  data[6] = CoDrone.droneTrimAll[6];  //Throttle
  data[7] = CoDrone.droneTrimAll[7];
  data[8] = CoDrone.droneTrimAll[8];  //Wheel
  data[9] = CoDrone.droneTrimAll[9];
  Send_Processing(tType_TrimAll, data, len);
}


void Send_IrMessage()
{
  byte data[12];
  byte len = 5;
  data[0] = CoDrone.droneIrMassage[0];  //Direction
  data[1] = CoDrone.droneIrMassage[1];
  data[2] = CoDrone.droneIrMassage[2];
  data[3] = CoDrone.droneIrMassage[3];
  data[4] = CoDrone.droneIrMassage[4];
  Send_Processing(tType_IrMessage, data, len);
}


void Send_ImuRawAndAngl()
{
  byte data[18];
  byte len = 18;
  data[0] = LowB(CoDrone.droneImuRawAndAngle[0]);   //AccX
  data[1] = HighB(CoDrone.droneImuRawAndAngle[0]);
  data[2] = LowB(CoDrone.droneImuRawAndAngle[1]);   //AccY
  data[3] = HighB(CoDrone.droneImuRawAndAngle[1]);
  data[4] = LowB(CoDrone.droneImuRawAndAngle[2]);   //AccZ
  data[5] = HighB(CoDrone.droneImuRawAndAngle[2]);
  data[6] = LowB(CoDrone.droneImuRawAndAngle[3]);   //GyroRoll
  data[7] = HighB(CoDrone.droneImuRawAndAngle[3]);
  data[8] = LowB(CoDrone.droneImuRawAndAngle[4]);   //GyroPitch
  data[9] = HighB(CoDrone.droneImuRawAndAngle[4]);
  data[10] = LowB(CoDrone.droneImuRawAndAngle[5]);  //GyroYaw
  data[11] = HighB(CoDrone.droneImuRawAndAngle[5]);
  data[12] = LowB(CoDrone.droneImuRawAndAngle[6]);  //AngleRoll
  data[13] = HighB(CoDrone.droneImuRawAndAngle[6]);
  data[14] = LowB(CoDrone.droneImuRawAndAngle[7]);  //AnglePitch
  data[15] = HighB(CoDrone.droneImuRawAndAngle[7]);
  data[16] = LowB(CoDrone.droneImuRawAndAngle[8]);  //AngleRoll
  data[17] = HighB(CoDrone.droneImuRawAndAngle[8]);

  Send_Processing(tType_ImuRawAndAngl, data, len);
}

void Send_Pressure()
{
  byte data[16];
  byte len = 16;

  for (int i = 0; i < len; i++) {
    data[i] = CoDrone.dronePressure[i];
  }
  Send_Processing(tType_Pressure, data, len);
}

void Send_Temperature()
{
  byte data[8];
  byte len = 8;

  for (int i = 0; i < len; i++) {
    data[0] = CoDrone.droneTemperature[i];
  }
  Send_Processing(tType_Temperature, data, len);
}



byte Command;
byte trans_state(byte state)
{
  static byte next_state;
  next_state = Command;
  return next_state;
}

void state_machine()
{
  static byte old_state = cmdType_Control;
  byte state = cmdType_Control;
  boolean state_change = false;

  state = trans_state(state);

  if (old_state != state) {
    state_change = true;
  } else {
    state_change = false;
  }

  switch (state) {
    case cmdType_Control:
      state_Control(state_change);
      break;
    case cmdType_EEP_Write:
      state_EEP_Write(state_change);
      break;
    case cmdType_EEP_Read:
      state_EEP_Read(state_change);
      break;
    case cmdType_TrimTune:
      state_TrimTune(state_change);
      break;
    case cmdType_TrimSet:
      state_TrimSet(state_change);
      break;
    case cmdType_Stop:
      state_Stop(state_change);
      break;
    case cmdType_Hover:
      state_Hover(state_change);
      break;
    case cmdType_GainTune:
      state_GainTune(state_change);
      break;
    default:
      state_Control(state_change);
      break;
  }
  old_state = state;
}

void state_Control(boolean state_change)
{
  YAW       = Yaw;      // Set the A3 analog pin to control the Yaw
  THROTTLE  = Throttle; // Set the A4 analog pin to control the Throttle
  ROLL      = Roll;     // Set the A5 analog pin to control the Roll
  PITCH     = Pitch;    // Set the A6 analog pin to control the Pitch
  CoDrone.Control(SEND_INTERVAL); // Send the new flight commands at the SEND_INTERVAL (50ms)
}

void state_EEP_Write(boolean state_change)
{
  if (state_change) {
    eep_write();
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
  }
}

void state_EEP_Read(boolean state_change)
{
  if (state_change) {
    eep_read();
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
  }
}

void state_TrimTune(boolean state_change)
{
  EEP.YawTrim = 0;
}


void state_TrimSet(boolean state_change)
{
  if (state_change) {
    CoDrone.Set_TrimAll(EEP.RollTrim, EEP.PitchTrim, EEP.YawTrim, EEP.ThrottleTrim, 0);
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
  }
}


void state_Stop(boolean state_change)
{
  CoDrone.FlightEvent(Stop);
}

void state_Hover(boolean state_change)
{

}

void state_GainTune(boolean state_change)
{

}

void Send_Processing(byte _cType, byte _data[], byte _length)
{
  byte _packet[30];

  //START CODE
  _packet[0] = START1;
  _packet[1] = START2;

  //CONTROL TYPE
  _packet[2] = _cType;

  //LENGTH
  _packet[3] = _length;

  //DATA
  for (int i = 0; i < _length + 3 ; i++)
  {
    _packet[i + 4] = _data[i];
  }
  mySerial.write(_packet, _length + 4);
}


void eep_write()
{
  byte* p = (byte*)&EEP;
  for (int j = 0; j < sizeof(EEPSTRUCT); j++) {
    EEPROM.write(j + EEP_START_ADRESS, *p);
    p++;
  }
}

void eep_read()
{
  byte* p = (byte*)&EEP;
  for (int j = 0; j < sizeof(EEPSTRUCT); j++) {
    *p = EEPROM.read(j + EEP_START_ADRESS);
    p++;
  }
}

void clearAndHome()
{
  mySerial.write(27); // ESC
  mySerial.print("[2J"); // clear screen
  mySerial.write(27); // ESC
  mySerial.print("[H"); // cursor to home
}


