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
byte ControlState;
byte CmdCtrlState;

char buff[10];

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


Metro interval100 = Metro(100);
Metro interval2500 = Metro(2500);
void get_dronedata()
{
  if (interval100.check()) {
    CoDrone.Request_DroneAttitude();
    CoDrone.Request_Range();
    //    CoDrone.Request_ImuRawAndAngle();
    //    CoDrone.Request_Pressure();
  }
  if (interval2500.check()) {
    //    CoDrone.Request_DroneGyroBias();
    //    CoDrone.Request_TrimAll();
    //    CoDrone.Request_Temperature();
  }
  CoDrone.Receive();

}


Metro interval50 = Metro(50);
Metro interval1000 = Metro(1000);
void send_pcdata()
{
  if (interval50.check()) {
    Send_Attitude();
    Send_Range();
    //    Send_ImuRawAndAngl();
    //    Send_Pressure();
    //    Send_IrMessage();
    Send_AnalogStick();
    Send_ControlState();
  }
  if (interval1000.check()) {

  }
}

void Send_ControlState()
{
  byte data[12];
  byte len = 1;
  data[0] = ControlState;

  Send_Processing(tType_ControlState, data, len);
}

void Send_Attitude()
{
  byte data[12];
  byte len = 7;
  data[0] = LowB(CoDrone.droneAttitude[0]);   //Roll
  data[1] = HighB(CoDrone.droneAttitude[0]);
  data[2] = LowB(CoDrone.droneAttitude[1]);   //Pitch
  data[3] = HighB(CoDrone.droneAttitude[1]);
  data[4] = LowB(CoDrone.droneAttitude[2]);   //Yaw
  data[5] = HighB(CoDrone.droneAttitude[2]);
  data[6] = CoDrone.Alive.Attitude;

  //  mySerial.print(millis()); mySerial.print(" ");
  //  mySerial.print(CoDrone.droneAttitude[0]); mySerial.print(" ");
  //  mySerial.print(CoDrone.droneAttitude[1]); mySerial.print(" ");
  //  mySerial.print(CoDrone.droneAttitude[2]); mySerial.print(" ");
  //  mySerial.print(CoDrone.Alive.Attitude); mySerial.print(" ");
  //  mySerial.println();

  Send_Processing(tType_Attitude, data, len);
}


void Send_AnalogStick()
{
  byte data[12];
  byte len = 8;
  data[0] = LowB(Roll);
  data[1] = HighB(Roll);
  data[2] = LowB(Pitch);
  data[3] = HighB(Pitch);
  data[4] = LowB(Yaw);
  data[5] = HighB(Yaw);
  data[6] = LowB(Throttle);
  data[7] = HighB(Throttle);
  Send_Processing(tType_AnalogStick, data, len);
}


void Send_GyroBias()
{
  byte data[12];
  byte len = 7;
  data[0] = CoDrone.droneGyroBias[0]; //Roll
  data[1] = CoDrone.droneGyroBias[1];
  data[2] = CoDrone.droneGyroBias[2]; //Pitch
  data[3] = CoDrone.droneGyroBias[3];
  data[4] = CoDrone.droneGyroBias[4]; //Yaw
  data[5] = CoDrone.droneGyroBias[5];
  data[6] = CoDrone.Alive.GyroBias;
  Send_Processing(tType_GyroBias, data, len);
}


void Send_TrimAll()
{
  byte data[12];
  byte len = 11;
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
  data[10] = CoDrone.Alive.TrimAll;
  Send_Processing(tType_TrimAll, data, len);
}


void Send_IrMessage()
{
  byte data[12];
  byte len = 6;
  data[0] = CoDrone.droneIrMassage[0];  //Direction
  data[1] = CoDrone.droneIrMassage[1];
  data[2] = CoDrone.droneIrMassage[2];
  data[3] = CoDrone.droneIrMassage[3];
  data[4] = CoDrone.droneIrMassage[4];
  data[5] = CoDrone.Alive.IrMessage;
  Send_Processing(tType_IrMessage, data, len);
}


void Send_ImuRawAndAngl()
{
  byte data[19];
  byte len = 19;
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
  data[16] = LowB(CoDrone.droneImuRawAndAngle[8]);  //AngleYaw
  data[17] = HighB(CoDrone.droneImuRawAndAngle[8]);
  data[18] = CoDrone.Alive.ImuRawAndAngle;

  Send_Processing(tType_ImuRawAndAngl, data, len);
}

void Send_Pressure()
{
  byte data[17];
  byte len = 17;

  for (int i = 0; i < len - 1; i++) {
    data[i] = CoDrone.dronePressure[i];
  }
  data[16] = CoDrone.Alive.Pressure;
  Send_Processing(tType_Pressure, data, len);
}

void Send_Temperature()
{
  byte data[9];
  byte len = 9;

  for (int i = 0; i < len - 1; i++) {
    data[i] = CoDrone.droneTemperature[i];
  }
  data[8] = CoDrone.Alive.Temperature;
  Send_Processing(tType_Temperature, data, len);
}

void Send_Range()
{
  byte data[13];
  byte len = 13;

  for (int i = 0; i < len - 1; i += 2) {
    data[i] = LowB(CoDrone.droneRange[i / 2]);
    data[i + 1] = HighB(CoDrone.droneRange[i / 2]);
  }
  data[12] = CoDrone.Alive.Range;
  Send_Processing(tType_Range, data, len);
}


void Send_String(String s)
{
  byte data[100];
  int i = 0;

  while (s[i] != '\0') {
    data[i] = s[i];
    i++;
  }
  data[i] = '\0';

  Send_Processing(tType_String, data, i + 1);
}


byte trans_state(byte state)
{
  static byte next_state;

  if (((IR_Sensor & 0xff) == 0x00) || (CmdCtrlState == cmdType_Control)) {
    next_state = cmdType_Control;
  } if (((IR_Sensor & 0x80) != 0x00) || (CmdCtrlState == cmdType_Stop)) {
    next_state = cmdType_Stop;
  } else if (((IR_Sensor & 0x01) != 0x00) || (CmdCtrlState == cmdType_Hover)) {
    next_state = cmdType_Hover;
  } else if (CmdCtrlState == cmdType_GainTune) {
    next_state = cmdType_TrimTune;
  }
  return next_state;
}

void state_machine()
{
  static byte old_state = cmdType_Control;
  static byte state = cmdType_Control;
  boolean state_change = false;

  state = trans_state(state);
  ControlState = state;

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
  if (state_change) {
    CoDrone.Buzz(2000, 4);
    CoDrone.Buzz(4000, 4);
  }
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

float e[3];
float Ts;
float P, I, D;
float h, d_h;
float u, du;
float preTime;
void state_Hover(boolean state_change)
{
  h = (float)CoDrone.droneRange[5];
  if (state_change) {
    d_h = h;     //とりあえず
    u = 0;
    e[0]=0;
    e[1]=0;
    e[2]=0;
    preTime = micros();
    return;
  }
  e[0]  = d_h - h;

  Ts = (micros() - preTime) / 1000000;
  preTime = micros();

  P =  EEP.Throttle_Kp * (e[0] - e[1]);
  I =  EEP.Throttle_Ki * e[0] * Ts;
  D =  EEP.Throttle_Kd * (e[0] - 2 * e[1] + e[2]) / Ts;
  du = P + I + D;
  RANGE_CHECK(du, -100, 100);
  u = u + du;
  
  Send_String("\tu: ");
  dtostrf(u, 8, 6, buff);
  Send_String(buff);

  Send_String("\th: ");
  dtostrf(h, 8, 6, buff);
  Send_String(buff);

  Send_String("\td_h: ");
  dtostrf(d_h, 8, 6, buff);
  Send_String(buff);
  Send_String("\n");
  
  RANGE_CHECK(u, -100, 100);
  
  e[2]  = e[1];
  e[1]  = e[0];
  
  YAW       = Yaw;      // Set the A3 analog pin to control the Yaw
  THROTTLE  = (int)u;   // Set the A4 analog pin to control the Throttle
  ROLL      = Roll;     // Set the A5 analog pin to control the Roll
  PITCH     = Pitch;    // Set the A6 analog pin to control the Pitch
  CoDrone.Control(SEND_INTERVAL); // Send the new flight commands at the SEND_INTERVAL (50ms)
  
//  Send_String("P: ");
//  dtostrf(P, 8, 6, buff);
//  Send_String(buff);
//  
//  Send_String("\tI: ");
//  dtostrf(I, 8, 6, buff);
//  Send_String(buff);
//
//  Send_String("\tD: ");
//  dtostrf(D, 8, 6, buff);
//  Send_String(buff);


}

void state_GainTune(boolean state_change)
{

}

void Send_Processing(byte _cType, byte *_data, byte _length)
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
  for (int i = 0; i < _length ; i++)
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


static byte cmdBuff[MAX_PACKET_LENGTH];
static byte dataBuff[MAX_PACKET_LENGTH];
static byte cmdIndex;
static byte checkHeader;
static byte receiveDtype;
static byte receiveLength;

void receive_pcdata(void) {
  CmdCtrlState = cmdType_EndOfType;
  if ( mySerial.available() >= 0 ) {
    int input = mySerial.read();
    cmdBuff[cmdIndex++] = (byte)input;

    if (cmdIndex >= MAX_PACKET_LENGTH) {
      checkHeader = 0;
      cmdIndex = 0;
    } else {
      if (cmdIndex == 1) {
        if (cmdBuff[0] == START1) {
          checkHeader = 1;
        } else {
          checkHeader = 0;
          cmdIndex = 0;
        }
      } else if (cmdIndex == 2)
      {
        if (checkHeader == 1) {
          if (cmdBuff[1] == START2) {
            checkHeader = 2;
          } else {
            checkHeader = 0;
            cmdIndex = 0;
          }
        }
      } else if (checkHeader == 2) {
        if (cmdIndex == 3) {
          receiveDtype =  cmdBuff[2];
          dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
        } else if (cmdIndex == 4) {
          receiveLength = cmdBuff[3];
          dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
        } else if (cmdIndex > 4) {
          if (receiveLength + 4 >= cmdIndex) {
            dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
          }
          if (receiveLength + 4 <= cmdIndex) {
            if (receiveDtype == PCcmdType_Control) {
              CmdCtrlState = dataBuff[2];
            } else if (receiveDtype == PCcmdType_GainTune) {
              int Kp, Ki, Kd;
              Kp = (dataBuff[5] << 24) | (dataBuff[4] << 16) | (dataBuff[3] << 8) | (dataBuff[2]);
              Ki = (dataBuff[9] << 24) | (dataBuff[8] << 16) | (dataBuff[7] << 8) | (dataBuff[6]);
              Kd = (dataBuff[13] << 24) | (dataBuff[12] << 16) | (dataBuff[11] << 8) | (dataBuff[10]);

              EEP.Throttle_Kp = (float)Kp / 1000;
              EEP.Throttle_Ki = (float)Ki / 1000;
              EEP.Throttle_Kd = (float)Kd / 1000;

              Send_String("GainTune\n");
              Send_String("Kp: ");
              dtostrf(EEP.Throttle_Kp, 8, 6, buff);
              Send_String(buff);
              Send_String("\n");
              Send_String("Ki: ");
              dtostrf(EEP.Throttle_Ki, 8, 6, buff);
              Send_String(buff);
              Send_String("\n");
              Send_String("Kd: ");
              dtostrf(EEP.Throttle_Kd, 8, 6, buff);
              Send_String(buff);
              Send_String("\n");
            }
            checkHeader = 0;
            cmdIndex = 0;
          }
        }
      } else {
        checkHeader = 0;
        cmdIndex = 0;
      }
    }
  }
}

