import  processing.serial.*; //<>// //<>//

enum cmdType
{
  cmdType_Control, 
    cmdType_EEP_Write, 
    cmdType_EEP_Read, 
    cmdType_TrimTune, 
    cmdType_TrimSet, 
    cmdType_Stop, 
    cmdType_Hover, 
    cmdType_GainTune,
};


enum PCcmdType
{
  PCcmdType_Control, 
    PCcmdType_GainTune
};


ReceiveData r_data;
SendData s_data;
int[]   data;
Serial  serial;

graphMonitor testGraph;
void setup() {  
  size(400, 250);
  data = new int [width];
  r_data = new ReceiveData();
  s_data = new SendData();
  serial = new Serial( this, Serial.list()[0], 9600 );

  frameRate(100);
  smooth();
  testGraph = new graphMonitor("Altitude", 100, 50, 1000, 400);
}

public void settings() {
  size(1440, 900, P3D);
}

void serialEvent(Serial port) {
  r_data.Receive(port);
}

void draw() {
  background(0xff);

  testGraph.graphDraw(r_data.AltitudeControl.d_h, r_data.AltitudeControl.h, 0);

  fill(0, 255, 0);
  ellipse(150+r_data.AnalogStick.Yaw, height-150-r_data.AnalogStick.Throttle, 20, 20);
  ellipse(150+300+r_data.AnalogStick.Roll, height-150-r_data.AnalogStick.Pitch, 20, 20);

  textSize(30);  
  textAlign(RIGHT, TOP);
  fill(0, 255, 0);
  text("ControlState: "+strs[r_data.ControlState], width-10, 0+10);
  text("Height:"+ r_data.Range.Bottom + " mm", width-10, 0+50);

  translate(width-300, height-300);
  rotateZ(radians(r_data.Attitude.Roll));
  rotateX(radians(r_data.Attitude.Pitch));
  rotateY(radians(-r_data.Attitude.Yaw));
  fill(0, 255, 0);
  box(200, 5, 300);
}



void keyPressed() {
  if (key == 'c') {
    s_data.Send_CtrlState(cmdType.cmdType_Control);
  } else if (key == 's') {
    s_data.Send_CtrlState(cmdType.cmdType_Stop);
  } else if (key == 't') {
    s_data.Send_CtrlState(cmdType.cmdType_GainTune);
  } else if (key == 'w') {
    s_data.Send_CtrlState(cmdType.cmdType_EEP_Write);
  } else if (key == 'r') {
    s_data.Send_CtrlState(cmdType.cmdType_EEP_Read);
  } else if (key == 'g') {
    s_data.Send_Gain(0.1, 0.07, 0.05);
  }
}

enum tType
{
  tType_Attitude, 
    tType_GyroBias, 
    tType_TrimAll, 
    tType_IrMessage, 
    tType_ImuRawAndAngl, 
    tType_Pressure, 
    tType_Temperature, 
    tType_AnalogStick, 
    tType_ControlState, 
    tType_Range, 
    tType_String, 
    tType_AltitudeControl,
};

String[] strs = { 
  "Control", 
  "EEP_Write", 
  "EEP_Read", 
  "TrimTune", 
  "TrimSet", 
  "Stop", 
  "Hover", 
  "GainTune", 
};


public class ReceiveData {

  private static final byte START1 = 0x0A;
  private static final byte START2 = 0x55;
  private static final byte MAX_PACKET_LENGTH = 100;
  private byte cmdBuff[] = new byte[MAX_PACKET_LENGTH];
  private byte dataBuff[] = new byte[MAX_PACKET_LENGTH];
  private byte cmdIndex;
  private byte checkHeader;
  private byte receiveDtype;
  private byte receiveLength;

  Attitude_t Attitude = new Attitude_t();
  GyroBias_t GyroBias = new GyroBias_t();
  TrimAll_t TrimAll = new TrimAll_t();
  IrMessage_t IrMessage = new IrMessage_t();
  ImuRawAndAngl_t ImuRawAndAngl = new ImuRawAndAngl_t();
  Pressure_t Pressure = new Pressure_t();
  Temperature_t Temperature = new Temperature_t();
  AnalogStick_t AnalogStick = new AnalogStick_t(); 
  Range_t Range = new Range_t();
  AltitudeControl_t AltitudeControl = new AltitudeControl_t();
  byte ControlState;


  private int UniteByte(byte upper, byte lower) {
    return ((upper<<8) | (lower & 0xFF) );
  }

  public void Receive(Serial port) {
    if ( port.available() >= 0 ) {
      int input = port.read();
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
              if (receiveDtype == tType.tType_Attitude.ordinal()) {
                Attitude.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                Attitude.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                Attitude.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                Attitude.AliveCnt = dataBuff[8];
                //print(Attitude.Roll); 
                //print(" ");
                //print(Attitude.Pitch); 
                //print(" ");
                //print(Attitude.Yaw); 
                //print(" ");
                //print(Attitude.AliveCnt); 
                //print(" ");
                //println();
              } else if (receiveDtype == tType.tType_GyroBias.ordinal()) {
                /* not available */
                GyroBias.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                GyroBias.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                GyroBias.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                GyroBias.AliveCnt = dataBuff[8];
                //print(GyroBias.Roll); print(" ");
                //print(GyroBias.Pitch); print(" ");
                //print(GyroBias.Yaw); print(" ");
                //print(GyroBias.AliveCnt); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_TrimAll.ordinal()) {
                TrimAll.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                TrimAll.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                TrimAll.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                TrimAll.Throttle = UniteByte(dataBuff[9], dataBuff[8]);
                TrimAll.Wheel = UniteByte(dataBuff[11], dataBuff[10]);
                TrimAll.AliveCnt = dataBuff[12];
                //print(TrimAll.Roll); print(" ");
                //print(TrimAll.Pitch); print(" ");
                //print(TrimAll.Throttle); print(" ");
                //print(TrimAll.Wheel); print(" ");
                //print(TrimAll.AliveCnt); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_IrMessage.ordinal()) {
                /* not available */
                IrMessage.Direction = dataBuff[2];
                IrMessage.IrMessage[0] = dataBuff[3];
                IrMessage.IrMessage[1] = dataBuff[4];
                IrMessage.IrMessage[2] = dataBuff[5];
                IrMessage.IrMessage[3] = dataBuff[6];
                IrMessage.AliveCnt     = dataBuff[7];
                //print(IrMessage.Direction);    print(" ");
                //print(IrMessage.IrMessage[0]); print(" ");
                //print(IrMessage.IrMessage[1]); print(" ");
                //print(IrMessage.IrMessage[2]); print(" ");
                //print(IrMessage.IrMessage[3]); print(" ");
                //print(IrMessage.AliveCnt);     print(" ");
                //println();
              } else if (receiveDtype == tType.tType_ImuRawAndAngl.ordinal()) {
                /* not available */
                ImuRawAndAngl.AccX = UniteByte(dataBuff[3], dataBuff[2]);
                ImuRawAndAngl.AccY = UniteByte(dataBuff[5], dataBuff[4]);
                ImuRawAndAngl.AccZ = UniteByte(dataBuff[7], dataBuff[6]);
                ImuRawAndAngl.GyroRoll = UniteByte(dataBuff[9], dataBuff[8]);
                ImuRawAndAngl.GyroPitch = UniteByte(dataBuff[11], dataBuff[10]);
                ImuRawAndAngl.GyroRoll = UniteByte(dataBuff[13], dataBuff[12]);
                ImuRawAndAngl.AngleRoll = UniteByte(dataBuff[15], dataBuff[14]);
                ImuRawAndAngl.AnglePitch = UniteByte(dataBuff[17], dataBuff[16]);
                ImuRawAndAngl.AngleRoll = UniteByte(dataBuff[19], dataBuff[18]);
                ImuRawAndAngl.AliveCnt = dataBuff[20];
                //print(ImuRawAndAngl.AccX); print(" ");
                //print(ImuRawAndAngl.AccY); print(" ");
                //print(ImuRawAndAngl.AccZ); print(" ");
                //print(ImuRawAndAngl.GyroRoll); print(" ");
                //print(ImuRawAndAngl.GyroPitch); print(" ");
                //print(ImuRawAndAngl.GyroRoll); print(" ");
                //print(ImuRawAndAngl.AngleRoll); print(" ");
                //print(ImuRawAndAngl.AnglePitch); print(" ");
                //print(ImuRawAndAngl.AngleRoll); print(" ");
                //print(ImuRawAndAngl.AliveCnt); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_Pressure.ordinal()) {
                /* not available */
                for (int i=0; i<16; i++) {
                  Pressure.Pressure[i] = dataBuff[i+2];
                }
                Pressure.AliveCnt = dataBuff[18];
                //for (int i=0; i<16; i++) {
                //  print(Pressure.Pressure[i]); print(" ");
                //}
                //print(Pressure.AliveCnt);
                //println();
              } else if (receiveDtype == tType.tType_Temperature.ordinal()) {
                /* not available */
                for (int i=0; i<8; i++) {
                  Temperature.Temperature[i] = dataBuff[i+2];
                }
                Temperature.AliveCnt = dataBuff[10];
                //for (int i=0; i<8; i++) {
                //  print(Temperature.Temperature[i]); print(" ");
                //}
                //print(Temperature.AliveCnt); 
                //println();
              } else if (receiveDtype == tType.tType_AnalogStick.ordinal()) {
                AnalogStick.Roll = UniteByte(dataBuff[3], dataBuff[2]);
                AnalogStick.Pitch = UniteByte(dataBuff[5], dataBuff[4]);
                AnalogStick.Yaw = UniteByte(dataBuff[7], dataBuff[6]);
                AnalogStick.Throttle = UniteByte(dataBuff[9], dataBuff[8]);
                //print(AnalogStick.Roll); print(" ");
                //print(AnalogStick.Pitch); print(" ");
                //print(AnalogStick.Yaw); print(" ");
                //print(AnalogStick.Throttle); print(" ");
                //println();
              } else if (receiveDtype == tType.tType_ControlState.ordinal()) {
                ControlState = dataBuff[2];
                //println(strs[ControlState]);
              } else if (receiveDtype == tType.tType_Range.ordinal()) {
                Range.Left   = UniteByte(dataBuff[3], dataBuff[2]);
                Range.Front  = UniteByte(dataBuff[5], dataBuff[4]);
                Range.Right  = UniteByte(dataBuff[7], dataBuff[6]);
                Range.Rear   = UniteByte(dataBuff[9], dataBuff[8]);
                Range.Top    = UniteByte(dataBuff[11], dataBuff[10]);
                Range.Bottom = UniteByte(dataBuff[13], dataBuff[12]);
                //println(strs[ControlState]);
              } else if (receiveDtype == tType.tType_String.ordinal()) {
                int i = 0;
                while (dataBuff[2+i]!='\0') {
                  print(char(dataBuff[2+i]));
                  i++;
                }
                //println();
              } else if (receiveDtype == tType.tType_AltitudeControl.ordinal()) {
                AltitudeControl.d_h = UniteByte(dataBuff[3], dataBuff[2]);
                AltitudeControl.h = UniteByte(dataBuff[5], dataBuff[4]);
                AltitudeControl.P = UniteByte(dataBuff[7], dataBuff[6]);
                AltitudeControl.I = UniteByte(dataBuff[9], dataBuff[8]);
                AltitudeControl.D = UniteByte(dataBuff[11], dataBuff[10]);
                AltitudeControl.du = UniteByte(dataBuff[13], dataBuff[12]);
                AltitudeControl.u = UniteByte(dataBuff[15], dataBuff[14]);

                print(AltitudeControl.h);
                print("\t");
                print(AltitudeControl.d_h);
                print("\n");
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
}



class Attitude_t {
  int Roll;
  int Pitch;
  int Yaw;
  byte AliveCnt;
}

class GyroBias_t {
  int Roll;
  int Pitch;
  int Yaw;
  byte AliveCnt;
}

class TrimAll_t {
  int Roll;
  int Pitch;
  int Yaw;
  int Throttle;
  int Wheel;
  byte AliveCnt;
}

class IrMessage_t {
  int Direction;
  int IrMessage[] = new int[4];
  byte AliveCnt;
}

class ImuRawAndAngl_t {
  int AccX;
  int AccY;
  int AccZ;
  int GyroRoll;
  int GyroPitch;
  int GyroYaw;
  int AngleRoll;
  int AnglePitch;
  int AngleYaw;
  byte AliveCnt;
}


class Range_t {
  int Left;
  int Front;
  int Right;
  int Rear;
  int Top;
  int Bottom;
  byte AliveCnt;
}

class Pressure_t {
  int Pressure[] = new int[16];
  byte AliveCnt;
}

class Temperature_t {
  int Temperature[] = new int[8];
  byte AliveCnt;
}

class AnalogStick_t {
  int Roll;
  int Pitch;
  int Yaw;
  int Throttle;
}

class AltitudeControl_t {
  int d_h;
  int h;
  int P;
  int I;
  int D;
  int du;
  int u;
}


public class SendData {

  private static final byte START1 = 0x0A;
  private static final byte START2 = 0x55;

  private void Send_Processing(byte _cType, byte _data[], byte _length)
  {
    byte _packet[] = new byte[30];
    //START CODE
    _packet[0] = START1;
    _packet[1] = START2;

    //CONTROL TYPE
    _packet[2] = _cType;

    //LENGTH
    _packet[3] = _length;

    //DATA
    for (int i = 0; i < _length; i++)
    {
      _packet[i + 4] = _data[i];
    }

    for (int i = 0; i < _length+4; i++) {
      serial.write(_packet[i]);
    }
  }

  public void Send_CtrlState(cmdType c)
  {
    byte data[] = new byte[1];
    byte len = 1;
    byte cType = (byte)PCcmdType.PCcmdType_Control.ordinal();

    data[0] = (byte)c.ordinal();

    Send_Processing(cType, data, len);
  }

  public void Send_Gain(float Kp, float Ki, float Kd)
  {
    byte data[] = new byte[12];
    byte len = 12;
    byte cType = (byte)PCcmdType.PCcmdType_GainTune.ordinal();
    int _Kp, _Ki, _Kd;

    _Kp = (int)(Kp*1000);
    _Ki = (int)(Ki*1000);
    _Kd = (int)(Kd*1000);

    println("GainTune");

    data[0] =  (byte)((_Kp>>0) & 0xff);
    data[1] = (byte)((_Kp>>8) & 0xff);
    data[2] = (byte)((_Kp>>16) & 0xff);
    data[3] = (byte)((_Kp>>24) & 0xff);
    data[4] =  (byte)((_Ki>>0) & 0xff);
    data[5] = (byte)((_Ki>>8) & 0xff);
    data[6] = (byte)((_Ki>>16) & 0xff);
    data[7] = (byte)((_Ki>>24) & 0xff);
    data[8] =  (byte)((_Kd>>0) & 0xff);
    data[9] = (byte)((_Kd>>8) & 0xff);
    data[10] = (byte)((_Kd>>16) & 0xff);
    data[11] = (byte)((_Kd>>24) & 0xff);

    Send_Processing(cType, data, len);
  }
}





class graphMonitor {
  String TITLE;
  int X_POSITION, Y_POSITION;
  int X_LENGTH, Y_LENGTH;
  float [] y1, y2, y3;
  float maxRange;
  graphMonitor(String _TITLE, int _X_POSITION, int _Y_POSITION, int _X_LENGTH, int _Y_LENGTH) {
    TITLE = _TITLE;
    X_POSITION = _X_POSITION;
    Y_POSITION = _Y_POSITION;
    X_LENGTH   = _X_LENGTH;
    Y_LENGTH   = _Y_LENGTH;
    y1 = new float[X_LENGTH];
    y2 = new float[X_LENGTH];
    y3 = new float[X_LENGTH];
    for (int i = 0; i < X_LENGTH; i++) {
      y1[i] = 0;
      y2[i] = 0;
      y3[i] = 0;
    }
  }

  void graphDraw(float _y1, float _y2, float _y3) {
    y1[X_LENGTH - 1] = _y1;
    y2[X_LENGTH - 1] = _y2;
    y3[X_LENGTH - 1] = _y3;
    for (int i = 0; i < X_LENGTH - 1; i++) {
      y1[i] = y1[i + 1];
      y2[i] = y2[i + 1];
      y3[i] = y3[i + 1];
    }
    maxRange = 1;
    for (int i = 0; i < X_LENGTH - 1; i++) {
      maxRange = (abs(y1[i]) > maxRange ? abs(y1[i]) : maxRange);
      maxRange = (abs(y2[i]) > maxRange ? abs(y2[i]) : maxRange);
      maxRange = (abs(y3[i]) > maxRange ? abs(y3[i]) : maxRange);
    }

    pushMatrix();

    translate(X_POSITION, Y_POSITION);
    fill(240);
    stroke(130);
    strokeWeight(1);
    rect(0, 0, X_LENGTH, Y_LENGTH);
    line(0, Y_LENGTH / 2, X_LENGTH, Y_LENGTH / 2);

    textSize(25);
    fill(60);
    textAlign(LEFT, BOTTOM);
    text(TITLE, 20, -5);
    textSize(22);
    textAlign(RIGHT);
    text(0, -5, Y_LENGTH / 2 + 7);
    text(nf(maxRange, 0, 1), -5, 18);
    text(nf(-1 * maxRange, 0, 1), -5, Y_LENGTH);

    translate(0, Y_LENGTH / 2);
    scale(1, -1);
    strokeWeight(1);
    for (int i = 0; i < X_LENGTH - 1; i++) {
      stroke(255, 0, 0);
      line(i, y1[i] * (Y_LENGTH / 2) / maxRange, i + 1, y1[i + 1] * (Y_LENGTH / 2) / maxRange);
      stroke(255, 0, 255);
      line(i, y2[i] * (Y_LENGTH / 2) / maxRange, i + 1, y2[i + 1] * (Y_LENGTH / 2) / maxRange);
      stroke(0, 0, 0);
      line(i, y3[i] * (Y_LENGTH / 2) / maxRange, i + 1, y3[i + 1] * (Y_LENGTH / 2) / maxRange);
    }
    popMatrix();
  }
}