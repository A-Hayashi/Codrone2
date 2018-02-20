import  processing.serial.*; //<>//

ReceiveData r_data;
int[]   data;

void setup() {  
  size(400, 250);
  data = new int [width];
  Serial  serial = new Serial( this, Serial.list()[0], 9600 );
  r_data = new ReceiveData(serial);
}

void serialEvent(Serial port) {
  println("eee");
  r_data.Receive();
}

void draw() {
  background(0);
  fill(0xaa);
  ellipse(width/2, height/2, r_data.AnalogStick.Roll*2, r_data.AnalogStick.Pitch*2);
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
    tType_AnalogStick;
};


public class ReceiveData {
  private Serial port;
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

  public ReceiveData(Serial p) {
    port = p;
  }

  private int UniteByte(byte upper, byte lower) {
    return ((upper<<8) | (lower & 0xFF) );
  }

  public void Receive() {
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
            if (receiveLength + 3 > cmdIndex) {
              dataBuff[cmdIndex - 3] = cmdBuff[cmdIndex - 1];
            }
            if (receiveLength + 4 <= cmdIndex) {
              if (receiveDtype == tType.tType_Attitude.ordinal()) {
                Attitude.Roll = UniteByte(dataBuff[1], dataBuff[0]);
                Attitude.Pitch = UniteByte(dataBuff[3], dataBuff[2]);
                Attitude.Yaw = UniteByte(dataBuff[5], dataBuff[4]);
              } else if (receiveDtype == tType.tType_GyroBias.ordinal()) {
                GyroBias.Roll = UniteByte(dataBuff[1], dataBuff[0]);
                GyroBias.Pitch = UniteByte(dataBuff[3], dataBuff[2]);
                GyroBias.Yaw = UniteByte(dataBuff[5], dataBuff[4]);
              } else if (receiveDtype == tType.tType_TrimAll.ordinal()) {
                TrimAll.Roll = UniteByte(dataBuff[1], dataBuff[0]);
                TrimAll.Pitch = UniteByte(dataBuff[3], dataBuff[2]);
                TrimAll.Yaw = UniteByte(dataBuff[5], dataBuff[4]);
                TrimAll.Throttle = UniteByte(dataBuff[7], dataBuff[6]);
                TrimAll.Wheel = UniteByte(dataBuff[9], dataBuff[8]);
              } else if (receiveDtype == tType.tType_IrMessage.ordinal()) {
                IrMessage.Direction = dataBuff[0];
                IrMessage.IrMessage[0] = dataBuff[1];
                IrMessage.IrMessage[1] = dataBuff[2];
                IrMessage.IrMessage[2] = dataBuff[3];
                IrMessage.IrMessage[3] = dataBuff[4];
              } else if (receiveDtype == tType.tType_ImuRawAndAngl.ordinal()) {
                ImuRawAndAngl.AccX = UniteByte(dataBuff[1], dataBuff[0]);
                ImuRawAndAngl.AccY = UniteByte(dataBuff[3], dataBuff[2]);
                ImuRawAndAngl.AccZ = UniteByte(dataBuff[5], dataBuff[4]);
                ImuRawAndAngl.GyroRoll = UniteByte(dataBuff[7], dataBuff[6]);
                ImuRawAndAngl.GyroPitch = UniteByte(dataBuff[9], dataBuff[8]);
                ImuRawAndAngl.GyroRoll = UniteByte(dataBuff[11], dataBuff[10]);
                ImuRawAndAngl.AngleRoll = UniteByte(dataBuff[13], dataBuff[12]);
                ImuRawAndAngl.AnglePitch = UniteByte(dataBuff[15], dataBuff[14]);
                ImuRawAndAngl.AngleRoll = UniteByte(dataBuff[17], dataBuff[16]);
              } else if (receiveDtype == tType.tType_Pressure.ordinal()) {
                for (int i=0; i<16; i++) {
                  Pressure.Pressure[i] = dataBuff[i];
                }
              } else if (receiveDtype == tType.tType_Temperature.ordinal()) {
                for (int i=0; i<8; i++) {
                  Temperature.Temperature[i] = dataBuff[i];
                }
              } else if (receiveDtype == tType.tType_AnalogStick.ordinal()) {
                println(dataBuff[3]);
                println(dataBuff[2]);
                AnalogStick.Roll = UniteByte(dataBuff[1], dataBuff[0]);
                AnalogStick.Pitch = UniteByte(dataBuff[3], dataBuff[2]);
                AnalogStick.Yaw = UniteByte(dataBuff[5], dataBuff[4]);
                AnalogStick.Throttle = UniteByte(dataBuff[7], dataBuff[6]);
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
}

class GyroBias_t {
  int Roll;
  int Pitch;
  int Yaw;
}

class TrimAll_t {
  int Roll;
  int Pitch;
  int Yaw;
  int Throttle;
  int Wheel;
}

class IrMessage_t {
  int Direction;
  int IrMessage[] = new int[4];
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
}

class Pressure_t {
  int Pressure[] = new int[16];
}

class Temperature_t {
  int Temperature[] = new int[8];
}

class AnalogStick_t {
  int Roll;
  int Pitch;
  int Yaw;
  int Throttle;
}