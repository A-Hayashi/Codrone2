import  processing.serial.*;

ReceiveData r_data;
int[]   data;

void setup() {  
  size(400, 250);
  data = new int [width];
  Serial  serial = new Serial( this, Serial.list()[0], 9600 );
  r_data = new ReceiveData(serial);
}

void serialEvent() {
  r_data.Receive();
}

void draw() {
  background(0);
  fill(0xaa);
  ellipse(width/2, height/2, r_data.Stick_Roll*2, r_data.Stick_Pitch*2);
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

  int Stick_Roll;
  int Stick_Pitch;

  public ReceiveData(Serial p) {
    port = p;
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
              } else if (receiveDtype == tType.tType_GyroBias.ordinal()) {
              } else if (receiveDtype == tType.tType_TrimAll.ordinal()) {
              } else if (receiveDtype == tType.tType_IrMessage.ordinal()) {
              } else if (receiveDtype == tType.tType_ImuRawAndAngl.ordinal()) {
              } else if (receiveDtype == tType.tType_Pressure.ordinal()) {
              } else if (receiveDtype == tType.tType_Temperature.ordinal()) {
              } else if (receiveDtype == tType.tType_AnalogStick.ordinal()) {
                Stick_Roll = (dataBuff[3]<<8) | (dataBuff[2] & 0xFF);
                Stick_Pitch = (dataBuff[5]<<8) | (dataBuff[4] & 0xFF);
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


class Attitude{
}

class GyroBias{
}

class TrimAll{
}

class IrMessage{
}

class ImuRawAndAngl{
}

class Pressure{
}

class Temperature{
}

class AnalogStick{
}