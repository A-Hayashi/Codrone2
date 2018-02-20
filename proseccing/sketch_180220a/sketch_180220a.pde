import  processing.serial.*;

Serial  serial;
int[]   data;

void setup() {  
  size(400, 250);
  data = new int [width];
  serial = new Serial( this, Serial.list()[0], 9600 );
}


public static final byte START1 = 0x0A;
public static final byte START2 = 0x55;
public static final byte MAX_PACKET_LENGTH = 100;
byte cmdBuff[] = new byte[MAX_PACKET_LENGTH];
byte dataBuff[] = new byte[MAX_PACKET_LENGTH];
byte cmdIndex;
byte checkHeader;
byte receiveDtype;
byte receiveLength;

int Stick_Roll;
int Stick_Pitch;

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

void serialEvent(Serial port) {
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

void draw() {
  background(0);
  fill(0xaa);
  ellipse(width/2, height/2, Stick_Roll*2, Stick_Pitch*2);
}