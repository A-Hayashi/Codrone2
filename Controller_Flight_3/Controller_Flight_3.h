#define EEP_START_ADRESS 20
#define RANGE_CHECK(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define LowB(x) (x & 0xFF)
#define HighB(x) (x >> 8)

typedef struct _EEPSTRUCT {
  int YawTrim;
  int ThrottleTrim;
  int PitchTrim;
  int RollTrim;
  double Yaw_Kp;
  double Yaw_Ti;
  double Yaw_Td;
  double Throttle_Kp;
  double Throttle_Ti;
  double Throttle_Td;
  double Pich_Kp;
  double Pich_Ti;
  double Pich_Td;
  double Roll_Kp;
  double Roll_Ti;
  double Roll_Td;
} EEPSTRUCT;

typedef struct _COMMAND_STRUCT {
  byte Command1;
  byte Command2;
  byte Command3;
  byte Command4;
} COMMAND_STRUCT;

typedef struct _OUTPUT_STRUCT {
  int Yaw;
  int Throttle;
  int Pitch;
  int Roll;
} OUTPUT_STRUCT;

typedef struct _TARGET_STRUCT {
  int Yaw;
  int Throttle;
  int Pitch;
  int Roll;
} TARGET_STRUCT;


enum tType
{
  tType_Attitude,
  tType_GyroBias,
  tType_TrimAll,
  tType_IrMessage,
  tType_ImuRawAndAngl,
  tType_Pressure,
  tType_Temperature,
  tType_AnalogStick
};

enum rType
{
  rType_Command,
  rType_TrimAll,
  rType_Target,
  rType_Output,
  rType_Gains
};

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

class EepMgrClass {
  public:
    EEPSTRUCT Data;
    void writeall();
    void readall();
};


class AnalogStickClass {
  public:
    int Yaw, Throttle, Pitch, Roll;
    void get_sens(void);
};

class SendPcClass {
  public:
    void send();
  private:
    void Send_Attitude();
    void Send_AnalogStick();
    void Send_GyroBias();
    void Send_TrimAll();
    void Send_IrMessage();
    void Send_ImuRawAndAngl();
    void Send_Pressure();
    void Send_Temperature();
    void Send_Processing(byte _cType, byte *_data, byte _length);
};


class IrSensClass {
  public:
    byte Data;
    void get_sens();
    void port_setting();
};

class ReceivePcClass {
  public:
    COMMAND_STRUCT Command;
    OUTPUT_STRUCT Output;
    TARGET_STRUCT Target;
    void Receive();
  private:
    byte cmdBuff[MAX_PACKET_LENGTH];
    byte dataBuff[MAX_PACKET_LENGTH];
    byte cmdIndex;
    byte checkHeader;
    byte receiveDtype;
    byte receiveLength;

    void Receive_Command();
    void Receive_TrimAll();
    void Receive_Output();
    void Receive_Target();
    void Receive_Gains();
    int UniteByte(byte upper, byte lower);
};




