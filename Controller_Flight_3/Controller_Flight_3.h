
typedef struct _EEPSTRUCT {
  int YawTrim;
  int ThrottleTrim;
  int PitchTrim;
  int RollTrim;
  float Yaw_Kp;
  float Yaw_Ti;
  float Yaw_Td;
  float Throttle_Kp;
  float Throttle_Ki;
  float Throttle_Kd;
  float Pich_Kp;
  float Pich_Ki;
  float Pich_Kd;
  float Roll_Kp;
  float Roll_Ki;
  float Roll_Kd;
} EEPSTRUCT;

#define EEP_START_ADRESS 20
#define RANGE_CHECK(x,min,max) ((x= (x<min  ? min : x<max ? x : max)))
#define LowB(x) (x & 0xFF)
#define HighB(x) (x >> 8)
#define MAX_PACKET_LENGTH 100

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
  cmdType_EndOfType,
};


enum PCcmdType
{
  PCcmdType_Control,
  PCcmdType_GainTune
};

