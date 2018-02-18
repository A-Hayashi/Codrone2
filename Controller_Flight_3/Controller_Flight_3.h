#define STOP        0
#define EEP_READ 	1
#define CONTROL     2
#define TRIM        3
#define EEP_WRITE   4
#define HOVER	    5
#define GAIN_TUNE   6

const String TRIM_STATE[] = {
  "STOP",
  "EEP_READ",
  "CONTROL",
  "TRIM",
  "EEP_WRITE",
  "HOVER",
  "GAIN_TUNE"
};

typedef struct _EEPSTRUCT {
  int YawTrim;
  int ThrottleTrim;
  int PitchTrim;
  int RollTrim;
  double k_p;
  double k_p2;
} EEPSTRUCT;


#define INCREMENT 20
//#define SAMPLING_PERIOD   0.1   //ƒTƒ“ƒvƒŠƒ“ƒOŽüŠú
