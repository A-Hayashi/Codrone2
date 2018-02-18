#define STOP        0
#define TRIM_RESET  1
#define CONTROL     2
#define TRIM        3
#define TRIM_STORE  4
#define HOVER		  5

const String TRIM_STATE[] = {
  "STOP",
  "TRIM_RESET",
  "CONTROL",
  "TRIM",
  "TRIM_STORE",
  "HOVER"
};

typedef struct _TRIMSTRUCT {
  int YawTrim;
  int ThrottleTrim;
  int PitchTrim;
  int RollTrim;
} TRIMSTRUCT;


#define INCREMENT 20