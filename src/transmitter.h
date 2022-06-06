void ProbeBFunc();
void ProbeFin();
void StartBFunc();
void StopBFunc();
void Uroutine(int);
void WManage();
void SerialRoutine();
void Blinker();
void LoadPars();
void stepperMoves();

#define StepOut 4
#define DirOut 5
#define StepOut2 6
#define DirOut2 7
#define ProbeBtn 2
#define StartBtn 3
#define LedOut 9

#define step2minMove 10
#define stepminMove 10

#define StepperSpeed 3000
#define StepperAcc 10000
#define ProbeSpeed 500
#define step1mm 3200 / 5
#define ProbeMoveMM -5
#define InitMoveMM 0

#define Stepper2Speed_DEFAULT 10000
#define Stepper2Acc_DEFAULT 3000
#define step2grad_DEFAULT 250
#define InitMoveMM2 0

#define Stepper2WeldStep_DEFAULT 2
#define PULSE_DEFAULT 300
#define COOLING_DEFAULT 7000
#define TigHeight_DEFAULT 3

#define ROTATING_DEFAULT 1000

#define UtargetCounts 3

#define BtnPressTicks 10

#define BlinkMeasure 500

#define WManageT 50
#define NoSparkPause 1000

#define UmaxD 1
#define UminD 2

#define ParAddr 0
#define ParAddrDelta sizeof(float)
#define ParCount 11
#define MinSerialLength 10

float UintAbs = 0;
long UcountAbs = -1;
float Utarget = 0;
float dHdU = 0.75;
float deltaH = 0;
bool Probe = false;
bool ProbeDone = false;
bool BlinkState = false;
bool Started = false;
int WaitUcounter = 0;
int CoolingCounter = 0;
float path = 0;
int steps = 0;
int PULSE = 0;
int COOLING = 0;
int PulseDuration = 0;
int CoolingDuration = 0;
int RotatinDuration = 0;
int Stepper2Speed = 0;
int Stepper2Acc = 0;
int step2grad = 0;
float Stepper2WeldStep = 0;
float TigHeight = 0;
int WeldPerm = 0;
int ZPerm = 0;
int XPerm = 0;
int NPulses = 0;
int ROTATING = 0;
int killme = 0;
int step2move = 0;
int stepmove = 0;