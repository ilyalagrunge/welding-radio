#include <AccelStepper.h>
#include "OneButton.h"
#include <MultiStepper.h>

void ProbeBFunc();
void ProbeFin();
void StartBFunc(bool);
void StopBFunc();
void StartBSwitch();

void Uroutine(int);
bool LongDistanceSparkFailure();
void WManage();
void RepeatMessageRoutine();
void RepeatMessageStart(int);
void RepeatMessageStop();
bool RepeatMessageStopBool();
void SerialRoutine();
void stepperMoves();
int SetupTransmitter();
void initTransmitter();
void transmitterLoop();
void Blinker();
void LoadPars();
void SendCoords();
void TransmitCoords();
void TransmitCoordsRoutine();
void Timings(float, int, float, int, int);
void SpeedAccSteps2PointCalc(bool);

//#define FinPointProtection

#define StepOut 4
#define DirOut 5
#define StepOut2 6
#define DirOut2 7
#define StepOut3 14
#define DirOut3 15
#define ProbeBtn 2
#define StartBtn 3
#define LedOut 9
#define ReleOut 17
#define longDistStep 5

#define step2minMove 3
#define stepminMove 3

#define StepperSpeed 300
#define StepperAcc 10000
#define ProbeSpeed 120
#define step1mm 100 //    800 / 4 TR
#define ProbeMoveMM -5
#define InitMoveMM 0

#define Stepper2Speed_Linear 600
#define Stepper2Acc_Linear 20000
#define step2grad_Linear 160

#define Stepper2Speed_Rotating 200
#define Stepper2Acc_Rotating 6000
#define step2grad_Rotating 34
#define InitMoveMM2 0

#define InitMoveMM3 0

#define Stepper3Speed_Linear 600
#define Stepper3Acc_Linear 20000
#define step3mm_Linear 160

#define Stepper3Speed_Rotating 600
#define Stepper3Acc_Rotating 20000
#define step3mm_Rotating 200

#define Stepper2WeldStep_DEFAULT 2
#define PULSE_DEFAULT 300
#define COOLING_DEFAULT 7000
#define TigHeight_DEFAULT 3

#define ROTATING_DEFAULT 1000

#define UtargetCounts 3

#define BtnPressTicks 10

#define BlinkMeasure 500

#define WManageT 50
#define WManageTMultiStep 50
#define LongDistanceRepeatCounter 10
#define NoSparkPause 1000

#define UmaxD 1
#define UminD 2
//#define dHdU 0.75
#define dHdU 1.0

#define ParAddr 0
#define CoordAddr 200
#define ParAddrDelta sizeof(float)
#define ParCount 14
#define MinSerialLength 1
#define CommandSerialLength 10

#define TransmitCoordsCounterMax 10
#define RepeatMessageCounterMax 15

#define TigPointT 200