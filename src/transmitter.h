#include <AccelStepper.h>
#include "OneButton.h"

void ProbeBFunc();
void ProbeFin();
void StartBFunc();
void StopBFunc();

void Uroutine(int);
void WManage();
void SerialRoutine();
void stepperMoves();
void SetupTransmitter();
void transmitterLoop();
void Blinker();
void LoadPars();

#define StepOut 4
#define DirOut 5
#define StepOut2 6
#define DirOut2 7
#define ProbeBtn 2
#define StartBtn 3
#define LedOut 9

#define step2minMove 5
#define stepminMove 5

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
#define MinSerialLength 1
#define CommandSerialLength 10