#include <AccelStepper.h>
#include "OneButton.h"

void ProbeBFunc();
void ProbeFin();
void StartBFunc(bool);
void StopBFunc();
void StartBSwitch();

void Uroutine(int);
void WManage();
void SerialRoutine();
void stepperMoves();
void SetupTransmitter();
void transmitterLoop();
void Blinker();
void LoadPars();
void SendCoords();
void TransmitCoords();
void TransmitCoordsRoutine();
void Timings(float, int);

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

#define step2minMove 3
#define stepminMove 3

#define StepperSpeed 600
#define StepperAcc 10000
#define ProbeSpeed 120
#define step1mm 200 //    800 / 4 TR
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
//#define dHdU 0.75
#define dHdU 1.0

#define ParAddr 0
#define ParAddrDelta sizeof(float)
#define ParCount 12
#define MinSerialLength 1
#define CommandSerialLength 10

#define TransmitCoordsCounterMax 10