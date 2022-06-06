//#define RECIEVER
#define TRANSMITTER

#ifdef TRANSMITTER
#include "transmitter.h"
#endif

#include <Arduino.h>
#include <Ticker.h>
#include <AccelStepper.h>
#include "OneButton.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>

#define DEBUG
//#define BLINKALWAYS

void RadioRoutine();
void RadioSend(int);
void RadioSendRepeat(int);

void (*resetFunc)(void) = 0;

float UconvertFloat(int);
int UconvertInt(float);
float IconvertFloat(int);

#ifdef RECIEVER
void UMeasure(int);
void IMeasure();
void Welding();
void TigStart(long);
void TigFinish();
#endif

#define PIN_CE 8
#define PIN_CSN 10
#define dataRepeat 3
#define RadioT 50
#define RadioRepeat 4
#define RadioDelay 50
#define RadioBlock 5

#define ThU 0.2
#define LimU 40
#define ThI 50
#define kU 20.48
#define Uoffset 0.93
#define kI 2.048
#define Uprobe 1.5
#define SparkDuration 1000

#ifdef RECIEVER
#define Uinput A0
#define Iinput A1
#define TigSwitch 2

#define WeldWait (long)(SparkDuration / TMeasure)

#define Umissed 20
#define Ufinish 10
#define TMeasure 5
#endif

int RadioBlockCounter = 0;

#ifdef RECIEVER
int I = 0;
float Ir = 0;
float Ur = 0;
float Uint = 0;
int Umiss = Umissed;
long Ucount = 0;
long WeldCount = 0;
bool Weld = false;
bool PulseStarted = false;
long WeldPulse = (long)(WeldWait + 200 / TMeasure);
#endif

#ifdef TRANSMITTER

#endif

#ifdef TRANSMITTER
AccelStepper stepper(AccelStepper::DRIVER, StepOut, DirOut);
AccelStepper stepper2(AccelStepper::DRIVER, StepOut2, DirOut2);
OneButton ProbeB(ProbeBtn, false);
OneButton StartB(StartBtn, false);
Ticker tickerBlink(Blinker, BlinkMeasure, 0, MILLIS);
Ticker tickerWManager(WManage, WManageT, 0, MILLIS);
#endif

#ifdef RECIEVER
Ticker ticker1(IMeasure, TMeasure, 0, MILLIS);
#endif

RF24 radio(PIN_CE, PIN_CSN);
Ticker RadioTicker(RadioRoutine, RadioT, 0, MILLIS);

#ifdef TRANSMITTER
const byte addresses[][6] = {"00001", "00002"};
#endif

#ifdef RECIEVER
const byte addresses[][6] = {"00002", "00001"};
#endif

void setup()
{
#ifdef RECIEVER
  pinMode(TigSwitch, OUTPUT);
  digitalWrite(TigSwitch, LOW);
  ticker1.start();
#endif

#ifdef TRANSMITTER
  LoadPars();

  pinMode(StepOut, OUTPUT);
  pinMode(DirOut, OUTPUT);
  pinMode(StepOut2, OUTPUT);
  pinMode(DirOut2, OUTPUT);
  pinMode(ProbeBtn, INPUT);
  pinMode(StartBtn, INPUT);
  pinMode(LedOut, OUTPUT);
  digitalWrite(LedOut, LOW);

  ProbeB.setPressTicks(BtnPressTicks);
  ProbeB.attachLongPressStart(ProbeBFunc);

  StartB.setPressTicks(BtnPressTicks);
  StartB.attachLongPressStart(StartBFunc);
  StartB.attachLongPressStop(StopBFunc);

  stepper.setPinsInverted(false, false);
  stepper.setMaxSpeed(StepperSpeed);
  stepper.setAcceleration(StepperAcc);
  stepper.setCurrentPosition(0);

  stepper2.setPinsInverted(false, false);
  stepper2.setMaxSpeed(Stepper2Speed);
  stepper2.setAcceleration(Stepper2Acc);
  stepper2.setCurrentPosition(0);

  tickerBlink.start();
  tickerWManager.start();

  stepper.move(InitMoveMM * step1mm);
  stepper2.move(InitMoveMM2 * step2grad);
#endif

  Serial.begin(9600);
  Serial.setTimeout(100);
  while (!Serial)
    ;
  Serial.println("START");
#ifdef RECIEVER
  Serial.println("RECIEVER");
#endif
#ifdef TRANSMITTER
  Serial.println("TRANSMITTER");
  Serial.print("Pulse=");
  Serial.println(PULSE);
  Serial.print("Cooling=");
  Serial.println(COOLING);
  Serial.print("WeldStep=");
  Serial.println(Stepper2WeldStep);
  Serial.print("TigHeight=");
  Serial.println(TigHeight);
  Serial.print("WeldPerm=");
  Serial.println(WeldPerm);
  Serial.print("ZPerm=");
  Serial.println(ZPerm);
  Serial.print("XPerm=");
  Serial.println(XPerm);
  Serial.print("NPulses=");
  Serial.println(NPulses);
  Serial.print("step2grad=");
  Serial.println(step2grad);
  Serial.print("Stepper2Acc=");
  Serial.println(Stepper2Acc);
  Serial.print("Stepper2Speed=");
  Serial.println(Stepper2Speed);
  Serial.print("info:step move time=");
  Serial.println(ROTATING);
#endif

  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();

  RadioTicker.start();
}

void loop()
{
#ifdef TRANSMITTER
  tickerBlink.update();
  tickerWManager.update();
  ProbeB.tick();
  StartB.tick();
  stepper.run();
  stepper2.run();
#endif

#ifdef RECIEVER
  ticker1.update();
#endif

  RadioTicker.update();
}

void Blinker()
{
#ifdef TRANSMITTER
#ifndef BLINKALWAYS
  if (ProbeDone)
  {
#endif
    if (BlinkState)
    {
      digitalWrite(LedOut, HIGH);
    }
    else
    {
      digitalWrite(LedOut, LOW);
    }
    BlinkState = !BlinkState;
#ifndef BLINKALWAYS
  }
  else
  {
    digitalWrite(LedOut, LOW);
  }
#endif
#endif
}

void LoadPars()
{
  int i = 0;
  for (i = 0; i < ParCount; i++)
  {
    switch (i)
    {
    case 0:
      EEPROM.get(ParAddr + i * ParAddrDelta, PULSE);
      if (PULSE < 0)
        PULSE = PULSE_DEFAULT;
      break;
    case 1:
      EEPROM.get(ParAddr + i * ParAddrDelta, COOLING);
      if (COOLING < 0)
        COOLING = COOLING_DEFAULT;
      break;
    case 2:
      EEPROM.get(ParAddr + i * ParAddrDelta, Stepper2WeldStep);
      break;
    case 3:
      EEPROM.get(ParAddr + i * ParAddrDelta, TigHeight);
      if (TigHeight < 0)
        TigHeight = TigHeight_DEFAULT;
      break;
    case 4:
      EEPROM.get(ParAddr + i * ParAddrDelta, WeldPerm);
      if (WeldPerm < 0)
        WeldPerm = 0;
      break;
    case 5:
      EEPROM.get(ParAddr + i * ParAddrDelta, ZPerm);
      if (ZPerm < 0)
        ZPerm = 0;
      break;
    case 6:
      EEPROM.get(ParAddr + i * ParAddrDelta, XPerm);
      if (XPerm < 0)
        XPerm = 0;
      break;
    case 7:
      EEPROM.get(ParAddr + i * ParAddrDelta, NPulses);
      if (NPulses < 0)
        NPulses = 10000;
      break;
    case 8:
      EEPROM.get(ParAddr + i * ParAddrDelta, step2grad);
      if (step2grad < 0)
        step2grad = step2grad_DEFAULT;
      break;
    case 9:
      EEPROM.get(ParAddr + i * ParAddrDelta, Stepper2Acc);
      if (Stepper2Acc < 0)
        Stepper2Acc = Stepper2Acc_DEFAULT;
      break;
    case 10:
      EEPROM.get(ParAddr + i * ParAddrDelta, Stepper2Speed);
      if (Stepper2Speed < 0)
        Stepper2Speed = Stepper2Speed_DEFAULT;
      break;
    default:
      break;
    }
  }

  PulseDuration = (int)((PULSE + SparkDuration + NoSparkPause) / WManageT);
  ROTATING = int((float)Stepper2Speed / (float)Stepper2Acc * 1000 + (float)step2grad * (float)(abs(Stepper2WeldStep)) / (float)Stepper2Speed * 1000) + 100;
  CoolingDuration = (int)((COOLING + ROTATING) / WManageT);
  RotatinDuration = (int)(ROTATING / WManageT);

  if (WeldPerm < 1)
    ZPerm = 0;
  if (ZPerm < 1)
    ProbeDone = true;
}

void SerialRoutine()
{
  while (Serial.available())
  {
    String a;
    String pars[ParCount];
    int i = 0;
    int index = 0;
    a = Serial.readString();

    if (a.length() > MinSerialLength)
    {
      for (i = 0; i < ParCount; i++)
      {
        index = a.indexOf(';');
        pars[i] = a.substring(0, index);
        a = a.substring(index + 1);
        switch (i)
        {
        case 0:
        case 1:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
          EEPROM.put(ParAddr + i * ParAddrDelta, pars[i].toInt());
          break;
        case 2:
        case 3:
          EEPROM.put(ParAddr + i * ParAddrDelta, pars[i].toFloat());
          break;
        default:
          break;
        }
      }
      resetFunc();
    }
    else
    {
      if (a == "PROBE")
      {
        ProbeBFunc();
      }
      else if (a == "START")
      {
        StartBFunc();
      }
      else if (a == "STOP")
      {
        StopBFunc();
      }
      else if (a == "RIGHT")
      {
        step2move = step2minMove;
      }
      else if (a == "LEFT")
      {
        step2move = -step2minMove;
      }
      else if (a == "UP")
      {
        stepmove = stepminMove;
      }
      else if (a == "DOWN")
      {
        stepmove = -stepminMove;
      }
    }
  }
}

void Welding()
{
#ifdef RECIEVER
  if (PulseStarted)
  {
    if (WeldCount == 0)
    {
      digitalWrite(TigSwitch, HIGH);
    }
    if (WeldCount == WeldPulse)
    {
      digitalWrite(TigSwitch, LOW);
      PulseStarted = false;
      WeldCount = -1;
    }

    WeldCount++;

    if (Weld && (WeldCount < WeldWait))
    {
      WeldCount = WeldWait;
    }

    if (!Weld && !(WeldCount < WeldWait))
    {
      WeldCount = WeldPulse;
    }
  }
#endif
}

void TigStart(long PulseT)
{
#ifdef RECIEVER
  WeldCount = 0;
  WeldPulse = (long)(WeldWait + PulseT / TMeasure);
  PulseStarted = true;
#endif
}

void TigFinish()
{
#ifdef RECIEVER
  PulseStarted = false;
  digitalWrite(TigSwitch, LOW);
  WeldCount = 0;
#endif
}

void Uroutine(int Uroute)
{
#ifdef TRANSMITTER
  float UrouteF;
  UrouteF = UconvertFloat(Uroute);
  if (UrouteF < Uprobe)
  {
    if (Probe)
    {
      ProbeFin();
    }
    if (Started)
    {
      StopBFunc();
      if (ZPerm > 0)
        ProbeDone = false;
#ifdef DEBUG
      Serial.println("KZ!");
#endif
    }
  }
  else
  {
    if (Started)
    {
      CoolingCounter = CoolingDuration;

      if (UcountAbs < 0)
      {
        UcountAbs++;
      }
      else if (UcountAbs < UtargetCounts)
      {
        UintAbs = (UintAbs * UcountAbs + UrouteF) / (UcountAbs + 1);
        UcountAbs++;
      }
      else if (UcountAbs == UtargetCounts)
      {
        Utarget = UintAbs;
#ifdef DEBUG
        Serial.print("Utarget : ");
        Serial.println(Utarget);
#endif
        UcountAbs++;
      }

      if (Utarget > 0)
      {
        if ((UrouteF < (Utarget + UmaxD)) && (UrouteF > (Utarget - UminD)))
        {
          deltaH = dHdU * (Utarget - UrouteF);
#ifdef DEBUG
          Serial.print("deltaH : ");
          Serial.println(deltaH);
#endif
          if (ZPerm > 0)
            stepper.moveTo(stepper.currentPosition() + deltaH * step1mm);
        }
      }
    }
  }
#endif
}

void WManage()
{
#ifdef TRANSMITTER
  SerialRoutine();
  if (Started)
  {
    if (CoolingCounter > 0)
    {
      if (CoolingCounter == RotatinDuration)
      {
        if (XPerm > 0)
        {
          stepper2.moveTo(stepper2.currentPosition() + (long)(Stepper2WeldStep * step2grad));
          steps++;
          path = steps * Stepper2WeldStep;
          if (!(steps < NPulses))
          {
            StopBFunc();
          }
#ifdef DEBUG
          Serial.print("Steps : ");
          Serial.println(steps);
          Serial.print("Path : ");
          Serial.println(path);
#endif
        }
      }
      CoolingCounter--;
      if (CoolingCounter == 0)
      {
        WaitUcounter = PulseDuration;
      }
    }
    else
    {
      if (WaitUcounter == PulseDuration)
      {
        if (WeldPerm > 0)
        {
          RadioSendRepeat(PULSE);
        }
        else
        {
          CoolingCounter = CoolingDuration;
        }
      }
      if (WaitUcounter == 0)
      {
        WaitUcounter = PulseDuration + 1;
      }
      WaitUcounter--;
    }
  }
  else
  {
    stepperMoves();
  }
#endif
}

void stepperMoves()
{
  int stepsign = 0;
  int step2sign = 0;
  if (step2move != 0)
  {
    if (step2move > 0)
      step2sign = 1;
    else
      step2sign = -1;
    step2move -= step2sign;
    if (step2move != 0)
    {
      // stepper2.moveTo(stepper2.currentPosition()+(long)(step2grad*step2sign*10));
      stepper2.move((long)(step2grad * step2sign));
    }
    else
    {
      // stepper2.moveTo(stepper2.currentPosition()+(long)(step2sign*Stepper2Speed * Stepper2Speed / 2 / Stepper2Acc));
      //   stepper2.move((long)(step2sign*Stepper2Speed * Stepper2Speed / 2 / Stepper2Acc));
    }
  }
  if (stepmove != 0)
  {
    if (stepmove > 0)
      stepsign = 1;
    else
      stepsign = -1;
    stepmove -= stepsign;
    if (stepmove != 0)
    {
      // stepper.moveTo(stepper.currentPosition()+(long)(step1mm*stepsign*10));
      stepper.move((long)(step1mm * stepsign));
    }
    else
    {
      // stepper.moveTo(stepper.currentPosition()+(long)(step2sign*StepperSpeed * StepperSpeed / 2 / StepperAcc));
      //   stepper.move((long)(step2sign*StepperSpeed * StepperSpeed / 2 / StepperAcc));
    }
  }
}

void IMeasure()
{
#ifdef RECIEVER
  Welding();

  int Uinp = 0;
  Uinp = analogRead(Uinput);

  if (UconvertFloat(Uinp) < Uprobe)
  {
    TigFinish();
    RadioSendRepeat(Uinp);
  }

  I = analogRead(Iinput);
  Ir = IconvertFloat(I);
  if (Ir > ThI)
  {
    if (!Weld)
    {
      Weld = true;
      Serial.println("!!!!!Weld Start");
    }

    UMeasure(Uinp);
  }
  else if (Weld)
  {
    Weld = false;
    Serial.print("!!!!!Weld Finish");

    if ((Ucount == Ufinish) && (Uint > ThU) && (Uint < LimU))
    {
      RadioSendRepeat(UconvertInt(Uint));
    }
    Uint = 0;
    Umiss = Umissed;
    Ucount = 0;
#ifdef DEBUG
    Serial.print("Uint : ");
    Serial.println(Uint);
#endif
  }
#endif
}

void UMeasure(int AnU)
{
#ifdef RECIEVER
  Ur = UconvertFloat(AnU);
  if ((Ur > ThU) && (Ur < LimU))
  {
    if ((Umiss < 1) && (Ucount < Ufinish))
    {
      Uint = (Uint * Ucount + Ur) / (Ucount + 1);
      Ucount++;
    }
    else
    {
      Umiss--;
    }
  }
#endif
}

void ProbeBFunc()
{
#ifdef TRANSMITTER
  if (ZPerm > 0)
  {
#ifdef DEBUG
    Serial.println("Probe Start");
#endif
    Probe = true;
    ProbeDone = false;
    stepper.setMaxSpeed(ProbeSpeed);
    stepper.move(ProbeMoveMM * step1mm);
  }
#endif
}

void ProbeFin()
{
#ifdef TRANSMITTER
  Probe = false;
  ProbeDone = true;
  stepper.stop();
  stepper.setMaxSpeed(StepperSpeed);
  stepper.setCurrentPosition(0);
  stepper.moveTo(TigHeight * step1mm);
#ifdef DEBUG
  Serial.println("Probe Finish");
#endif
#endif
}

void StartBFunc()
{
#ifdef TRANSMITTER
  if (ProbeDone)
  {
    stepper2.setCurrentPosition(0);
    Utarget = 0;
    UcountAbs = -1;
    UintAbs = 0;
    path = 0;
    steps = 0;
    WaitUcounter = PulseDuration;
    CoolingCounter = 0;
    Started = true;
#ifdef DEBUG
    Serial.println("TIG Start");
#endif
  }
  else
  {
#ifdef DEBUG
    Serial.println("NO PROBE - NO START");
#endif
  }
#endif
}

void StopBFunc()
{
#ifdef TRANSMITTER
#ifdef DEBUG
  Serial.println("TIG Stop");
#endif
  Started = false;
  stepper.stop();
  stepper2.stop();
#endif
}

void RadioRoutine()
{
  int data[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  int message = 0;
  int checkcount = 0;
  int i = -1;
  int j = 0;
  if (radio.available() && (RadioBlockCounter == 0))
  {
    radio.read(&data, sizeof(data));
    while ((checkcount < dataRepeat) && (i < 6))
    {
      checkcount = 0;
      i++;
      for (j = i + 1; j < 8; j++)
      {
        if (data[j] == data[i])
        {
          checkcount++;
        }
      }
    }
    if (!(checkcount < dataRepeat) && (data[i] > 0))
    {
      RadioBlockCounter = RadioBlock;
      message = data[i];
      Serial.print("U=");
      Serial.println(UconvertFloat(message));
#ifdef RECIEVER
      TigStart(message);
#endif
#ifdef TRANSMITTER
      if (WeldPerm > 0)
        Uroutine(message);
#endif
    }
  }
  else
  {
    if (RadioBlockCounter > 0)
    {
      RadioBlockCounter--;
      radio.flush_rx();
    }
  }
}

void RadioSend(int Mes)
{
  int data[dataRepeat + 1];
  int i;
  for (i = 0; i < dataRepeat + 1; i++)
  {
    data[i] = Mes;
  }
  radio.stopListening();
  radio.write(&data, (dataRepeat + 1) * 2);
  delay(5);
  radio.startListening();
}

void RadioSendRepeat(int t)
{
  int i = 0;
  RadioSend(t);
  for (i = 0; i < RadioRepeat - 1; i++)
  {
    delay(RadioDelay);
    RadioSend(t);
  }
  Serial.print("Pulse duration : ");
  Serial.println(t);
}

float UconvertFloat(int uIn)
{
  return (((float)uIn) / kU - Uoffset);
}

int UconvertInt(float uFl)
{
  return ((int)((uFl + Uoffset) * kU));
}

float IconvertFloat(int iIn)
{
  return (((float)iIn) / kI);
}