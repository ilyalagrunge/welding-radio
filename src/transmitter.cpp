#include <EEPROM.h>
#include "utils.h"

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

String SerialString = "";

AccelStepper stepper(AccelStepper::DRIVER, StepOut, DirOut);
AccelStepper stepper2(AccelStepper::DRIVER, StepOut2, DirOut2);
OneButton ProbeB(ProbeBtn, false);
OneButton StartB(StartBtn, false);
Ticker tickerBlink(Blinker, BlinkMeasure, 0, MILLIS);
Ticker tickerWManager(WManage, WManageT, 0, MILLIS);

void (*resetFunc)(void) = 0;

void SetupTransmitter()
{
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
    StartB.attachLongPressStart(StartBSwitch);

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
}

void transmitterLoop()
{
    tickerBlink.update();
    tickerWManager.update();
    ProbeB.tick();
    StartB.tick();
    stepper.run();
    stepper2.run();
}

void SerialRoutine()
{
    char inb = 0;
    String a = "";
    while (Serial.available()>0){
        inb = Serial.read();
        if (inb != '*')
        {
            SerialString = String(SerialString + String(inb));
        }
        else
        {
            a=SerialString;
            SerialString="";
        }
    }

    String pars[ParCount];
    int i = 0;
    int index = 0;

    if (a.length() > CommandSerialLength)
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
    else if(a.length() > MinSerialLength)
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
            stepper2.setMaxSpeed((float)(Stepper2Speed*4));
            step2move = step2minMove;
        }
        else if (a == "LEFT")
        {
            stepper2.setMaxSpeed((float)(Stepper2Speed*4));
            step2move = -step2minMove;
        }
        else if (a == "UP")
        {
            stepper.setMaxSpeed(StepperSpeed);
            stepmove = stepminMove;
        }
        else if (a == "DOWN")
        {
            stepper.setMaxSpeed(StepperSpeed);
            stepmove = -stepminMove;
        }
    }
}

void Uroutine(int Uroute)
{
    if (!WeldPerm)
        return;
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
}

void WManage()
{
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
            stepper2.move((long)(step2grad * step2sign));
        }
        else
        {
            stepper2.move(step2sign * stepper2.speed() * stepper2.speed() / 2 / Stepper2Acc);
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
            stepper.move((long)(step1mm * stepsign));
        }
        else
        {
            stepper.move(stepsign * stepper.speed() * stepper.speed() / 2 / StepperAcc);
        }
    }
}

void ProbeBFunc()
{
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
}

void ProbeFin()
{
    Probe = false;
    ProbeDone = true;
    stepper.stop();
    stepper.setMaxSpeed(StepperSpeed);
    stepper.setCurrentPosition(0);
    stepper.moveTo(TigHeight * step1mm);
#ifdef DEBUG
    Serial.println("Probe Finish");
#endif
}

void StartBSwitch(){
    if (Started){
        StopBFunc();
    }
    else{
        StartBFunc();
    }
}

void StartBFunc()
{
    if (ProbeDone)
    {
        stepper2.setCurrentPosition(0);
        stepper2.setMaxSpeed(Stepper2Speed);
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
}

void StopBFunc()
{
#ifdef DEBUG
    Serial.println("TIG Stop");
#endif
    Started = false;
    stepper.stop();
    stepper2.stop();
}

void Blinker()
{
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