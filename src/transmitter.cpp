#include <EEPROM.h>
#include "utils.h"

float UintAbs = 0;
long UcountAbs = -1;
float Utarget = 0;
float deltaH = 0;
bool Probe = false;
bool ProbeDone = false;
bool FinPoint = false;
bool Move2Point = false;
bool BlinkState = false;
bool Started = false;
bool longDistance = false;
bool RepeatMessageSend = false;
int RepeatMessageCounter = 0;
int RepeatMessage = 0;
int WaitUcounter = 0;
int CoolingCounter = 0;
float path = 0;
int steps = 0;
int PULSE = 0;
int COOLING = 0;
int PulseDuration = 0;
int RelePulseDuration = 0;
int CoolingDuration = 0;
int RotatinDuration = 0;
int Stepper2Speed = 0;
int Stepper2Acc = 0;
float StepperZPointSpeed = 0;
float StepperZPointAcc = 0;
int step2grad = 0;
float StepperWeldStep = 0;
float Stepper2WeldStep = 0;
float Stepper3WeldStep = 0;
float StepperZWeldStep = 0;
float Stepper2LastStep = 0;
float Stepper3LastStep = 0;
float StepperZLastStep = 0;
int lastStepN = 0;
float TigHeight = 0;
int WeldPerm = 0;
bool LongDistanceReleConfirm=false;
int ZPerm = 0;
int XPerm = 0;
int NPulses = 0;
int ROTATING = 0;
int killme = 0;
int step2move = 0;
int step3move = 0;
int stepmove = 0;
int TransmitCoordsCounter = 0;
float initX = 0;
float initY = 0;
float initZ = 0;

String SerialString = "";

AccelStepper stepper(AccelStepper::DRIVER, StepOut, DirOut);
AccelStepper stepper2(AccelStepper::DRIVER, StepOut2, DirOut2);
AccelStepper stepper3(AccelStepper::DRIVER, StepOut3, DirOut3);
OneButton ProbeB(ProbeBtn, false);
OneButton StartB(StartBtn, false);
Ticker tickerBlink(Blinker, BlinkMeasure, 0, MILLIS);
Ticker tickerWManager(WManage, WManageT, 0, MILLIS);

// MultiStepper steppers;

void (*resetFunc)(void) = 0;

int SetupTransmitter()
{
    LoadPars();

    pinMode(StepOut, OUTPUT);
    pinMode(DirOut, OUTPUT);
    pinMode(StepOut2, OUTPUT);
    pinMode(DirOut2, OUTPUT);
    pinMode(StepOut3, OUTPUT);
    pinMode(DirOut3, OUTPUT);
    pinMode(ProbeBtn, INPUT);
    pinMode(StartBtn, INPUT);
    pinMode(LedOut, OUTPUT);
    digitalWrite(LedOut, LOW);
    pinMode(ReleOut, OUTPUT);
    digitalWrite(ReleOut, LOW);

    ProbeB.setPressTicks(BtnPressTicks);
    ProbeB.attachLongPressStart(ProbeBFunc);

    StartB.setPressTicks(BtnPressTicks);
    StartB.attachLongPressStart(StartBSwitch);

    stepper.setPinsInverted(false, false);
    stepper.setMaxSpeed(StepperSpeed);
    stepper.setAcceleration(StepperAcc);
    stepper.setCurrentPosition(initZ);

    stepper2.setPinsInverted(false, false);
    stepper2.setMaxSpeed(Stepper2Speed);
    stepper2.setAcceleration(Stepper2Acc);
    stepper2.setCurrentPosition(initY);

    stepper3.setPinsInverted(false, false);
    stepper3.setMaxSpeed(Stepper2Speed);
    stepper3.setAcceleration(Stepper2Acc);
    stepper3.setCurrentPosition(initX);

    // steppers.addStepper(stepper);
    // steppers.addStepper(stepper2);
    // steppers.addStepper(stepper3);

    tickerBlink.start();
    tickerWManager.start();

    stepper.move(InitMoveMM * step1mm);
    stepper2.move(InitMoveMM2 * step2grad);
    stepper3.move(InitMoveMM2 * step2grad);

    Serial.println("TRANSMITTER");
    Serial.print("Pulse=");
    Serial.println(PULSE);
    Serial.print("Cooling=");
    Serial.println(COOLING);
    Serial.print("WeldStep=");
    Serial.println(Stepper2WeldStep);
    Serial.print("TigHeight=");
    Serial.println(TigHeight);
    Serial.print("Utarget:");
    Serial.println(Utarget);
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
    TransmitCoords();

    return (WeldPerm);
}

void transmitterLoop()
{
    // if (Started && longDistance){
    //     steppers.run();
    // }
    // else{
    stepper.run();
    stepper2.run();
    stepper3.run();
    //}
    tickerWManager.update();
    // tickerBlink.update();
    // ProbeB.tick();
    // StartB.tick();
}

void SerialRoutine()
{
    char inb = 0;
    String a = "";
    while (Serial.available() > 0)
    {
        inb = Serial.read();
        if (inb != '*')
        {
            SerialString = String(SerialString + String(inb));
        }
        else
        {
            a = SerialString;
            SerialString = "";
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
            case 11:
                EEPROM.put(ParAddr + i * ParAddrDelta, pars[i].toFloat());
                break;
            default:
                break;
            }
        }
        resetFunc();
    }
    else if (a.length() > MinSerialLength)
    {
        if (a == "PROBE")
        {
            if (!Started)
                ProbeBFunc();
        }
        else if (a == "START")
        {
            if (!Started)
                StartBFunc(false);
        }
        else if (a == "STARP")
        {
            if (!Started)
                StartBFunc(true);
        }
        else if (a == "STOP")
        {
            StopBFunc();
        }
        else if (a == "POINT")
        {
            if (!Started)
            {
                FinPoint = true;
                stepper.setCurrentPosition(0);
                stepper2.setCurrentPosition(0);
                stepper3.setCurrentPosition(0);
                Serial.println("Fin Point Set");
                TransmitCoords();
            }
        }
        else if (a == "RIGHT")
        {
            if (!Started)
            {
                stepper2.setAcceleration(Stepper2Acc);
                stepper2.setMaxSpeed(Stepper2Speed_DEFAULT * 4);
            }
            step2move = step2minMove;
        }
        else if (a == "LEFT")
        {
            if (!Started)
            {
                stepper2.setAcceleration(Stepper2Acc);
                stepper2.setMaxSpeed(Stepper2Speed_DEFAULT * 4);
            }
            step2move = -step2minMove;
        }
        else if (a == "Y+")
        {
            if (XPerm > 0)
            {
                if (!Started)
                {
                    stepper3.setAcceleration(Stepper2Acc);
                    stepper3.setMaxSpeed(Stepper2Speed_DEFAULT * 4);
                }
                step3move = step2minMove;
            }
        }
        else if (a == "Y-")
        {
            if (XPerm > 0)
            {
                if (!Started)
                {
                    stepper3.setAcceleration(Stepper2Acc);
                    stepper3.setMaxSpeed(Stepper2Speed_DEFAULT * 4);
                }
                step3move = -step2minMove;
            }
        }
        else if (a == "UP")
        {
            if (!Started)
            {
                stepper.setAcceleration(StepperAcc);
                stepper.setMaxSpeed(StepperSpeed * 4);
            }
            stepmove = stepminMove;
        }
        else if (a == "DOWN")
        {
            if (!Started)
            {
                stepper.setAcceleration(StepperAcc);
                stepper.setMaxSpeed(StepperSpeed * 4);
            }
            stepmove = -stepminMove;
        }
        else if (a == "RIGHTS")
        {
            if (!Started)
            {
                stepper2.setAcceleration(Stepper2Acc);
                stepper2.setMaxSpeed(Stepper2Speed_DEFAULT / 4);
            }
            step2move = step2minMove;
        }
        else if (a == "LEFTS")
        {
            if (!Started)
            {
                stepper2.setAcceleration(Stepper2Acc);
                stepper2.setMaxSpeed(Stepper2Speed_DEFAULT / 4);
            }
            step2move = -step2minMove;
        }
        else if (a == "Y+S")
        {
            if (XPerm > 0)
            {
                if (!Started)
                {
                    stepper3.setAcceleration(Stepper2Acc);
                    stepper3.setMaxSpeed(Stepper2Speed_DEFAULT / 4);
                }
                step3move = step2minMove;
            }
        }
        else if (a == "Y-S")
        {
            if (XPerm > 0)
            {
                if (!Started)
                {
                    stepper3.setAcceleration(Stepper2Acc);
                    stepper3.setMaxSpeed(Stepper2Speed_DEFAULT / 4);
                }
                step3move = -step2minMove;
            }
        }
        else if (a == "UPS")
        {
            if (!Started)
            {
                stepper.setAcceleration(StepperAcc);
                stepper.setMaxSpeed(StepperSpeed / 4);
            }
            stepmove = stepminMove;
        }
        else if (a == "DOWNS")
        {
            if (!Started)
            {
                stepper.setAcceleration(StepperAcc);
                stepper.setMaxSpeed(StepperSpeed / 4);
            }
            stepmove = -stepminMove;
        }
        else if (a == "UT+")
        {
            Utarget += 0.1;
            Serial.print("Utarget:");
            Serial.println(Utarget);
        }
        else if (a == "UT-")
        {
            Utarget -= 0.1;
            Serial.print("Utarget:");
            Serial.println(Utarget);
        }
        else if (a == "SAVE")
        {
            EEPROM.put(CoordAddr + 0 * ParAddrDelta, (float)stepper3.currentPosition());
            EEPROM.put(CoordAddr + 1 * ParAddrDelta, (float)stepper2.currentPosition());
            EEPROM.put(CoordAddr + 2 * ParAddrDelta, (float)stepper.currentPosition());
            Serial.println("Coords Saved:");
        }
        else if (a == "WELD")
        {
            if (WeldPerm==2) RepeatMessageStart(TigStartCom);
            if (WeldPerm==1) RadioSendRepeat(TigPointT);
        }
    }
}

bool LongDistanceSparkFailure(){
    if ((WeldPerm==2) && Started){
        LongDistanceReleConfirm=true;
        CoolingCounter=0;
        return true;
    }
    return false;
}

void Uroutine(int Uroute)
{
    if ((WeldPerm == 0) || longDistance)
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
            //#ifdef DEBUG
            Serial.println("KZ!");
            //#endif
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
            else if ((UcountAbs == UtargetCounts) && (Utarget < 1))
            {
                Utarget = UintAbs;
                //#ifdef DEBUG
                Serial.print("Utarget:");
                Serial.println(Utarget);
                //#endif
                UcountAbs++;
            }

            if ((Utarget > 0) && (UcountAbs > 1))
            {
                if ((UrouteF < (Utarget + UmaxD)) && (UrouteF > (Utarget - UminD)))
                {
                    deltaH = dHdU * (Utarget - UrouteF);
                    //#ifdef DEBUG
                    Serial.print("deltaH:");
                    Serial.println(deltaH);
                    //#endif
                    if (ZPerm > 0)
                    {
                        stepper.setAcceleration(StepperAcc);
                        stepper.setMaxSpeed(StepperSpeed);
                        stepper.move(deltaH * step1mm);
                    }
                }
            }
        }
    }
}

void RepeatMessageStart(int mes){
    RepeatMessage=mes;
    RepeatMessageCounter=RepeatMessageCounterMax;
    RepeatMessageSend=true;
}

void RepeatMessageStop(){
    RepeatMessageSend=false;
}

bool RepeatMessageStopBool(){
    bool temp;
    temp = RepeatMessageSend;
    RepeatMessageStop();
    return temp;
}

void RepeatMessageRoutine(){
    if (RepeatMessageSend){
        if (RepeatMessageCounter==RepeatMessageCounterMax) RadioSendRepeat(RepeatMessage);
        if (RepeatMessageCounter>0){
            RepeatMessageCounter--;
        }
        else{
            RepeatMessageCounter=RepeatMessageCounterMax;
            Serial.println("No Answer");
        }
    }
}

void WManage()
{
    SerialRoutine();
    RepeatMessageRoutine();
    if (!(longDistance && Started)) TransmitCoordsRoutine();
    
    if (Started)
    {
        if ((longDistance) && (stepper2.currentPosition() == 0) && (stepper3.currentPosition() == 0) && (stepper.currentPosition() == 0)){
            StopBFunc();
            TransmitCoordsRoutine();
        }
        else if (CoolingCounter > 0)
        {
            if (CoolingCounter == RotatinDuration)
            {
                if (!longDistance) TransmitCoords();
                if (Move2Point)
                {
                    if (steps == lastStepN)
                    {
                        if (longDistance)
                        {
                            SpeedAccSteps2PointCalc(false);
                            stepper2.moveTo(0);
                            stepper3.moveTo(0);
                            stepper.moveTo(0);
                            steps--;
                        }
                        else
                        {
                            StopBFunc();
                        }
                    }
                    else
                    {
                        if (steps == lastStepN - 1)
                        {
                            stepper.setMaxSpeed(StepperZPointSpeed);
                            stepper.setAcceleration(StepperZPointAcc);
                            // if (longDistance){
                            // long positions[2]; // Array of desired stepper positions
                            // positions[0] = 0;
                            // positions[1] = 0;
                            // positions[2] = 0;
                            // stepper.setCurrentPosition(stepper.currentPosition());
                            // stepper2.setCurrentPosition(stepper2.currentPosition());
                            // stepper3.setCurrentPosition(stepper3.currentPosition());
                            // stepper.setAcceleration(1000000);
                            // stepper2.setAcceleration(1000000);
                            // stepper3.setAcceleration(1000000);
                            // steppers.moveTo(positions);
                            // Serial.println("multisteppers");
                            //}
                            // else{
                            if (longDistance)
                            {
                                SpeedAccSteps2PointCalc(false);
                                stepper2.moveTo(0);
                                stepper3.moveTo(0);
                                stepper.moveTo(0);
                            }
                            else
                            {
                                stepper2.move(Stepper2LastStep * step2grad);
                                stepper3.move(Stepper3LastStep * step2grad);
                                stepper.move(StepperZLastStep * step1mm);
                            }
                            // Serial.println("basesteppers");
                            //}
                            path = path + sqrt(pow(Stepper2LastStep, 2) + pow(Stepper3LastStep, 2) + pow(StepperZLastStep, 2));
                        }
                        else
                        {
                            if (longDistance) {
                                if (LongDistanceReleConfirm || WeldPerm==0){
                                    SpeedAccSteps2PointCalc(false);
                                    stepper2.moveTo(0);
                                    stepper3.moveTo(0);
                                    stepper.moveTo(0);
                                }else{
                                    steps--;
                                    CoolingCounter=LongDistanceRepeatCounter;
                                    Serial.println("No Answer");
                                }
                            }
                            else{
                                stepper2.move(Stepper2WeldStep * step2grad);
                                stepper3.move(Stepper3WeldStep * step2grad);
                                stepper.setMaxSpeed(StepperZPointSpeed);
                                stepper.setAcceleration(StepperZPointAcc);
                                stepper.move(StepperZWeldStep * step1mm);
                                path = path + sqrt(pow(Stepper2WeldStep, 2) + pow(Stepper3WeldStep, 2) + pow(StepperZWeldStep, 2));
                            }
                        }
                    }
                    steps++;
                }
                else
                {
                    stepper2.move((long)(Stepper2WeldStep * step2grad));
                    steps++;
                    path = steps * Stepper2WeldStep;
                    if (!(steps < NPulses))
                    {
                        StopBFunc();
                    }
                }
                //#ifdef DEBUG
                
                if(!longDistance){
                    Serial.print("Steps : ");
                    Serial.println(steps);
                    Serial.print("Path : ");
                    Serial.println(path);
                }
                //#endif
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
                if ((WeldPerm > 0) && (!longDistance))
                {
                    RadioSendRepeat(PULSE);
                }
                else if ((WeldPerm==2) && longDistance && !LongDistanceReleConfirm){
                    RadioSendRepeat(TigStartCom);
                }
                else
                {
                    digitalWrite(ReleOut, HIGH);
                    // CoolingCounter = CoolingDuration;
                    WaitUcounter = RelePulseDuration;
                }
            }
            if (WaitUcounter == 0)
            {
                if ((WeldPerm == 0) || ((WeldPerm == 2) && longDistance))
                {
                    digitalWrite(ReleOut, LOW);
                    WaitUcounter = 1;
                    CoolingCounter = CoolingDuration;
                }
                else
                {
                    WaitUcounter = PulseDuration + 1;
                }
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
    int step3sign = 0;
    if (step3move != 0)
    {
        if (step3move > 0)
            step3sign = 1;
        else
            step3sign = -1;
        step3move -= step3sign;
        if (step3move != 0)
        {
            stepper3.move((long)(step2grad * step3sign * 10));
        }
        else
        {
            // stepper3.move(step3sign * stepper3.speed() * stepper3.speed() / 2 / Stepper2Acc);
            stepper3.stop();
            SendCoords();
        }
    }
    if (step2move != 0)
    {
        if (step2move > 0)
            step2sign = 1;
        else
            step2sign = -1;
        step2move -= step2sign;
        if (step2move != 0)
        {
            stepper2.move((long)(step2grad * step2sign * 10));
        }
        else
        {
            // stepper2.move(step2sign * stepper2.speed() * stepper2.speed() / 2 / Stepper2Acc);
            stepper2.stop();
            SendCoords();
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
            stepper.move((long)(step1mm * stepsign * 10));
        }
        else
        {
            // stepper.move(stepsign * stepper.speed() * stepper.speed() / 2 / StepperAcc);
            stepper.stop();
            SendCoords();
        }
    }
}

void ProbeBFunc()
{
    if (ZPerm > 0)
    {
        //#ifdef DEBUG
        Serial.println("Probe Start");
        //#endif
        Probe = true;
        ProbeDone = false;
        stepper.setAcceleration(StepperAcc);
        stepper.setMaxSpeed(ProbeSpeed);
        stepper.move(ProbeMoveMM * step1mm);
    }
}

void ProbeFin()
{
    Probe = false;
    ProbeDone = true;
    stepper.stop();
    stepper.setAcceleration(StepperAcc);
    stepper.setMaxSpeed(StepperSpeed);
    stepper.move(TigHeight * step1mm);
    //#ifdef DEBUG
    Serial.println("Probe Finish");
    //#endif
}

void StartBSwitch()
{
    if (Started)
    {
        StopBFunc();
    }
    else
    {
        StartBFunc(false);
    }
}

void StartBFunc(bool p)
{
#ifdef StartNeedsProbe
    if (ProbeDone)
    {
#endif
// stepper2.setCurrentPosition(0);
#ifdef FinPointProtection
        if (p && !FinPoint)
        {
#else
    if (0)
    {
#endif
            Serial.println("FINISH POINT NOT SET");
        }
        else
        {
            Move2Point = p;
            longDistance = false;
            LongDistanceReleConfirm = false;
            EEPROM.get(ParAddr + 11 * ParAddrDelta, Utarget);
            if (Utarget < 1)
            {
                Utarget = 0;
            }
            else
            {
                Serial.print("Utarget:");
                Serial.println(Utarget);
            }
            if (Move2Point)
            {
                SpeedAccSteps2PointCalc(true);
                Serial.println("TIG Start to Point");
            }
            else
            {
                stepper.setAcceleration(StepperAcc);
                stepper.setMaxSpeed(StepperSpeed);
                stepper2.setMaxSpeed(Stepper2Speed);
                stepper2.setAcceleration(Stepper2Acc);
                Stepper2WeldStep = StepperWeldStep;
                Timings(Stepper2Acc, Stepper2Speed, Stepper2WeldStep, WManageT);
                Serial.println("TIG Start");
            }
            UcountAbs = -1;
            UintAbs = 0;
            path = 0;
            steps = 0;
            WaitUcounter = PulseDuration;
            CoolingCounter = 0;
            Started = true;
        }
#ifdef StartNeedsProbe
    }
    else
    {
        Serial.println("NO PROBE - NO START");
    }
#endif
}

void SpeedAccSteps2PointCalc(bool launch)
{
    float pointPath = 0;
    pointPath = (float)(sqrt(pow((float)stepper2.currentPosition() / step2grad, 2) + pow((float)stepper3.currentPosition() / step2grad, 2) + pow((float)stepper.currentPosition() / step1mm, 2)));
    float coord2 = abs((float)stepper2.currentPosition()) / step2grad;
    float coord3 = abs((float)stepper3.currentPosition()) / step2grad;
    float zcoord = abs((float)stepper.currentPosition()) / step1mm;
    float a = zcoord / pointPath;
    float a2 = coord2 / pointPath;
    float a3 = coord3 / pointPath;
    // float tPath=0;
    StepperZPointSpeed = (float)Stepper2Speed * a / (float)step2grad * step1mm;
    StepperZPointAcc = (float)Stepper2Acc * a / (float)step2grad * step1mm;
    stepper.setMaxSpeed(StepperZPointSpeed);
    if (launch) stepper.setAcceleration(StepperZPointAcc);
    stepper2.setMaxSpeed((float)Stepper2Speed * a2);
    if (launch) stepper2.setAcceleration((float)Stepper2Acc * a2);
    stepper3.setMaxSpeed((float)Stepper2Speed * a3);
    if (launch) stepper3.setAcceleration((float)Stepper2Acc * a3);
    if (StepperWeldStep == 0)
    {
        longDistance = true;
        StepperZWeldStep = longDistStep * a;
        Stepper2WeldStep = longDistStep * a2;
        Stepper3WeldStep = longDistStep * a3;
        // StepperZWeldStep = zcoord;
        // Stepper2WeldStep = coord2;
        // Stepper3WeldStep = coord3;
        // lastStepN=1;
        // StepperZLastStep=StepperZWeldStep;
        // Stepper2LastStep=Stepper2WeldStep;
        // Stepper3LastStep=Stepper3WeldStep;
    }
    else
    {
        StepperZWeldStep = StepperWeldStep * a;
        Stepper2WeldStep = StepperWeldStep * a2;
        Stepper3WeldStep = StepperWeldStep * a3;
    }
    float AbsWeldStep = (float)(sqrt(pow(Stepper2WeldStep, 2) + pow(Stepper3WeldStep, 2) + pow(StepperZWeldStep, 2)));
    if (launch) lastStepN = (int)(pointPath / AbsWeldStep) + 1;
    Stepper2LastStep = coord2 - Stepper2WeldStep * (lastStepN - 1);
    Stepper3LastStep = coord3 - Stepper3WeldStep * (lastStepN - 1);
    StepperZLastStep = zcoord - StepperZWeldStep * (lastStepN - 1);

    /*if (Stepper2WeldStep>0){
        tPath=(float) (Stepper2WeldStep*step2grad/stepper2.maxSpeed());
    }
    else if (Stepper3WeldStep>0){
        tPath=(float) (Stepper3WeldStep*step2grad/stepper3.maxSpeed());
    }

    stepper2.setMaxSpeed((float)(Stepper2WeldStep*step2grad/tPath));
    stepper3.setMaxSpeed((float)(Stepper3WeldStep*step2grad/tPath));
    */

    /*Serial.print("StepperZWeldStep=");
    Serial.println(StepperZWeldStep);
    Serial.print("Stepper2WeldStep*step2grad=");
    Serial.println(Stepper2WeldStep * step2grad);
    Serial.print("Stepper3WeldStep*step2grad=");
    Serial.println(Stepper3WeldStep * step2grad);

    Serial.print("stepper.maxSpeed=");
    Serial.println(stepper.maxSpeed());
    Serial.print("stepper2.maxSpeed=");
    Serial.println(stepper2.maxSpeed());
    Serial.print("stepper3.maxSpeed=");
    Serial.println(stepper3.maxSpeed());
*/
    if (stepper.currentPosition() > 0)
    {
        StepperZWeldStep = -StepperZWeldStep;
        StepperZLastStep = -StepperZLastStep;
    }
    if (stepper2.currentPosition() > 0)
    {
        Stepper2WeldStep = -Stepper2WeldStep;
        Stepper2LastStep = -Stepper2LastStep;
    }
    if (stepper3.currentPosition() > 0)
    {
        Stepper3WeldStep = -Stepper3WeldStep;
        Stepper3LastStep = -Stepper3LastStep;
    }

    if (longDistance)
    {
        // stepper.setAcceleration(1000000);
        // stepper2.setAcceleration(1000000);
        // stepper3.setAcceleration(1000000);
        tickerWManager.interval(WManageTMultiStep);
        // Timings((float)Stepper2Acc * a2, Stepper2Speed * a2, WManageTMultiStep);
        PulseDuration = 0;
        if (a2>a3)
            ROTATING = int((float)step2grad * (float)(abs(Stepper2WeldStep)) / (float)(Stepper2Speed * a2) * 1000);
        else
            ROTATING = int((float)step2grad * (float)(abs(Stepper3WeldStep)) / (float)(Stepper2Speed * a3) * 1000);
        CoolingDuration = ROTATING / WManageTMultiStep;
        RotatinDuration = (int)(ROTATING / WManageTMultiStep);
        RelePulseDuration = 0;
    }
    else
    {
        tickerWManager.interval(WManageT);
        if (a2>a3)
            Timings((float)Stepper2Acc * a2, Stepper2Speed * a2, Stepper2WeldStep, WManageT);
        else
            Timings((float)Stepper2Acc * a3, Stepper2Speed * a3, Stepper3WeldStep, WManageT);
    }

    /*Serial.print("LasststepN=");
    Serial.println(lastStepN);
    Serial.print("Stepper2WeldStep=");
    Serial.println(Stepper2WeldStep);
    Serial.print("Stepper3WeldStep=");
    Serial.println(Stepper3WeldStep);
    Serial.print("Stepper2LastStep=");
    Serial.println(Stepper2LastStep);
    Serial.print("Stepper3LastStep=");
    Serial.println(Stepper3LastStep);
    Serial.print("AbsWeldStep=");
    Serial.println(AbsWeldStep);
    Serial.print("pointPath=");
    Serial.println(pointPath);
    Serial.print("a=");
    Serial.println(a);
    Serial.print("a2=");
    Serial.println(a2);
    Serial.print("a3=");
    Serial.println(a3);
    */
}

void StopBFunc()
{
    //#ifdef DEBUG
    Serial.println("TIG Stop");
    //#endif
    digitalWrite(ReleOut, LOW);
    if (WeldPerm==2) RepeatMessageStart(TigStopCom);
    if (longDistance)
    {
        stepper.setCurrentPosition(stepper.currentPosition());
        stepper2.setCurrentPosition(stepper2.currentPosition());
        stepper3.setCurrentPosition(stepper3.currentPosition());
        tickerWManager.interval(WManageT);
    }
    Started = false;
    stepper.stop();
    stepper2.stop();
    stepper3.stop();
    SendCoords();
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
            StepperWeldStep = Stepper2WeldStep;
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
        case 11:
            EEPROM.get(ParAddr + i * ParAddrDelta, Utarget);
            if (Utarget < 1)
                Utarget = 0;
            break;
        default:
            break;
        }
        EEPROM.get(CoordAddr + 0 * ParAddrDelta, initX);
        EEPROM.get(CoordAddr + 1 * ParAddrDelta, initY);
        EEPROM.get(CoordAddr + 2 * ParAddrDelta, initZ);
    }

    Timings(Stepper2Acc, Stepper2Speed, Stepper2WeldStep, WManageT);

    if (!(WeldPerm == 1))
        ZPerm = 0;
    if (ZPerm < 1)
        ProbeDone = true;
}

void Timings(float realstepper2acc, int realstepper2speed, float WStep, int WM)
{
    PulseDuration = (int)((PULSE + SparkDuration + NoSparkPause) / WM);
    ROTATING = int((float)realstepper2speed / (float)realstepper2acc * 1000 + (float)step2grad * (float)(abs(WStep)) / (float)realstepper2speed * 1000) + 300;
    CoolingDuration = (int)((COOLING + ROTATING) / WM);
    RotatinDuration = (int)(ROTATING / WM);
    RelePulseDuration = (int)((PULSE) / WM);
}

void SendCoords()
{
    TransmitCoordsCounter = TransmitCoordsCounterMax;
}

void TransmitCoords()
{
    Serial.print("X:");
    Serial.println((float)stepper3.currentPosition() / (float)step2grad, 1);
    Serial.print("Y:");
    Serial.println((float)stepper2.currentPosition() / (float)step2grad, 1);
    Serial.print("Z:");
    Serial.println((float)stepper.currentPosition() / (float)step1mm, 1);
}

void TransmitCoordsRoutine()
{
    if (TransmitCoordsCounter > 0)
    {
        TransmitCoordsCounter--;
        if (TransmitCoordsCounter == 0)
        {
            TransmitCoords();
        }
    }
}