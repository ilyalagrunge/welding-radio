#include "utils.h"

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

Ticker ticker1(IMeasure, TMeasure, 0, MILLIS);

void SetupReciever()
{
    pinMode(TigSwitch, OUTPUT);
    pinMode(Uinput, INPUT);
    pinMode(Iinput, INPUT);
    digitalWrite(TigSwitch, LOW);
    ticker1.start();

    Serial.println("RECIEVER");
}

void recieverLoop()
{
    ticker1.update();
}

void Welding()
{
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
            RadioSendRepeat(SparkFail);
        }
    }
}

void TigStart(long PulseT)
{
    WeldCount = 0;
    WeldPulse = (long)(WeldWait + PulseT / TMeasure);
    PulseStarted = true;
}

void TigFinish()
{
    PulseStarted = false;
    digitalWrite(TigSwitch, LOW);
    WeldCount = 0;
}

void IMeasure()
{
    Welding();

    int Uinp = 0;
    Uinp = analogRead(Uinput);

    if (UconvertFloat(Uinp) < Uprobe)
    {
        TigFinish();  //kz
        RadioSendRepeat(Uinp);  //kz
    }

    I = analogRead(Iinput);
    Ir = IconvertFloat(I);
    if (Ir > ThImax)
    {
        if (!Weld)
        {
            Weld = true;
            //Serial.println("!!!!!Weld Start");
        }
        UMeasure(Uinp);
    }
    else if ((Weld) && (Ir < ThImin))
    {
        Weld = false;
        //Serial.print("!!!!!Weld Finish");

        if ((Ucount == Ufinish) && (Uint > ThU) && (Uint < LimU))
        {
            RadioSendRepeat(UconvertInt(Uint));
        }
        Uint = 0;
        Umiss = Umissed;
        Ucount = 0;
#ifdef DEBUG
        //Serial.print("Uint : ");
        //Serial.println(Uint);
#endif
    }
}

void UMeasure(int AnU)
{
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
}