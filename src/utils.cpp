#include "utils.h"

#ifdef RECIEVER
const byte addresses[][6] = {"00002", "00003", "00001"};
#endif
#ifdef TRANSMITTER
const byte addresses[][6] = {"00001", "00010", "00002", "00003"};
#endif

int RadioBlockCounter = 0;

RF24 radio(PIN_CE, PIN_CSN);
Ticker RadioTicker(RadioRoutine, RadioT, 0, MILLIS);

void RadioSetup(int WRadioType)
{
    radio.begin();
    #ifdef RECIEVER
        radio.openWritingPipe(addresses[2]);
        radio.openReadingPipe(1, addresses[WRadioType-1]);
        #endif
    #ifdef TRANSMITTER
        radio.openWritingPipe(addresses[1 + WRadioType]);
        radio.openReadingPipe(1, addresses[0]);
    #endif
    radio.setPALevel(RF24_PA_HIGH);
    radio.startListening();

    RadioTicker.start();
    #ifdef RECIEVER
        RadioSendRepeat(Reseted);
    #endif
}

void radioLoop()
{
    RadioTicker.update();
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
#ifdef RECIEVER
            TigStart(message);
#endif
#ifdef TRANSMITTER
            switch (message)
            {
            case Reseted:
                Serial.println("Reciever Reseted!");
                break;
            case SparkFail:
                Serial.println("Spark Failure!");
                break;
            default:
                Serial.print("U=");
                Serial.println(UconvertFloat(message));
                Uroutine(message);
                break;
            }
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
    for (i = 0; i < RadioRepeat; i++)
    {
        RadioSend(t);
        if (RadioRepeat>1) delay(RadioDelay);
    }
    #ifdef TRANSMITTER
        Serial.print("Pulse duration : ");
        Serial.println(t);
    #endif
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