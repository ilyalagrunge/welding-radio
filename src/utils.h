#include <Ticker.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <GyverWDT.h>

#include "transmitter.h"

#include "reciever.h"

#define DEBUG
//#define BLINKALWAYS
//#define StartNeedsProbe

#define SparkFail 1
#define Reseted 2
#define TigStartCom 1
#define TigStopCom 2

#define PIN_CE 8
#define PIN_CSN 10
#define dataRepeat 3
#define RadioT 50
#define RadioRepeat 1
#define RadioDelay 50
#define RadioBlock 5
#define RadioBlockRadioRele 1

#define ThU 0.2
#define LimU 40
#define ThImin 30
#define ThImax 50
#define kU 20.48
#define Uoffset 0.93
#define kI 2.048
#define Uprobe 1.5
#define SparkDuration 1000

void RadioRoutine();
void RadioSend(int);
void RadioSendRepeat(int);
void RadioSetup(int);
void radioLoop();

float UconvertFloat(int);
int UconvertInt(float);
float IconvertFloat(int);
