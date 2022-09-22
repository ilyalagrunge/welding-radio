#include "utils.h"

void setup()
{
  int WeldRadioType=0;
  Serial.begin(9600);
  Serial.setTimeout(100);
  while (!Serial)
    ;
  Serial.println("START");

#ifdef RECIEVER
  WeldRadioType=SetupReciever();
#endif

#ifdef TRANSMITTER
  WeldRadioType=SetupTransmitter();
#endif

  RadioSetup(WeldRadioType);
}

void loop()
{
#ifdef TRANSMITTER
  transmitterLoop();
#endif

#ifdef RECIEVER
  recieverLoop();
#endif

  radioLoop();
}