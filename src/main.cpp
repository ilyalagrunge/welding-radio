#include "utils.h"

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(100);
  while (!Serial)
    ;
  Serial.println("START");

#ifdef RECIEVER
  void SetupReciever();
#endif

#ifdef TRANSMITTER
  void SetupTransmitter();
#endif

  RadioSetup();
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