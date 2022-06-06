void UMeasure(int);
void IMeasure();
void Welding();
void TigStart(long);
void TigFinish();
void SetupReciever();
void recieverLoop();

#define Uinput A0
#define Iinput A1
#define TigSwitch 2

#define WeldWait (long)(SparkDuration / TMeasure)

#define Umissed 20
#define Ufinish 10
#define TMeasure 5