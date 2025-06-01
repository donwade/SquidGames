#ifndef ARDUINO_M5STACK_Core2
extern void radioSendPacket(char *message);
extern void setupRadioTx(void);
extern void setBlueLED(bool ON);
extern void setRedLED(bool ON);

void setup_tbeam(void);


extern int  xprintf(uint8_t lineNo, const char *format, ...);
extern int  iprintf(uint8_t lineNo, const char *format, ...);
extern int  oprintf(uint8_t lineNo, const char *format, ...);
#define REFRESH  display.display();

#endif


void setup_tbeam(void);
 

