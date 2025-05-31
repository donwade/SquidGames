#ifdef ARDUINO_M5STACK_Core2
extern void radioSendPacket(char *message);
extern void setupRadioTx(void);
extern void setBlueLED(bool ON);
extern void setRedLED(bool ON);



extern int  xprintf(uint8_t lineNo, const char *format, ...);
extern int  iprintf(uint8_t lineNo, const char *format, ...);
extern int  oprintf(uint8_t lineNo, const char *format, ...);

extern void setup_M5();

#define REFRESH // (ttgo) display.display();

#endif
 

