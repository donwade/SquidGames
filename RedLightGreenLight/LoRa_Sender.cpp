//from LilyGo T-beam examples
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <LoRa.h>
#include <Wire.h>  

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)


#define BAND  433E6

unsigned int counter = 0;

//SSD1306 display(0x3c, 21, 22);

String rssi = "RSSI --";
String packSize = "--";
String packet ;

extern void cipherEncryption(char *source);
extern void cipherDecryption(char *source);

extern void setBlueLED(bool ON);

char msg[]="the quick brown fox 1,3,5";

void radioTxInit(void) {
	
  Serial.printf("radioTxInit start on %f", BAND);

  Serial.begin(115200);
  while (!Serial);
  Serial.println();

  Serial.println(msg);
  cipherEncryption(msg);
  Serial.println(msg);
  cipherDecryption(msg);
  Serial.println(msg);

  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);

  if (!LoRa.begin(BAND)) 
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
 
  //LoRa.onReceive(cbk);
  //LoRa.receive();

  Serial.println("radioTxInit ok");
   
  delay(1500);
}

void radioSendPacket(char *message) 
{

  static bool bTxToggle;
  unsigned long now = 0;  
  static unsigned long lastTimeCalled = 0; 
  
  now = millis();
//  if ( (now - lastTimeCalled) < 1000) return;
  
  // send packet
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;
 
  //toggle LED
  bTxToggle = !bTxToggle;
  setBlueLED(bTxToggle);
  
  Serial.println("ping");
  
  
}

