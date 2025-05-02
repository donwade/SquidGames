#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include "SSD1306.h" 

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6

String rxRssi = "RSSI --";
String rxPacketSize = "--";
String rxPacket ;

void loraData(){
  //display.drawString(0 , 15 , "Received "+ rxPacketSize + " bytes");
  //display.drawStringMaxWidth(0 , 26 , 128, rxPacket);
  //display.drawString(0, 0, rxRssi); 
  //display.display();
  
  Serial.println(rxRssi);
}

void cbk(int packetSize) {
  rxPacket ="";
  rxPacketSize = String(packetSize,DEC);
  for (int i = 0; i < packetSize; i++) { rxPacket += (char) LoRa.read(); }
  rxRssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  loraData();
}

void rxSetup() {

  Serial.println("LoRa Receiver Callback");
  SPI.begin(SCK,MISO,MOSI,SS);

  LoRa.setPins(SS,RST,DI0);  
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //LoRa.onReceive(cbk);
  LoRa.receive();
  Serial.println("rxSetup ok");
  delay(1500);
}

void rxLoop() 
{
  int packetSize = LoRa.parsePacket();
  if (packetSize) { cbk(packetSize);  }
  delay(10);
}

