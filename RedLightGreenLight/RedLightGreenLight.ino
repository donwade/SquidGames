/*****************************************
  ESP32 GPS VKEL 9600 Bds
This version is for T22_v01 20190612 board
As the power management chipset changed, it
require the axp20x library that can be found
https://github.com/lewisxhe/AXP202X_Library
You must import it as gzip in sketch submenu
in Arduino IDE
This way, it is required to power up the GPS
module, before trying to read it.

Also get TinyGPS++ library from: 
https://github.com/mikalhart/TinyGPSPlus
******************************************/

#include <TinyGPS++.h>
#include <axp20x.h>

#include <SPI.h>
#include <Wire.h>  
#include "SSD1306.h" 


#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    868E6

SSD1306 display(0x3c, 21, 22);


TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;
//------------------------------------------------------------------
u_int8_t char_height = 0;

static void setFont(uint8_t size)
{
	switch (size)
	{
		case 10:
			display.setFont(ArialMT_Plain_10);
			char_height = 10;
		break;
		
		case 16:
			display.setFont(ArialMT_Plain_16);
			char_height = 16;
		break;

		case 24:
			display.setFont(ArialMT_Plain_24);
			char_height = 24;
		break;
	}
}

int  xprintf(uint8_t lineNo, const char *format, ...) 
{
	va_list args;
	va_start(args, format);
	char buffer[100];
	vsprintf(buffer, format, args);
	
    display.drawString(0, lineNo * char_height, buffer);
	
	va_end(args);
	return 0;
}

//---------------------------------------------------------

void loop(){while(1) delay(-1);};  // keep arduino happy

//---------------------------------------------------------
void led_display1(void)
{
	display.clear();
	xprintf(0, "LA=%+9.7f", gps.location.lat());
	xprintf(1, "LO=%+9.7f", gps.location.lng());
	xprintf(2, "DIR=%3d %3s", (int)gps.course.deg(), gps.cardinal(gps.course.deg()));
	xprintf(3, "M/S=%6.1f", gps.speed.mps());
	//xprintf(3, "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year());
	display.display();
}
//---------------------------------------------------------

void loop1(void *not_used)
{
	while(1)
	{
		Serial.print("Latitude  : ");
		Serial.println(gps.location.lat(), 5);


		Serial.print("Longitude : ");
		Serial.println(gps.location.lng(), 4);

		led_display1();

		Serial.print("Satellites: ");
		Serial.println(gps.satellites.value());
		Serial.print("Altitude  : ");
		Serial.print(gps.altitude.feet() / 3.2808);
		Serial.println("M");

		Serial.print("Time      : ");
		Serial.print(gps.time.hour());
		Serial.print(":");
		Serial.print(gps.time.minute());
		Serial.print(":");
		Serial.println(gps.time.second());


		Serial.print("Speed     : ");
		Serial.println(gps.speed.kmph()); 
		Serial.println("**********************");


		smartDelay(1000);

		if (millis() > 5000 && gps.charsProcessed() < 10)
		Serial.println(F("No GPS data received: check wiring"));
	}
}

//---------------------------------------------------------

void loop2(void *not_used)
{
	while(1) delay(1000);
}

//---------------------------------------------------------

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
  } while (millis() - start < ms);
}

//---------------------------------------------------------


void setup()
{
	Serial.begin(115200);
	
	// oled stuff
	pinMode(16,OUTPUT);
	digitalWrite(16, LOW);	  // set GPIO16 low to reset OLED
	delay(50); 
	digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high?

	// gps power mgt
	
	Wire.begin(21, 22);
	if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
	Serial.println("AXP192 Begin PASS");
	} else {
	Serial.println("AXP192 Begin FAIL");
	}
	axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
	axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
	axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
	axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
	axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
	GPS.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX
	
	display.init();
	display.flipScreenVertically();  
	setFont(16);
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);

	xprintf(0, "START");
	display.display();
	delay(3000);
	

	xTaskCreatePinnedToCore(loop2, "loop2", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(loop1, "loop1", 4096, NULL, 1, NULL, 0);
	 
}


