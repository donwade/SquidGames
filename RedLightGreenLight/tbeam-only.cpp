#ifndef ARDUINO_M5STACK_Core2
#include "tbeam-only"
#include <axp20x.h>
#include <SPI.h>
#include <Wire.h>  
#include "SSD1306.h" 

AXP20X_Class axp;

#define BUILTIN_LED 4  // TIP t-beam
bool bActionGPIO38	= false;

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

SSD1306 display(0x3c, 21, 22);

// Theres only one USR button on the T-beam .

#define GPIO_BUTTON 38

void IRAM_ATTR snapShotISR() 
{
	unsigned long now = 0;	
	static unsigned long lastTimeCalled = 0; 

    now = millis();

	// if last down time is < 250mS its a bounce of earlier event
	// just ignore and reset the counter.

	if (now - lastTimeCalled < 250) 
	{
		lastTimeCalled = now;
		return;
	}

	// no activity in the last 250ms, must be a new event.
	
	if (! bActionGPIO38)
	{
		bActionGPIO38  = true;
	}
}
//----------------------------

void setupButton() 
{
    pinMode(GPIO_BUTTON, INPUT_PULLUP);
    attachInterrupt(GPIO_BUTTON, snapShotISR, FALLING);
}

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
//---------------------------------------------------------

int  xprintf(uint8_t lineNo, const char *format, ...) 
{
	va_list args;
	va_start(args, format);
	char buffer[30];
	vsnprintf(buffer, sizeof(buffer)-1, format, args);

	// erase past background to black
	display.setColor(BLACK);
	display.fillRect(0, lineNo * char_height, display.getWidth(), char_height);
	display.setColor(WHITE);
	
	display.drawString(0, lineNo * char_height, buffer);
	
	va_end(args);
	return 0;
}

//---------------------------------------------------------

int  oprintf(uint8_t lineNo, const char *format, ...) 
{
	va_list args;
	va_start(args, format);
	char buffer[30];
	vsnprintf(buffer, sizeof(buffer)-1, format, args);

	// erase past background to black
	display.setColor(BLACK);
	display.fillRect(0, lineNo * char_height, display.getWidth(), char_height);
	
	display.setColor(WHITE);
	display.drawRect(0, (lineNo * char_height)+1 , display.getWidth(), char_height );
	
	display.drawString(0, lineNo * char_height, buffer);
	
	va_end(args);
	return 0;
}


//---------------------------------------------------------
// inverted printf  (black text on white background)

int  iprintf(uint8_t lineNo, const char *format, ...) 
{
	va_list args;
	va_start(args, format);
	char buffer[30];
	vsnprintf(buffer, sizeof(buffer)-1, format, args);

	// erase past background to WHITE
	display.setColor(WHITE);
	display.fillRect(0, lineNo * char_height, display.getWidth(), char_height);
	display.setColor(BLACK);
	
	display.drawString(0, lineNo * char_height, buffer);
	
	va_end(args);
	return 0;
}
//------------------------------------------------------------------

void setBlueLED(bool ON)
{
	axp.setChgLEDMode(ON ? AXP20X_LED_LOW_LEVEL : AXP20X_LED_OFF);
}

void setRedLED(bool ON)
{
	digitalWrite(BUILTIN_LED, ON ? 0 : 1);
}


//---------------------------------------------------------
void setup_tbeam(void)
{
	setupButton(); 

	// oled stuff
	pinMode(16,OUTPUT);
	digitalWrite(16, LOW);	  // set GPIO16 low to reset OLED
	delay(50); 
	digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high?

	// gps power mgt
	Wire.begin(21, 22);
	if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) 
	{
		Serial.println("AXP192 Begin PASS");
	} 
	else
	{
		Serial.println("AXP192 Begin FAIL");
	}
	
	axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);		//lora
	axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);		//gps
	axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
	axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
	axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);	//oled

	GPS.begin(9600, SERIAL_8N1, 34, 12);   //17-TX 18-RX

	display.init();
	display.flipScreenVertically();  
	setFont(16);
	
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);

#ifdef BT_enabled
	// bluetooth init
	a2dp_source.set_ssid_callback(isValid);
	a2dp_source.set_auto_reconnect(false);
	a2dp_source.set_data_callback_in_frames(get_data_frames);
	a2dp_source.set_volume(30);
	a2dp_source.start();  
#endif

	radioTxInit();

	xprintf(0, "BUILD");
	xprintf(1, "%s" , __DATE__);
	xprintf(2, "%s", __TIME__);
	display.display();

	pinMode(BUILTIN_LED, OUTPUT);
	setRedLED(0);
	setBlueLED(0);

}
//---------------------------------------------------------

void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
	  delay(100);		// stop hard loop allow multi tasking
	  esp_task_wdt_reset();
  } while (millis() - start < ms);
}

#else
#include <Arduino.h>
void setup_tbeam(void) 
{
	Serial.printf("not a t-beam ... moving on");
};


#endif


