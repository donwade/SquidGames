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


// Bluetooth for Arduino (C) 2020 Phil Schatzmann
// https://github.com/pschatzmann/ESP32-A2DP.git
// https://github.com/pschatzmann/arduino-audio-tools.git

#include <TinyGPS++.h>
#include <axp20x.h>


#include <SPI.h>
#include <Wire.h>  
#include "SSD1306.h" 

#include "BluetoothA2DPSource.h"
#include <math.h> 

#include <assert.h>

/*
// handle diagnostic informations given by assertion and abort program execution:
void __assert_func(const char *__file, int __lineno, const char *__func, const char *__sexp) 
{
    // transmit diagnostic informations through serial link.
    //Serial.print("assert" << __file << ":" << __lineno << "(" << __func << ")" << __sexp);
    Serial.printf("assert %s:%d %s %s %s ", __file , __lineno, __func, __sexp);
}
*/
void assertion_failure(const char* expr, const char* file, int linenum)
{
    Serial.print("Assertion failed in '");
    Serial.print(file);
    Serial.print("' on line ");
    Serial.print(linenum);
    Serial.print(": ");
    Serial.println(expr);
    Serial.flush();

    while (1);
}

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

	// 'erase past background to blace'
	display.setColor(BLACK);
	display.fillRect(0, lineNo * char_height, display.getWidth(), char_height);
	
	display.setColor(WHITE);
	display.drawString(0, lineNo * char_height, buffer);
	display.display();
	
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



#define left_freq  13.
#define right_freq 40.

uint32_t callback_ctr =0;
uint32_t tick_ctr = 0;

BluetoothA2DPSource a2dp_source;

// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data

int32_t get_data_frames(Frame *frame, int32_t frame_count)
{
    static float m_time = 0.0;
    float m_amplitude = 20000.0;  // -32,768 to 32,767
    float m_tickPeriod = 1.0 / 44100.0;
    float m_phase = 0.0;
    float pi_2 = PI * 2.0;
	callback_ctr++;
	
    // fill the channel data
    for (int sample = 0; sample < frame_count; ++sample) 
	{
#if 1
		int mod = tick_ctr / left_freq;
		
		if (mod & 1)
		{
			frame[sample].channel1 = m_amplitude;
		}
		else 
		{
			frame[sample].channel1 = -m_amplitude;
		}
		
		mod = tick_ctr / right_freq;
		if (mod & 1)
		{
			frame[sample].channel2 = m_amplitude;
		}
		else 
		{
			frame[sample].channel2 = -m_amplitude;
		}
		
#else
		*/
        float left_angle = pi_2 * left_freq * m_time + m_phase;
        frame[sample].channel1 = m_amplitude * sin(left_angle);

        float right_angle = pi_2 * right_freq * m_time + m_phase;
        frame[sample].channel2 = m_amplitude * sin(right_angle);
#endif	
		tick_ctr++;
        m_time += m_tickPeriod;
    }

    return frame_count;
}

// Return true to connect, false will continue scanning: You can can use this
// callback to build a list.
bool isValid(const char* ssid, esp_bd_addr_t address, int rssi){
  static uint8_t hi = 0;
  hi++;
  Serial.printf("available SSID: %s %d\n", ssid, hi);
  xprintf( hi & 3, "%s %d", ssid, hi);
  display.display();
  //return false;
  return true;
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
	
	// bluetooth init
	a2dp_source.set_ssid_callback(isValid);
	a2dp_source.set_auto_reconnect(false);
	a2dp_source.set_data_callback_in_frames(get_data_frames);
	a2dp_source.set_volume(30);
	a2dp_source.start();  

	display.init();
	display.flipScreenVertically();  
	setFont(16);
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);

	xprintf(0, "AAAAAAA");
	display.display(); delay(1000);
	xprintf(0, "BBBBBB");
	display.display(); delay(1000);
	xprintf(0, "CCCCC");
	display.display(); delay(1000);
	xprintf(0, "DDDD");
	display.display(); delay(1000);
	
	xprintf(0, "START");
	display.display();
	delay(3000);
	

	xTaskCreatePinnedToCore(loop2, "loop2", 4096, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(loop1, "loop1", 4096, NULL, 1, NULL, 0);
	 
}


