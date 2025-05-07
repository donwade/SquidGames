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

#include <iostream>
#include <cstring>
#include <string>


#include <SPI.h>
#include <Wire.h>  
#include "SSD1306.h" 

#include "BluetoothA2DPSource.h"
#include <math.h> 

#include <assert.h>
#include "lookup.h"   //table of targets

#define BUILTIN_LED 4  // TIP t-beam

TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;

extern void radioTxInit(void);
extern void radioSendPacket(char *message); 


#define MIN_SPEED_KPH 9 // dont do compass if speed lower than this.
//------------------------------------------------------------------

typedef struct gpsLocation { double lng; double lat; };

typedef struct gpsMisc 
{
	float speed;
	const char *cardinal;
	float hdop;
	float Kmph;
	float course;		//direction in float degrees
	
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
};
	
gpsLocation iLocation;
gpsMisc     iMisc;

//-----------------------------------------------------------------

#define SIMULATOR

void getData(void)
{
#ifdef SIMULATOR
	String cppStr;
	char *cstr;
	char charo[100];
	char notUsed[30];
	char notUsed1[30];
	char clat[20];
	char clng[20];
	char cSpeed[5];
	char  cDeg[6];

	
	while(!Serial.available()) { delay(10);}

	// or many lines get read
	cppStr = Serial.readStringUntil('\n');  

	// convert 'String' to C-String
	cstr = new char [cppStr.length()+1];
	std::strcpy (cstr, cppStr.c_str());

	//18:31:02 @ +45.2944592 -75.8636137 ^  14 kph dir 110 ESE

	// sscanf  %f not available on embedded systems without hard work 	

	sscanf((char*) cstr, "%s %s %s %s %s %s %s %s %s %s\n", 
						 &notUsed, &notUsed, 
						 &clat, &clng, 
						 &notUsed,
						 &cSpeed,
						 &notUsed,
						 &notUsed1,
						 &cDeg,
						 &notUsed
						 );


	iLocation.lat = atof(clat);
	iLocation.lng = atof(clng);

	iMisc.Kmph = atof(cSpeed);
	iMisc.course = atof( cDeg);
	iMisc.cardinal = gps.cardinal(iMisc.course);

	if (cstr) delete [] cstr;

#if 0
	Serial.printf("\n\n-------start-----\n");
	Serial.println(cstr);
	Serial.printf("lat=%10.7f \n", iLocation.lat);
	Serial.printf("lng=%10.7f \n", iLocation.lng);

	Serial.printf(" k/c/c %6.4f %6.4f %s\n", 
			iMisc.Kmph, 
			iMisc.course, 
			iMisc.cardinal);
	
	Serial.println(cDeg);
	Serial.printf("-------done-----\n");
#endif

#else

	smartDelay(1000);

	iLocation.lat = gps.location.lat();
	iLocation.lng = gps.location.lng();
	iMisc.hour = gps.time.hour();
	iMisc.minute = gps.time.minute();
	iMisc.second = gps.time.second();
	iMisc.Kmph = gps.speed.kmph();
	iMisc.hdop = gps.hdop.hdop();
	iMisc.course = gps.course.deg();

	
	if (millis() > 5000 && gps.charsProcessed() < 10)
	Serial.println(F("No GPS data received: check wiring"));

#endif
}


gpsLocation gpsAverage;
bool bActionGPIO38	= false;


#define GPS_SAMPLE_SIZE 4
gpsLocation samples [ GPS_SAMPLE_SIZE ];
uint8_t sIndex;

char BT_SSID[17] = "== none ====";

//------------------------------------------------------------------

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

SSD1306 display(0x3c, 21, 22);

bool cardinalSin(int16_t windowCenter, uint8_t width, int16_t test)
{
	int16_t LHS, RHS, TEST;
	bool bInside;
	
	// Serial.printf("center=%d, width=%d, test=%d\n", windowCenter, width, test);
	
	TEST = (test + 360) % 360;
	windowCenter = (windowCenter + 360) % 360;

	LHS = (windowCenter - width);
	RHS = (windowCenter + width);
	// Serial.printf("Window RAW   LHS=%3d < X < RHS=%3d\n", LHS, RHS);
	
	// convert any coord that went negative to all positive
	LHS = ( LHS + 360) % 360;
	RHS = ( RHS + 360) % 360;

	// Serial.printf("Window RANGE LHS=%3d < X < RHS=%3d\n", LHS, RHS);

	if ( LHS < RHS )
	{
		//classic no adj needed.
	}
	else
	{
		// the window straddles about the 0 point somewhere
		// RHS will be low value, LHS hi value.
		// Promote RHS into unmodulo 360
		RHS += 360; 
		
		// Serial.printf("    RHS TWEAKED LHS=%3d < X < RHS=%3d\n", LHS, RHS);

		// where does the test point sit on the straddle line?
		// if the test sits in the wrapped area (0...low) then promote it
		
		if (TEST + 360 <= 360) // adjustment past the RHS is illegal
		{	
			TEST += 360;
			// Serial.printf("    TEST TWEAKED LHS=%3d < X < RHS=%3d\n", LHS, RHS);
		}
	}

	bInside = ( LHS <= TEST && TEST <= RHS );
	
	// Serial.printf("Window  TEST LHS=%3d < %3d < RHS=%3d\n", LHS, TEST, RHS);
	// Serial.printf( "%d/%d you are %s", test, TEST, bInside ? "INSIDE" : "NOT INSIDE");
	// Serial.println("\n");
	
	return bInside;
	
}

//------------------------------------------------------------------
void getOldestSample(gpsLocation *result)
{
	result->lat = samples[sIndex].lat;
	result->lng = samples[sIndex].lng;
}

//------------------------------------------------------------------
void calcGPSaverage(void)
{
	gpsLocation result;
	
	result.lat = 0.0;
	result.lng = 0.0;
	
	for (int i= 0; i < GPS_SAMPLE_SIZE; i++)
	{
		result.lat += samples[i].lat;
		result.lng += samples[i].lng;
	}
	result.lat /= float(GPS_SAMPLE_SIZE);
	result.lng /= float(GPS_SAMPLE_SIZE);
	gpsAverage = result;
}

//------------------------------------------------------------------
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

//---------------------------------------------------------
// return index to closest target.

int firstChoiceIndex;
int secondChoiceIndex;

int findClosestCamera(float vehicleLat, float vehicleLng)
{
	int dist;
	int course;
	int i;
	int closestDist = INT_MAX;
		
	const char *cardinal;
	
	// do not do any GPS with 0.0 it will hang (hi GD).
	if (vehicleLat < 1.0 ) return 0;
	
	for (i = 0; i <  NUM_GPS_ENTRIES; i++)
	{
		//Serial.printf("%+9.7f  %+9.7f\n",  cameraLocations[i].lat, cameraLocations[i].lng);
		//course = (int)gps.courseTo(vehicleLat, vehicleLng, cameraLocations[i].lat, cameraLocations[i].lng);
		//cardinal = gps.cardinal(course);

		dist = (int) gps.distanceBetween(vehicleLat, vehicleLng, cameraLocations[i].lat, cameraLocations[i].lng);

		if ( dist < closestDist )
		{
			secondChoiceIndex = firstChoiceIndex;
			closestDist = dist;
			firstChoiceIndex = i;
		}

	}
	
#ifdef SHOW_DECISIONS 
	Serial.println();
	Serial.printf("lat=%9.7f lng=%9.7f \n", vehicleLat, vehicleLng);
	
	for (i = 0; i <  NUM_GPS_ENTRIES; i++)
	{
		dist = (int)gps.distanceBetween(vehicleLat, vehicleLng, cameraLocations[i].lat, cameraLocations[i].lng);
		course = (int)gps.courseTo(vehicleLat, vehicleLng, cameraLocations[i].lat, cameraLocations[i].lng);
		cardinal = gps.cardinal(course);
		
		char star;

		star = (i == firstChoiceIndex) ? '1' : ' ';
		if ( star != '1' ) star = (i == secondChoiceIndex) ? '2' : ' ';
		
		Serial.printf("%c [%2d] dist=%4d course=%3d cardinal=%s\n",
			star, i,  dist, course, cardinal);
		
	}
#endif
	delay(10);	// or race condition happens
}

//---------------------------------------------------------

void loop(){while(1) delay(1000);};  // keep arduino happy

//---------------------------------------------------------
typedef enum absStates_e { MARK_START, MARK_END, ARRIVED };

absStates_e absState = MARK_START;

gpsLocation startLocation;
gpsLocation endLocation;

static int veh_course;
static const char *veh_cardinal = "???";

void stateDisplay(void)
{
	int dist;
	int course;
	const char *dir;
	
#ifdef DATA_CAPTURE	
	static uint8_t toggleCount = 0;

	toggleCount++;
	switch (absState)
	{
		case MARK_START:

			// distance has no meaning as we have no start point
			
			
			xprintf(0, "LA=%+9.7f", gpsAverage.lat);
			xprintf(1, "LN=%+9.7f", gpsAverage.lng);

			xprintf(3, "MARK START");

			if (iMisc.Kmph() > 5)
				xprintf(2, "%3d %s", veh_course, veh_cardinal);
			else
				xprintf(2, "%2d/%2d/%4d S=%2d", gpX.date.day(), gpX.date.month(), 
						gpX.date.year(), gpX.satellites.value());

			display.display();

			setRedLED(toggleCount & 1);
			setBlueLED(!(toggleCount & 1));

			if (bActionGPIO38)
			{
				startLocation = gpsAverage;
				bActionGPIO38 = false;
				absState = MARK_END;
				Serial.println("");
			}
			
		break;

		case MARK_END:

			dist = gpX.distanceBetween(startLocation.lat, startLocation.lng, gpsAverage.lat, gpsAverage.lng);
			course = (int)gps.courseTo(startLocation.lat, startLocation.lng, gpsAverage.lat, gpsAverage.lng);
			dir = gps.cardinal(course);
			
			xprintf(0, "LA=%+9.7f", gpsAverage.lat);
			xprintf(1, "LO=%+9.7f", gpsAverage.lng);
			xprintf(2, "END=%d %s", course, dir);
			xprintf(3, "MARK END!");
			display.display();

			setRedLED(toggleCount & 1);
			setBlueLED(false);


			if (bActionGPIO38)
			{
				endLocation = gpsAverage;
				bActionGPIO38 = false;
				absState = ARRIVED;

				// immediately report to serial port 
				// calc using approch logic
				dist = gps.distanceBetween(endLocation.lat, endLocation.lng, startLocation.lat, startLocation.lng );
				course = (int)gps.courseTo(endLocation.lat, endLocation.lng, startLocation.lat, startLocation.lng );
				dir = gps.cardinal(course);

				// throw out arrival, prep for C program
				// fill in onStreet and crossStreet from google maps later
				// formated nicely for cut/paste into google maps lat/long fmt
				
				Serial.printf("   { %+9.7f,%+9.7f , %3d, \"%3s\", \"onStreet\" , \"crossStreet\" }, \n"
				, endLocation.lat, endLocation.lng, course, dir);

			}
			
		break;

		case ARRIVED:	// arrived at camera.
			// course direction is view FROM distance going to CAMERA
			dist = gps.distanceBetween(endLocation.lat, endLocation.lng, startLocation.lat, startLocation.lng );
			course = (int)gps.courseTo(endLocation.lat, endLocation.lng, startLocation.lat, startLocation.lng );
			dir = gps.cardinal(course);
			
			xprintf(0, "LA=%+9.7f", endLocation.lat);
			xprintf(1, "LO=%+9.7f", endLocation.lng);
			xprintf(2, "END=%d %s", course, dir);
			xprintf(3, "APPROACHING");
			display.display();

			setRedLED(false);
			setBlueLED(toggleCount & 1);

			if (bActionGPIO38)
			{
				bActionGPIO38 = false;
				absState = MARK_START;   // and do it again
			}

	}
#else

	{
		int dist, course;
		const char *cardinal;
		
		findClosestCamera(iLocation.lat, iLocation.lng);
		
		
		course = (int)gps.courseTo(iLocation.lat, iLocation.lng, cameraLocations[firstChoiceIndex].lat, cameraLocations[firstChoiceIndex].lng);
		cardinal = gps.cardinal(course);

		dist = (int) gps.distanceBetween(iLocation.lat, iLocation.lng, cameraLocations[firstChoiceIndex].lat, cameraLocations[firstChoiceIndex].lng);
		
		xprintf(0, "%s", cameraLocations[firstChoiceIndex].onStreet);
		xprintf(1, "%s",  cameraLocations[firstChoiceIndex].crossStreet);

/*
		What is HDOP 
		< 1 Ideal

		1-2 Excellent
		Highest possible confidence level to be used for applications demanding the highest possible precision at all times.
		At this confidence level, positional measurements are considered accurate enough to meet all but the most sensitive applications.

		2-5 Good

		Represents a level that marks the minimum appropriate for making accurate decisions. Positional measurements could be used to make reliable in-route navigation suggestions to the user.
		Positional measurements could be used for calculations, but the fix quality could still be improved. A more open view of the sky is

		5-10 Moderate

		10-20 Fair
		Represents a low confidence level. Positional measurements should be discarded or used only to indicate a very rough estimate

		>20 Poor At this level, measurements should be discarded
*/

		xprintf(3, "%3d kph %3s %3.1f", (int)iMisc.Kmph, veh_cardinal, iMisc.hdop);

		if (dist > 100)
			xprintf(2, "%3d m %3d %s", dist, course, cardinal);
		else
			oprintf(2, "%3d m %3d %s", dist, course, cardinal);

		display.display();
	}	

#endif
}
//---------------------------------------------------------
extern void radioSendPacket(char *message);

void loop1(void *not_used)
{
	char msg[30];
	unsigned long startProfileTime;
	unsigned long difftime;
	
	snprintf(msg, sizeof(msg),"hello"); 

	static unsigned long lastProfileTime; 

	while(1)
	{
		getData();
		
		{
			// update rolling history
			noInterrupts();

			samples[sIndex].lat = iLocation.lat;
			samples[sIndex].lng = iLocation.lng;

			// sIndex is left pointing to NEXT position to write to on the next pass
			// therefore sIndex points to oldest entry by time

			if (++sIndex == GPS_SAMPLE_SIZE) sIndex = 0;
			interrupts();
		}	

		calcGPSaverage();

		// get direction only if going fast enough
		// otherwise it points all over the place
		
		if (iMisc.Kmph > MIN_SPEED_KPH )
		{
			gpsLocation oldest;
			getOldestSample(&oldest);
			veh_course = (int)gps.courseTo(oldest.lat, oldest.lng, iLocation.lat, iLocation.lng );
			veh_cardinal = gps.cardinal(veh_course);
		}

		
		Serial.printf("%2d:%02d:%02d @ %+9.7f %+9.7f ^ %3d kph dir %3d %s\n", 
				iMisc.hour,iMisc.minute,iMisc.second,
				iLocation.lat, iLocation.lng,
				(int)iMisc.Kmph, (int)iMisc.course, gps.cardinal(iMisc.course)
				);

		// profile loop time. So far about 3ms total		
		//difftime =  micros() - startProfileTime;
		//Serial.printf("profile = %d uS\n", difftime);

		stateDisplay();
		
		//startProfileTime = micros();

		
	}
}


//-----------------------------------------------------------
// BLUETOOTH

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


bool isValid(const char* btSSID, esp_bd_addr_t address, int rssi){
  static uint8_t hi = 0;
  hi++;
  snprintf(BT_SSID, sizeof(BT_SSID), "%s", btSSID);
  Serial.printf("available SSID: %s %d\n", btSSID, hi);
  //return false;
  return true;
}

//---------------------------------------------------------

void loop2(void *not_used)
{
	while(1)
	{
		delay(300);
	}
}

//---------------------------------------------------------

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS.available())
      gps.encode(GPS.read());
	  delay(100);		// stop hard loop allow multi tasking
  } while (millis() - start < ms);
}

//---------------------------------------------------------
extern void setupRadioTx(void);

void setBlueLED(bool ON)
{
	axp.setChgLEDMode(ON ? AXP20X_LED_LOW_LEVEL : AXP20X_LED_OFF);
}

void setRedLED(bool ON)
{
	digitalWrite(BUILTIN_LED, ON ? 0 : 1);
}

//---------------------------------------------------------

void setup()
{

	Serial.begin(115200);

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

#if 0	
	// bluetooth init
	a2dp_source.set_ssid_callback(isValid);
	a2dp_source.set_auto_reconnect(false);
	a2dp_source.set_data_callback_in_frames(get_data_frames);
	a2dp_source.set_volume(30);
	a2dp_source.start();  
#endif

	display.init();
	display.flipScreenVertically();  
	setFont(16);
	
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);

	radioTxInit();

	xprintf(0, "BUILD");
	xprintf(1, "%s" , __DATE__);
	xprintf(2, "%s", __TIME__);
	display.display();

	pinMode(BUILTIN_LED, OUTPUT);
	setRedLED(0);
	setBlueLED(0);

	delay(4000);

#if 0  // set cardinal view range
	#define STEP 20
	for (int x = 0; x < 360; x +=STEP)
	{
		Serial.println(cardinalSin(x, 20, x + 21));  // test for just outside RHS
	}
	Serial.println("---------");
	
	for (int x = 0; x < 360; x +=STEP)
	{
		Serial.println(cardinalSin(x, 20, x - 21));  // test for just outside LHS
	}
	Serial.println("---------");

	for (int x = 0; x < 360; x +=STEP)
	{
		Serial.println(cardinalSin(x, 20, x - 19));  // test for just inside LHS
	}
	Serial.println("---------");

	for (int x = 0; x < 360; x +=STEP)
	{
		Serial.println(cardinalSin(x, 20, x + 19));  // test for just inside RHS
	}
	Serial.println("---------");

	for (int x = 0; x < 360; x +=STEP)
	{
		Serial.println(cardinalSin(x, 20, x + 20));  // test for on the line
	}
	Serial.println("---------");
#endif


	
	TaskHandle_t foo;
	xTaskCreate(loop1, "loop1", 4096, NULL, 5, &foo);

	//xTaskCreatePinnedToCore(loop1, "loop1", 4096, NULL, 1, NULL, 0);

	//xTaskCreatePinnedToCore(loop2, "loop2", 4096, NULL, 1, NULL, 1);

	 
}


