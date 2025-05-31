/*
    Description: This case is used to test CORE2 OR CORE2 FOR AWS.
    Please note the selection of your device on line 15 of the programs!
*/
#ifdef ARDUINO_M5STACK_Core2

static bool bIsAWScore = 0;  // Please select your device
                  // Core2 = 1,Core2_AWS = 0;

#include <string>
#include <cstring>
#include <ArduinoECCX08.h>
#include <M5Core2.h>
#include <driver/i2s.h>

#include "FastLED.h"
#include "Fonts/EVA_11px.h"
#include "Fonts/EVA_20px.h"
#include "WiFi.h"
#include "Wire.h"
//#include "fft.h"
//#include "line3D.h"

#define LEDS_PIN 25
#define LEDS_NUM 10
CRGB ledsBuff[LEDS_NUM]; 

ECCX08Class myECCX08(Wire1, 0x35);

extern const unsigned char CoverImage[21301];
extern const unsigned char clockImage[18401];
extern const unsigned char CoreMainImage[87169];
extern const unsigned char batPowerImage[13769];
extern const unsigned char touchImage[12262];
extern const unsigned char SDCardImage[14835];
extern const unsigned char imageMenu[14900];
extern const unsigned char previewR[120264];
extern const unsigned char wifiSacnImage[28123];
extern const unsigned char TimerAppImage[59165];
extern const unsigned char SettingAppImage[50771];
extern const unsigned char bibiSig[8820];

extern const unsigned char image_rect_0006[1394];
extern const unsigned char image_rect_0005[1394];
extern const unsigned char image_rect_0004[1394];
extern const unsigned char image_rect_0003[1394];
extern const unsigned char image_rect_0002[1394];
extern const unsigned char image_rect_0001[1394];

extern const unsigned char image_DigNumber_0000_0[504];
extern const unsigned char image_DigNumber_0001_1[504];
extern const unsigned char image_DigNumber_0002_2[504];
extern const unsigned char image_DigNumber_0003_3[504];
extern const unsigned char image_DigNumber_0004_4[504];
extern const unsigned char image_DigNumber_0005_5[504];
extern const unsigned char image_DigNumber_0006_6[504];
extern const unsigned char image_DigNumber_0007_7[504];
extern const unsigned char image_DigNumber_0008_8[504];
extern const unsigned char image_DigNumber_0009_9[504];
extern const unsigned char image_DigNumber_0010_10[504];

extern const unsigned char image_DigNumber_35px_0000_0[315];
extern const unsigned char image_DigNumber_35px_0001_1[315];
extern const unsigned char image_DigNumber_35px_0002_2[315];
extern const unsigned char image_DigNumber_35px_0003_3[315];
extern const unsigned char image_DigNumber_35px_0004_4[315];
extern const unsigned char image_DigNumber_35px_0005_5[315];
extern const unsigned char image_DigNumber_35px_0006_6[315];
extern const unsigned char image_DigNumber_35px_0007_7[315];
extern const unsigned char image_DigNumber_35px_0008_8[315];
extern const unsigned char image_DigNumber_35px_0009_9[315];
extern const unsigned char image_DigNumber_35px_0010_10[315];

extern const unsigned char image_rect320_20_0001[3200];
extern const unsigned char image_rect320_20_0002[3200];
extern const unsigned char image_rect320_20_0003[3200];
extern const unsigned char image_rect320_20_0004[3200];
extern const unsigned char image_rect320_20_0005[3200];
extern const unsigned char image_rect320_20_0006[3200];
extern const unsigned char image_rect320_20_0007[3200];

extern const unsigned char image_number8x7_01[35];
extern const unsigned char image_number8x7_02[35];
extern const unsigned char image_number8x7_03[35];
extern const unsigned char image_number8x7_04[35];
extern const unsigned char image_number8x7_05[35];
extern const unsigned char image_number8x7_06[35];
extern const unsigned char image_number8x7_07[35];
extern const unsigned char image_number8x7_08[35];
extern const unsigned char image_number8x7_09[35];
extern const unsigned char image_number8x7_10[35];
extern const unsigned char image_number8x7_11[35];
extern const unsigned char image_number8x7_12[35];
extern const unsigned char image_number8x7_13[35];
extern const unsigned char image_number8x7_14[35];
extern const unsigned char image_number8x7_15[35];
extern const unsigned char image_number8x7_16[35];

extern const unsigned char image_chaging_0001[149];
extern const unsigned char image_chaging_0002[149];
extern const unsigned char image_chaging_0003[149];
extern const unsigned char image_chaging_0004[149];
extern const unsigned char image_chaging_0005[149];
extern const unsigned char image_chaging_0006[149];
extern const unsigned char image_chaging_0007[149];

extern const unsigned char image_Sysinit_0000s_0000_L1[875];
extern const unsigned char image_Sysinit_0000s_0001_L2[875];
extern const unsigned char image_Sysinit_0001s_0000_R1[875];
extern const unsigned char image_Sysinit_0001s_0001_R2[875];

extern const unsigned char image_TouchFish_0001[1672];
extern const unsigned char image_TouchFish_0002[1672];
extern const unsigned char image_TouchFish_0003[1672];
extern const unsigned char image_TouchFish_0004[1672];
extern const unsigned char image_TouchFish_0005[1672];
extern const unsigned char image_TouchFish_0006[1672];

//===============================================================
uint8_t *rectptrBuff[6] = {
    (uint8_t *)image_rect_0001, (uint8_t *)image_rect_0002,
    (uint8_t *)image_rect_0003, (uint8_t *)image_rect_0004,
    (uint8_t *)image_rect_0005, (uint8_t *)image_rect_0006,
};

uint8_t *rect320ptrBuff[7] = {
    (uint8_t *)image_rect320_20_0001, (uint8_t *)image_rect320_20_0002,
    (uint8_t *)image_rect320_20_0003, (uint8_t *)image_rect320_20_0004,
    (uint8_t *)image_rect320_20_0005, (uint8_t *)image_rect320_20_0006,
    (uint8_t *)image_rect320_20_0007,
};

uint8_t *DigNumber[11] = {
    (uint8_t *)image_DigNumber_0000_0,  (uint8_t *)image_DigNumber_0001_1,
    (uint8_t *)image_DigNumber_0002_2,  (uint8_t *)image_DigNumber_0003_3,
    (uint8_t *)image_DigNumber_0004_4,  (uint8_t *)image_DigNumber_0005_5,
    (uint8_t *)image_DigNumber_0006_6,  (uint8_t *)image_DigNumber_0007_7,
    (uint8_t *)image_DigNumber_0008_8,  (uint8_t *)image_DigNumber_0009_9,
    (uint8_t *)image_DigNumber_0010_10,
};

uint8_t *DigNumber_35px[11] = {
    (uint8_t *)image_DigNumber_35px_0000_0,
    (uint8_t *)image_DigNumber_35px_0001_1,
    (uint8_t *)image_DigNumber_35px_0002_2,
    (uint8_t *)image_DigNumber_35px_0003_3,
    (uint8_t *)image_DigNumber_35px_0004_4,
    (uint8_t *)image_DigNumber_35px_0005_5,
    (uint8_t *)image_DigNumber_35px_0006_6,
    (uint8_t *)image_DigNumber_35px_0007_7,
    (uint8_t *)image_DigNumber_35px_0008_8,
    (uint8_t *)image_DigNumber_35px_0009_9,
    (uint8_t *)image_DigNumber_35px_0010_10,
};

uint8_t *Number_7x10px[16] = {
    (uint8_t *)image_number8x7_01,  // 0
    (uint8_t *)image_number8x7_02,  // 1
    (uint8_t *)image_number8x7_03,  // 2
    (uint8_t *)image_number8x7_04,  // 3
    (uint8_t *)image_number8x7_05,  // 4
    (uint8_t *)image_number8x7_06,  // 5
    (uint8_t *)image_number8x7_07,  // 6
    (uint8_t *)image_number8x7_08,  // 7
    (uint8_t *)image_number8x7_09,  // 8
    (uint8_t *)image_number8x7_10,  // 9
    (uint8_t *)image_number8x7_11,  // V
    (uint8_t *)image_number8x7_12,  // X
    (uint8_t *)image_number8x7_13,  // Y
    (uint8_t *)image_number8x7_14,  // :
    (uint8_t *)image_number8x7_15,  // CHARGING
    (uint8_t *)image_number8x7_16,  // G
};

uint8_t *batRect[7] = {
    (uint8_t *)image_chaging_0001, (uint8_t *)image_chaging_0002,
    (uint8_t *)image_chaging_0003, (uint8_t *)image_chaging_0004,
    (uint8_t *)image_chaging_0005, (uint8_t *)image_chaging_0006,
    (uint8_t *)image_chaging_0007,
};

uint8_t *TouchFishBuff[7] = {
    (uint8_t *)image_TouchFish_0001, (uint8_t *)image_TouchFish_0002,
    (uint8_t *)image_TouchFish_0003, (uint8_t *)image_TouchFish_0004,
    (uint8_t *)image_TouchFish_0005, (uint8_t *)image_TouchFish_0006,
};
//===============================================================
#define CONFIG_I2S_BCK_PIN     12
#define CONFIG_I2S_LRCK_PIN    0
#define CONFIG_I2S_DATA_PIN    2
#define CONFIG_I2S_DATA_IN_PIN 34

#define SPAKER_I2S_NUMBER I2S_NUM_0

#define MODE_MIC 0
#define MODE_SPK 1

//===============================================================

typedef struct i2cbIsAWScore {
    i2cbIsAWScore() {
        Name    = "";
        addr    = 0;
        nextPtr = nullptr;
    };
    String Name;
    uint8_t addr;
    struct i2cbIsAWScore *nextPtr;
} i2cbIsAWScore_t;

i2cbIsAWScore_t i2cParentptr;

typedef enum {
    kPOWER_EXTERNAL = 0,
    kPOWER_INTERNAL,
    kPOWER_MAX
} system_power_t;

struct systemState {
    RTC_TimeTypeDef Rtctime;
    system_power_t power = kPOWER_MAX;
    uint16_t batCount    = 0;

    uint8_t batVoltageBuff[15];
    uint8_t batVoltageWriteptr    = 11;
    uint8_t batVoltageReadptr     = 0;
    uint16_t batVoltageWriteCount = 0;

    bool touchState  = false;
    bool SDCardState = false;

    uint16_t SDCardscaneCount = 0;

    HotZone_t *App1Zone[6];
    HotZone_t *MPU6886;

    bool soundFlag = true;

} sysState;

static QueueHandle_t fftvalueQueue = nullptr;
static QueueHandle_t i2sstateQueue = nullptr;

typedef struct {
    uint8_t state;
    void *audioPtr;
    uint32_t audioSize;
} i2sQueueMsg_t;

//===============================================================

// 40-80-119   130-160-200  230-270-310
HotZone_t touchBtn0(10, 241, 120, 280);
HotZone_t touchBtn1(130, 241, 200, 280);
HotZone_t touchBtn2(230, 241, 310, 280);

#define FAILD_COLOR 255, 35, 35
#define SUCCE_COLOR 255, 255, 255

void setCheckState(int number, bool state, bool flush = false);
//===============================================================

bool InitI2SSpakerOrMic(int mode, unsigned rate = 44100) {
    esp_err_t err = ESP_OK;

	Serial.println(__FUNCTION__); delay (3000);
	Serial.println("NOPE NOT HAPPENING"); return false;
	

	
    i2s_driver_uninstall(SPAKER_I2S_NUMBER);
    i2s_config_t i2s_config = {
        .mode        = (i2s_mode_t)(I2S_MODE_MASTER),
        .sample_rate = rate,
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,

		// Set the format of the communication.
		#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
	        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  
		#else                                   
    	    .communication_format = I2S_COMM_FORMAT_I2S,
		#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count    = 2,
        .dma_buf_len      = 128,
    };
    if (mode == MODE_MIC) {
        i2s_config.mode =
            (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    } else {
        i2s_config.mode     = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.use_apll = false;
        i2s_config.tx_desc_auto_clear = true;
    }

    Serial.println("Init i2s_driver_install"); delay(3000);

    err += i2s_driver_install(SPAKER_I2S_NUMBER, &i2s_config, 0, NULL);
    i2s_pin_config_t tx_pin_config;
#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    tx_pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif
    tx_pin_config.bck_io_num   = CONFIG_I2S_BCK_PIN;
    tx_pin_config.ws_io_num    = CONFIG_I2S_LRCK_PIN;
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;
    tx_pin_config.data_in_num  = CONFIG_I2S_DATA_IN_PIN;

    Serial.println("Init i2s_set_pin"); delay(1000);
    err += i2s_set_pin(SPAKER_I2S_NUMBER, &tx_pin_config);

	Serial.println("Init i2s_set_clk"); delay(1000);
    err += i2s_set_clk(SPAKER_I2S_NUMBER, rate, I2S_BITS_PER_SAMPLE_16BIT,
                       I2S_CHANNEL_MONO);

    return true;
}
//===============================================================

void sysErrorSkip() {
    // M5.Axp.SetLDOEnable(3,false);

    HotZone_t toucZone(0, 0, 320, 280);

    while (1) {
        if (M5.Touch.ispressed()) {
            TouchPoint_t point = M5.Touch.getPressPoint();
            if (toucZone.inHotZone(point)) {
                break;
            }
        }
        delay(10);
    }
}
//===============================================================

void addI2cbIsAWScore(String name, uint8_t addr) {
    i2cbIsAWScore_t *lastptr = &i2cParentptr;

    while (lastptr->nextPtr != nullptr) {
        lastptr = lastptr->nextPtr;
    }

    i2cbIsAWScore_t *ptr = (i2cbIsAWScore_t *)calloc(1, sizeof(i2cbIsAWScore_t));
    ptr->Name        = name;
    ptr->addr        = addr;
    ptr->nextPtr     = nullptr;
    lastptr->nextPtr = ptr;
}
//===============================================================

int scani2caddr() {
	Serial.println("Scanning all i2c addresses\n");
    for (int i = 0; i < 128; i++) {
        Wire1.beginTransmission(i);
        if (Wire1.endTransmission() == 0) {
            Serial.printf("device at %02X\n", i);
        }
    }
	Serial.println("Scanning all i2c addresses done\n\n");
    return 0;
}
//===============================================================
int checkPsram() {
    uint8_t *testbuff = (uint8_t *)ps_calloc(100 * 1024, sizeof(uint8_t));
    if (testbuff == nullptr) {
        Serial.printf("PSRAM malloc failed\n");
        sysErrorSkip();
        return -1;
    } else {
        Serial.printf("PSRAM malloc Successful\n");
    }
    delay(100);

    for (size_t i = 0; i < 102400; i++) {
        testbuff[i] = 0xA5;
        if (testbuff[i] != 0xA5) {
            Serial.printf("PSRAM read failed\n");
            sysErrorSkip();
            return -1;
        }
    }
    Serial.printf("PSRAM W&R Successful\n");
    return 0;
}
//===============================================================

int checkI2cAddr() {
    uint8_t count        = 0;
    i2cbIsAWScore_t *lastptr = &i2cParentptr;
    do {
        lastptr = lastptr->nextPtr;
        Serial.printf("Addr:0x%02X - Name:%s\r\n", lastptr->addr,
                      lastptr->Name.c_str());

        if (lastptr->addr == 0x35) {
            // wakeup
            Wire1.setClock(100000u);
            Wire1.beginTransmission(0x00);
            Wire1.endTransmission();
            delayMicroseconds(1500);
            Wire1.beginTransmission(lastptr->addr);
        } else {
            Wire1.beginTransmission(lastptr->addr);
        }
        if (Wire1.endTransmission() == ESP_OK) {
            String log = "I2C " + lastptr->Name + " Found";
            Serial.println(log);
        } else {
            String log = "I2C " + lastptr->Name + " Find failed";
            Serial.println(log);
            sysErrorSkip();
        }
        delay(100);
        count++;
    } while (lastptr->nextPtr != nullptr);
    return 0;
}
//===============================================================

int checkIMUInit() {
    if (M5.IMU.Init() == 0) {
        Serial.printf("IMU Check Successful\n");
    } else {
        Serial.printf("IMU Check failed\n");
        sysErrorSkip();
    }
    return 0;
}
//===============================================================

int checkAETCC608AInit() {
    if (!myECCX08.begin()) {
        Serial.printf("AT608A Check failed\n");
        while (1) delay(100);
    } else {
        Serial.printf("AT608A Check Successful\n");
    }

    String serialNumber = myECCX08.serialNumber();

    Serial.print("ECCX08 Serial Number = ");
    Serial.println(serialNumber);
    Serial.println();

    myECCX08.end();
    Wire1.begin(21, 22, 100000UL);
    return 0;
}
//===============================================================

int checkSDCard() {
    sdcard_type_t Type = SD.cardType();

    if (Type == CARD_UNKNOWN || Type == CARD_NONE) {
        Serial.printf("SDCard Find failed\n");
		Serial.println("no SD card detected. use this 15 sec delay in event of fault\n");
		delay(15000);
        //sysErrorSkip();
    } else {
        Serial.printf("SDCard Found\n");
        Serial.printf("SDCard Type = %d \r\n", Type);
        Serial.printf("SDCard Size = %d \r\n",
                      (int)(SD.cardSize() / 1024 / 1024));
    }
    return 0;
}
//===============================================================
/*
void i2s_task(void *arg) {
    size_t bytes_written = 0;
    // Core2CovreSig[160908]
    i2s_write(SPAKER_I2S_NUMBER, previewR, 120264, &bytes_written,
              portMAX_DELAY);
    // delay(500);
    vTaskDelete(NULL);
}
*/
//===============================================================

void choosePower() {
    uint32_t color1 = 0, color2 = 0;
    uint16_t posy = 6;

    system_power_t PowerNow;

    PowerNow = (M5.Axp.isACIN()) ? kPOWER_EXTERNAL : kPOWER_INTERNAL;

    if (PowerNow == sysState.power) return;

    if (PowerNow == kPOWER_EXTERNAL) {
        color1 = 0x6c6c6c;
        color2 = 0xff9c00;
        posy   = 68;
    } else {
        color2 = 0x6c6c6c;
        color1 = 0xff9c00;
        posy   = 3;
    }

    sysState.power = PowerNow;
}
//===============================================================

void MPU6886Test() {
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    double theta = 0, last_theta = 0;
    double phi = 0, last_phi = 0;
    double alpha = 0.2;

    M5.IMU.getAccelData(&accX, &accY, &accZ);

    if ((accX < 1) && (accX > -1)) {
        theta = asin(-accX) * 57.295;
    }
    if (accZ != 0) {
        phi = atan(accY / accZ) * 57.295;
    }

    theta = alpha * theta + (1 - alpha) * last_theta;
    phi   = alpha * phi + (1 - alpha) * last_phi;

    last_theta = theta;
    last_phi   = phi;
}

//===============================================================
void clockSetup() {
    RTC_TimeTypeDef RTCTime;
    RTCTime.Hours   = 23;
    RTCTime.Minutes = 33;
    RTCTime.Seconds = 33;
    M5.Rtc.SetTime(&RTCTime);
}

void copyRTCtime() {
    M5.Rtc.GetTime(&sysState.Rtctime);
}


//---------------------------------------------------------

void smartDelay(unsigned long ms)
{
  delay(ms);
}

//===============================================================
#ifdef NEED_FFT
static void i2sMicroFFTtask(void *arg) {
    uint8_t FFTDataBuff[128];
    uint8_t FFTValueBuff[24];
    uint8_t *microRawData = (uint8_t *)calloc(2048, sizeof(uint8_t));
    size_t bytesread;
    int16_t *buffptr;
    double data = 0;
    float adc_data;
    uint16_t ydata;
    uint32_t subData;

    uint8_t state = MODE_MIC;
    i2sQueueMsg_t QueueMsg;
    while (1) {
        if (xQueueReceive(i2sstateQueue, &QueueMsg, (TickType_t)0) == pdTRUE) {
            // Serial.println("Queue Now");
            if (QueueMsg.state == MODE_MIC) {
                InitI2SSpakerOrMic(MODE_MIC);
                state = MODE_MIC;
            } else {
                // Serial.println("Spaker");
                // Serial.printf("Length:%d",QueueMsg.audioSize);
                InitI2SSpakerOrMic(MODE_SPK);
                size_t written = 0;
                i2s_write(SPAKER_I2S_NUMBER, (unsigned char *)QueueMsg.audioPtr,
                          QueueMsg.audioSize, &written, portMAX_DELAY);
                state = MODE_SPK;
            }
        } else if (state == MODE_MIC) {
            fft_config_t *real_fft_plan =
                fft_init(1024, FFT_REAL, FFT_FORWARD, NULL, NULL);
            i2s_read(I2S_NUM_0, (char *)microRawData, 2048, &bytesread,
                     (100 / portTICK_RATE_MS));
            buffptr = (int16_t *)microRawData;

            for (int count_n = 0; count_n < real_fft_plan->size; count_n++) {
                adc_data = (float)map(buffptr[count_n], INT16_MIN, INT16_MAX,
                                      -2000, 2000);
                real_fft_plan->input[count_n] = adc_data;
            }
            fft_execute(real_fft_plan);

            for (int count_n = 1; count_n < real_fft_plan->size / 4;
                 count_n++) {
                data = sqrt(real_fft_plan->output[2 * count_n] *
                                real_fft_plan->output[2 * count_n] +
                            real_fft_plan->output[2 * count_n + 1] *
                                real_fft_plan->output[2 * count_n + 1]);
                if ((count_n - 1) < 128) {
                    data                       = (data > 2000) ? 2000 : data;
                    ydata                      = map(data, 0, 2000, 0, 255);
                    FFTDataBuff[128 - count_n] = ydata;
                }
            }

            for (int count = 0; count < 24; count++) {
                subData = 0;
                for (int count_i = 0; count_i < 5; count_i++) {
                    subData += FFTDataBuff[count * 5 + count_i];
                }
                subData /= 5;
                FFTValueBuff[count] = map(subData, 0, 255, 0, 8);
            }
            xQueueSend(fftvalueQueue, (void *)&FFTValueBuff, 0);
            fft_destroy(real_fft_plan);
            // Serial.printf("mmp\r\n");
        } else {
            delay(10);
        }
    }
}

//===============================================================
void microPhoneSetup() {
    fftvalueQueue = xQueueCreate(5, 24 * sizeof(uint8_t));
    if (fftvalueQueue == 0) {
        return;
    }

    i2sstateQueue = xQueueCreate(5, sizeof(i2sQueueMsg_t));
    if (i2sstateQueue == 0) {
        return;
    }

    InitI2SSpakerOrMic(MODE_MIC);
    xTaskCreatePinnedToCore(i2sMicroFFTtask, "microPhoneTask", 4096, NULL, 3,
                            NULL, 0);

    DisFFTbuff.createSprite(143, 54);
}
//===============================================================

void MicroPhoneFFT() {
    uint8_t FFTValueBuff[24];
    xQueueReceive(fftvalueQueue, (void *)&FFTValueBuff, portMAX_DELAY);
    DisFFTbuff.fillRect(0, 0, 143, 54, DisFFTbuff.color565(0x33, 0x20, 0x00));
    uint32_t colorY = DisFFTbuff.color565(0xff, 0x9c, 0x00);
    uint32_t colorG = DisFFTbuff.color565(0x66, 0xff, 0x00);
    uint32_t colorRect;
    for (int x = 0; x < 24; x++) {
        for (int y = 0; y < 9; y++) {
            if (y < FFTValueBuff[23 - x]) {
                colorRect = colorY;
            } else if (y == FFTValueBuff[23 - x]) {
                colorRect = colorG;
            } else {
                continue;
            }
            DisFFTbuff.fillRect(x * 6, 54 - y * 6 - 5, 5, 5, colorRect);
        }
    }
    DisFFTbuff.pushSprite(170, 130);
}
#endif
//===============================================================

void batpowerFlush() {
    float batVoltage    = M5.Axp.GetBatVoltage();
	
    float batPercentage = (batVoltage < 3.2) ? 0 : (batVoltage - 3.2) * 100;
    int rectwidth       = 27 - 27 * (int)batPercentage / 100;
    uint8_t batVoltage1 = (uint16_t)batVoltage % 10;
    uint8_t batVoltage2 = (uint16_t)(batVoltage * 10) % 10;


    if (M5.Axp.isACIN()) {
        sysState.batCount++;
        sysState.batCount %= 7;
    }

    if (sysState.batVoltageWriteCount > 50) {
        sysState.batVoltageWriteCount = 0;
        sysState.batVoltageBuff[sysState.batVoltageWriteptr] =
            (batPercentage / 20) + 1;
        sysState.batVoltageWriteptr++;
        sysState.batVoltageReadptr++;

        sysState.batVoltageWriteptr %= 15;
        sysState.batVoltageReadptr %= 15;


        for (int i = 0; i < 11; i++) {
            int lim =
                sysState.batVoltageBuff[(sysState.batVoltageReadptr + i) % 15];
        }
    }
    sysState.batVoltageWriteCount++;

}

//===============================================================
void sdcardSetup() {
    sdcard_type_t Type = SD.cardType();

    if (Type == CARD_UNKNOWN || Type == CARD_NONE) {
        sysState.SDCardState = false;

    } else {
        sysState.SDCardState = true;
        uint64_t sdcardSize = SD.cardSize() * 10 / 1024 / 1024 / 1024;
        uint64_t sdcardFreeSize =
            (SD.cardSize() - SD.usedBytes()) * 10 / 1024 / 1024 / 1024;

        Serial.printf("SDCard Type = %d \r\n", Type);
        Serial.printf("SDCard Size = %d \r\n",
                      (int)(SD.cardSize() / 1024 / 1024));
    }
}
//===============================================================

void sdCardFlush() {
    sysState.SDCardscaneCount++;
    if (sysState.SDCardscaneCount < 50) {
        return;
    }
    sysState.SDCardscaneCount = 0;
    sdcard_type_t Type        = SD.cardType();

    if (Type == CARD_UNKNOWN || Type == CARD_NONE) {
        sysState.SDCardState = false;
    } else {
        sysState.SDCardState = true;

        uint64_t sdcardSize = SD.cardSize() * 10 / 1024 / 1024 / 1024;
        uint64_t sdcardFreeSize =
            (SD.cardSize() - SD.usedBytes()) * 10 / 1024 / 1024 / 1024;

         Serial.printf("SDCard Type = %d \r\n", Type);
         Serial.printf("SDCard Size = %d \r\n", (int)(SD.cardSize() / 1024 /
         1024));
    }
}

//===============================================================
void AppOut() {
    Serial.println("Hello App");
}

//===============================================================
void APPSleep() {
    delay(500);
    M5.Axp.SetSleep();
}

//===============================================================

void TFTTest() {
    HotZone Btn(0, 0, 320, 280);
    TouchPoint_t pos = M5.Touch.getPressPoint();

    int colorIndex           = 0;
    uint32_t colorList[4][2] = {{TFT_RED, TFT_RED},
                                {TFT_GREEN, TFT_GREEN},
                                {TFT_BLUE, TFT_BLUE},
                                {TFT_WHITE, TFT_BLACK}};

    M5.Lcd.fillRect(0, 0, 320, 240, colorList[colorIndex][1]);
    M5.Lcd.drawRect(0, 0, 320, 240, colorList[colorIndex][0]);

    i2sQueueMsg_t msg;

    msg.state     = MODE_SPK;
    msg.audioPtr  = (void *)bibiSig;
    msg.audioSize = 8820;

    bool pressed = true;
    while (1) {
        pos = M5.Touch.getPressPoint();

        if ((pos.x != -1) && (pressed == false)) {
            pressed = true;

            msg.state     = MODE_SPK;
            msg.audioPtr  = (void *)bibiSig;
            msg.audioSize = 8820;

            xQueueSend(i2sstateQueue, &msg, (TickType_t)portMAX_DELAY);
            colorIndex++;
            if (colorIndex == 4) break;

            M5.Lcd.fillRect(0, 0, 320, 240, colorList[colorIndex][1]);
            M5.Lcd.drawRect(0, 0, 320, 240, colorList[colorIndex][0]);
        } else if (pos.x == -1) {
            pressed = false;
        }
    }

}
//===============================================================

void setCheckState(int number, bool state, bool flush) {
    int posx = 0, posy = 0;
    uint8_t *ptr1 = nullptr, *ptr2 = nullptr;
    if (number >= 5) {
        posx = 166;
        posy = 154 - (number - 5) * 33;
        ptr1 = (uint8_t *)image_Sysinit_0000s_0000_L1;
        ptr2 = (uint8_t *)image_Sysinit_0000s_0001_L2;
    } else {
        posx = 44;
        posy = 22 + number * 33;
        ptr1 = (uint8_t *)image_Sysinit_0001s_0000_R1;
        ptr2 = (uint8_t *)image_Sysinit_0001s_0001_R2;
    }
    int32_t color = (state) ? 0x35ffae : 0xff0000;
    if (flush == false) return;
}

//===============================================================
static uint16_t imageCount = 0;
static uint16_t timecount  = 0;

// Display all the touch event on the M5Core2 screen.
// Press the A button to turn on/off E_MOVE event detection
// Press the B button to turn on/off long press (500mS) detection
// Press the C button to turn on/off key repeat (200mS)

TFT_eSprite disp(&M5.Lcd);

bool first_scroll = true;
bool show_move    = false;
bool long_press   = true;
bool key_repeat   = false;

// Defines gestures
Gesture swipeRight("Swipe Right", 80, DIR_RIGHT, 30, true);
Gesture swipeDown("Swipe Down", 60, DIR_DOWN, 30, true);
Gesture swipeLeft("Swipe Left", 80, DIR_LEFT, 30, true);
Gesture swipeUp("Swipe Up", 60, DIR_UP, 30, true);

// Use a scrollable sprite for output. Looks nice!
//
void setup_disp() {
    disp.createSprite(320, 240);
    disp.setScrollRect(0, 0, 320, 240);
    disp.fillSprite(BLACK);
    disp.setTextFont(2);
    disp.setTextSize(1);
    disp.setTextColor(WHITE, BLACK);
    disp.setCursor(0, 0);
    disp.pushSprite(0, 0);
}

// Lazy output routine. Just enough to do the job.
// str should fit on the screen (320 pixels) and not contain a \n
//
void output_info(const char* name, const char* info) {
    Serial.printf("%-15s: %s\n", name, info);
    if (disp.getCursorY() >= 220) {
        disp.scroll(0, first_scroll ? -3 : -16);
        first_scroll = false;
    }
    disp.printf("%s", name);
    disp.setCursor(110, disp.getCursorY());
    disp.printf("%s\n", info);
    if (disp.getCursorY() >= 220) {
        disp.setCursor(0, 220);
    }
    disp.pushSprite(0, 0);
}

// Let the user know what to do
//
void splash_screen() {
    output_info("", "Welcome to TouchView");
    output_info("Button A Sets", (show_move) ? "E_MOVE will be displayed"
                                             : "E_MOVE will be ignored");
    output_info("Button B Sets", (long_press) ? "Long Presses will be detected"
                                              : "Long Presses will be ignored");
    output_info("Button C Sets",
                (key_repeat) ? "Key Repeat Enabled" : "Key Repeat Disabled");
    output_info("To Use", "Touch the Screen");
}

void eventHandler(Event& e) {
    char buffer[32];
    sprintf(buffer, "%3d,%3d/%3d,%3d %3d %3d %3d", e.from.x, e.from.y, e.to.x,
            e.to.y, e.distance(), e.direction(), e.duration);
    output_info(e.typeName(), buffer);
}

void gestureHandler(Event& e) {
    output_info(e.typeName(), e.gesture->getName());
}

// Remove/replace event handlers, since list of evetns could change.
//
void setup_events() {
    M5.background.delHandlers();
    uint16_t events =
        (show_move) ? E_ALL
                    : (E_ALL - E_MOVE);  // Show all events, or everything but
                                         // E_MOVE? Controlled with A button.
    M5.background.longPressTime =
        (long_press) ? 500 : 0;  // Detect long presses (500mS) or not?
                                 // Controlled with B button.
    M5.background.repeatDelay =
        (key_repeat) ? 200 : 0;  // Repeat press events every 200mS or not?
                                 // Controlled with the C button.
    M5.background.addHandler(eventHandler, events);
}

// Gestues only need to be set up once. I'll protect against multiple
// initializations.
//
void setupGestures() {
    static bool done = false;
    if (!done) {
        done = true;
        swipeRight.addHandler(gestureHandler, E_GESTURE);
        swipeLeft.addHandler(gestureHandler, E_GESTURE);
        swipeUp.addHandler(gestureHandler, E_GESTURE);
        swipeDown.addHandler(gestureHandler, E_GESTURE);
    }
}

static void setup_LCD() 
{
    //M5.begin();
    
	scani2caddr();
    setup_disp();
    splash_screen();
    setup_events();
    setupGestures();
}

static void loop_LCD() {
	
    M5.update();
    if (M5.BtnA.wasPressed()) {
        show_move = !show_move;
        output_info("Button A", (show_move) ? "E_MOVE will be displayed"
                                            : "E_MOVE will be ignored");
        setup_events();
    }

    if (M5.BtnB.wasPressed()) {
        long_press = !long_press;
        output_info("Button B", (long_press) ? "Long Presses will be detected"
                                             : "Long Presses will be ignored");
        setup_events();
    }

    if (M5.BtnC.wasPressed()) {
        key_repeat = !key_repeat;
        output_info("Button C", (key_repeat) ? "Key Repeat Enabled"
                                             : "Key Repeat Disabled");
        setup_events();
    }
}


// Lazy output routine. Just enough to do the job.
// str should fit on the screen (320 pixels) and not contain a \n
//
void one_liner(const char* name) {
    if (disp.getCursorY() >= 220) {
        disp.scroll(0, first_scroll ? -3 : -16);
        first_scroll = false;
    }
    disp.printf("%s", name);
    disp.setCursor(110, disp.getCursorY());
    if (disp.getCursorY() >= 220) {
        disp.setCursor(0, 220);
    }
    disp.pushSprite(0, 0);
}


int  xprintf(uint8_t lineNo, const char *format, ...) 
{
	va_list args;
	va_start(args, format);
	char buffer[30];
	vsnprintf(buffer, sizeof(buffer)-1, format, args);

	// erase past background to black
	//display.setColor(BLACK);
	//display.fillRect(0, lineNo * char_height, display.getWidth(), char_height);
	//display.setColor(WHITE);
	
	//display.drawString(0, lineNo * char_height, buffer);

	one_liner(buffer);
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
	//display.setColor(BLACK);
	//display.fillRect(0, lineNo * char_height, display.getWidth(), char_height);
	
	//display.setColor(WHITE);
	//display.drawRect(0, (lineNo * char_height)+1 , display.getWidth(), char_height );
	
	//display.drawString(0, lineNo * char_height, buffer);
	
	one_liner(buffer);
	
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
	//display.setColor(WHITE);
	//display.fillRect(0, lineNo * char_height, display.getWidth(), char_height);
	//display.setColor(BLACK);
	
	//display.drawString(0, lineNo * char_height, buffer);
	one_liner(buffer);
	
	va_end(args);
	return 0;
}



//=============================================================
void setup_M5() {
    M5.begin(true, true, true, false);

    M5.Axp.SetCHGCurrent(AXP192::kCHG_100mA);

    addI2cbIsAWScore("Axp192", 0x34);
    addI2cbIsAWScore("CST Touch", 0x38);
    addI2cbIsAWScore("IMU6886", 0x68);
    addI2cbIsAWScore("BM8563", 0x51);

    if (bIsAWScore == 0) addI2cbIsAWScore("AT608A", 0x35);

    M5.Axp.SetLcdVoltage(2800);

    SD.begin();

    sysState.App1Zone[0] = new HotZone(197, 191, 241, 235, &APPSleep);

    M5.Axp.SetLcdVoltage(3300);

    M5.Axp.SetBusPowerMode(0);
    M5.Axp.SetCHGCurrent(AXP192::kCHG_190mA);

    M5.Axp.SetSpkEnable(sysState.soundFlag);
    // dwade InitI2SSpakerOrMic(MODE_SPK);
    //xTaskCreatePinnedToCore(i2s_task, "i2s_task", 4096, NULL, 3, NULL, 0);

    M5.Axp.SetVibration(true);
    Serial.printf("Motor Test\n");
    delay(150);
    M5.Axp.SetVibration(false);

    M5.Axp.SetLed(1);
    Serial.printf("LED Test\n");
    delay(100);
    M5.Axp.SetLed(0);

    FastLED.addLeds<SK6812, LEDS_PIN>(ledsBuff, LEDS_NUM);
    for (int i = 0; i < LEDS_NUM; i++) {
        ledsBuff[i].setRGB(2, 2, 2);
    }
    FastLED.show();

    Wire1.begin(21, 22, 100000UL);
    checkI2cAddr();
    if (bIsAWScore == 0) checkAETCC608AInit();

    checkPsram();
    checkIMUInit();
    checkSDCard();

    M5.Axp.SetLDOVoltage(3, 3300);
    M5.Axp.SetLed(1);


    clockSetup();
    sdcardSetup();
    choosePower();
	
	setup_LCD(); 
    sdCardFlush();

    MPU6886Test();
	
}



void loop_m5() {
    if (timecount >= 2) {
        timecount = 0;
        copyRTCtime();
        batpowerFlush();
    }
    timecount++;
    //MicroPhoneFFT();
	
	loop_LCD();
}

#endif // ARDUINO_M5STACK_Core2
 
