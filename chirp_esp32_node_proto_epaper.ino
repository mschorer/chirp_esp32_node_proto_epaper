/**
 * 
 * FOR THIS EXAMPLE TO WORK, YOU MUST INSTALL THE "LoRaWAN_ESP32" LIBRARY USING
 * THE LIBRARY MANAGER IN THE ARDUINO IDE.
 * 
 * This code will send a two-byte LoRaWAN message every 15 minutes. The first
 * byte is a simple 8-bit counter, the second is the ESP32 chip temperature
 * directly after waking up from its 15 minute sleep in degrees celsius + 100.
 *
 * If your NVS partition does not have stored TTN / LoRaWAN provisioning
 * information in it yet, you will be prompted for them on the serial port and
 * they will be stored for subsequent use.
 *
 * See https://github.com/ropg/LoRaWAN_ESP32
*/

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <RadioLib.h>
#include <LoRaWAN_ESP32.h>

#include <SensorData.h>

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #include "driver/temperature_sensor.h"
#else
  #include "driver/temp_sensor.h"
#endif

#define LL_OFF  0
#define LL_LOW  1
#define LL_MED  2
#define LL_HGH  2

#define LOGLEVEL  LL_LOW

//#define LOGLEVEL  LL_LOW

#define RADIOLIB_DEBUG_PROTOCOL 1
#define HELTEC_V3FIX
//#define ADC_SCALE 0.00403532794741887
//#define ADC_SCALE ( 1/238.7)
//#define ADC_SCALE 0.00433
#define ADC_SCALE 0.00175

#define LED_OFF 0
#define LED_LOW 3
#define LED_MID 15
#define LED_BRIGHT  50

#define FALSE 0
#define TRUE 1

#define XSHUT_PIN 47

#define VBAT_CTRL GPIO_NUM_19
#define VBAT_ADC  GPIO_NUM_20

//#include <heltec_unofficial.h>
#include <heltec-eink-modules.h>

//-------------------------------------------------------

#include <stdarg.h>
#include <stdio.h>

//---- LoRaWan ----

uint8_t fPort = 1;

LoRaWANNode* node = 0;
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO_1, PIN_LORA_NRST, PIN_LORA_BUSY);
LCMEN2R13EFC1 display;    // V1.0
// QYEG0213RWS800 display;

// Pause between sends in seconds, so this is every 15 minutes. (Delay will be
// longer if regulatory or TTN Fair Use Policy requires it.)
#define MINIMUM_DELAY 300

#define RELAX_DELAY 120
#define RELAX_FB    3

#define ALERT_DELAY 60
#define ALERT_FB    3

#define EMERG_DELAY 30
#define EMERG_FB    10

#define AUTO_FALLBACK 16

#define LORA_DUTY_CYCLE 0     // for legal limit: 0, forTTN: 1250

// Pause between sends in seconds, so this is every 15 minutes. (Delay will be
// longer if regulatory or TTN Fair Use Policy requires it.)
#define MINIMUM_DELAY 300

// you can also retrieve additional information about an uplink or 
// downlink by passing a reference to LoRaWANEvent_t structure
LoRaWANEvent_t uplinkDetails;
LoRaWANEvent_t downlinkDetails;

//RTC_DATA_ATTR uint8_t count = 0;

//----------------------------------------------

//#include "Fonts/FreeMonoBold12pt7b.h"
#include "Fonts/FreeMono9pt7b.h"
#include "Fonts/FreeMonoBold9pt7b.h"

// These are just for the sketch's reference - the library does not require them
const Color DEFAULT_TEXT_COLOR = BLACK;
const GFXfont *DEFAULT_FONT = &FreeMono9pt7b;
const GFXfont *DEFAULT_BOLD = &FreeMonoBold9pt7b;
const int textbox_padding = 5;                  // How much padding inside textBox() - used by installGuide()
FullBounds f = display.bounds.full;             // Quick access to dimensions of full-screen

//-------------------------------------------------------

#define DOWN_SIZE 220

SensorData up( 128);
SensorData down( DOWN_SIZE);

LoraNode *localNode;

//---- RTC persistent storage ----

#define HIST_DEPTH 120

struct climHist {
  LoraBME280 bme[ HIST_DEPTH];
  int8_t current;
  int8_t last;
};

struct tofNode {
  LoraNode node;
  LoraToF tof;
  LoraFill fill;
  LoraBME280 bme;
  climHist hist;
};

struct dataMem {
  tofNode well;
  tofNode ibc;
  uint16_t loraDelay;
  uint16_t loraDlyFb;
};

#define RTC_BLOCK_OFFSET 64
struct rtcMem {
  struct dataMem data;
  long crc32;
};

RTC_DATA_ATTR rtcMem cache;

void restoreData( rtcMem *pData) {
  uint32_t crcNow = calculateCRC32( (uint8_t *)&pData->data, sizeof(pData->data));

    Serial.printf( "restore [%x] [%x]\n", crcNow, pData->crc32);

   if ( pData->crc32 != crcNow) {
    initData( &pData->data);
   }
}

void persistData( rtcMem *pData) {
  pData->crc32 = calculateCRC32( (uint8_t *)&pData->data, sizeof(pData->data));
  Serial.printf( "backup [%x]\n", pData->crc32);
}

void initData( dataMem* data) {
  uint8_t i;

  data->ibc.tof.dist = -1;
  data->ibc.fill.percent = 0;
  data->ibc.bme.temp = 250;
  data->ibc.bme.hmd = 500;
  data->ibc.bme.prs = 9500;
/*
  for ( i=0; i < HIST_DEPTH; i++) {
    data->ibc.hist.bme[i].temp = 250;
    data->ibc.hist.bme[i].hmd = 500;
    data->ibc.hist.bme[i].prs = 9500;
  }
*/
  data->ibc.hist.current = -1;
  data->ibc.hist.last = 0;

  data->well.tof.dist = -1;
  data->well.fill.percent = 0;
  data->well.bme.temp = 250;
  data->well.bme.hmd = 500;
  data->well.bme.prs = 9500;
/*
  for ( i=0; i < HIST_DEPTH; i++) {
    data->well.hist.bme[i].temp = 250;
    data->well.hist.bme[i].hmd = 500;        //(uint16_t) random( 500, 1000);
    data->well.hist.bme[i].prs = 9500; //(uint16_t) random( 9500, 11000);;
  }
*/
  data->well.hist.current = -1;
  data->well.hist.last = 0;

  data->loraDelay = MINIMUM_DELAY;
  data->loraDlyFb = 0;
}

//-------------------------------------------------------

//---- display code ----

void sprintAt( int16_t x, int16_t y, char *fmt, ...) {
  char gstring[48];
  va_list args;

  va_start(args, fmt);

  vsprintf( gstring, fmt, args);

  display.setCursor( x, y);   // Text cursor
  display.print( gstring);
}

void drawVBar( uint8_t x,  uint8_t y, uint8_t w, uint8_t h, int8_t headroom) {
  if ( headroom >= 0) {
    display.drawRect( x, y, w, headroom, DEFAULT_TEXT_COLOR);
    display.fillRect( x, y+headroom, w, h-headroom, DEFAULT_TEXT_COLOR);
  } else {
    display.drawRect( x, y, w, h, DEFAULT_TEXT_COLOR);
    display.drawLine( x, y, x+w, y+h, DEFAULT_TEXT_COLOR);
  }
}

void drawHBar( uint8_t x,  uint8_t y, uint8_t w, uint8_t h, int8_t fill) {
  display.fillRect( x, y, fill, h, DEFAULT_TEXT_COLOR);
  display.drawRect( x+fill, y, w-fill, h, DEFAULT_TEXT_COLOR);
}

void drawFill( LoraFill *ibc, LoraFill *well) {
  uint8_t y = 150;
  display.setFont( NULL);
  
  int8_t headroom = -1;
  if ( ibc) {
    if ( ibc->percent > 0) {
      headroom = 100 - min( 100, ibc->percent / 100);
    }
  }
  drawVBar( 11, y, 40, 100, headroom);
  //sprintAt( 10, y-8, "IBC");

  headroom = -1;
  if ( well) {
    if ( well->percent > 0) {
      headroom = 100 - min( 100, well->percent / 100);
    }
  }
  drawVBar( 66, y, 54, 100, headroom);
  //sprintAt( 70, y-8, "Well");
}

void drawLevels( LoraToF *ibc, LoraToF *well) {
  uint8_t y = 150;
  display.setFont( NULL);
/*  
  uint8_t headroom = 100;
  if ( ibc) {
    if ( ibc->dist > 0) {
      headroom = min( 100, ibc->dist / 12);
    }
  }
  drawVBar( 8, y, 40, 100, headroom);
*/
  sprintAt( 14, y-8, "% 4i", ibc->dist);
/*
  headroom = 100;
  if ( well) {
    if ( well->dist > 0) {
      headroom = min( 100, well->dist / 44);
    }
  }
  drawVBar( 62, y, 55, 100, headroom);
*/
  sprintAt( 78, y-8, "% 4i", well->dist);
}

void drawBME( uint8_t x, uint8_t y, LoraBME280 *bme, char* name) {
  //display.drawRect( x, y, 60, 60, DEFAULT_TEXT_COLOR);
  uint8_t decimal = bme->temp % 10;

  display.setFont( NULL); //DEFAULT_BOLD);
  sprintAt( x+3, y+3, name);

  //display.setFont( NULL);
  if ( bme) {
    display.setFont( DEFAULT_FONT);
    sprintAt( x+8, y+24, "% 4.0f", (float)bme->temp / 10);
    sprintAt( x+8, y+40, "%%%3.0f", (float)bme->hmd / 10);
    sprintAt( x+8, y+56, "% 4.0f", (float)bme->prs / 10);

    display.setFont( NULL);
    sprintAt( x+53, y+14, "%i", decimal);
  }
  //drawHBar( 72, y+1, 50, 5, batLevel);
}

void drawHist( uint8_t x, uint8_t y, climHist *hist, char* name) {
  int8_t i, idx, start;
  uint16_t minP=12000, maxP=100, mid;
  float scale,yp;

  display.drawRect( x, y, 122, 52, DEFAULT_TEXT_COLOR);

  display.setFont( NULL); //DEFAULT_FONT);
  sprintAt( x+2, y+2, name);

  Serial.printf("hist % 3i - % 3i\n", hist->current, hist->last);

  if ( hist->current >= 0) {
    idx = hist->last;
    for( i=0; i<HIST_DEPTH; i++) {
      if ( hist->bme[idx].prs > maxP) maxP = hist->bme[idx].prs;
      if ( hist->bme[idx].prs < minP) minP = hist->bme[idx].prs;

      if ( i % 2) display.drawPixel( i+1, y+51-hist->bme[idx].hmd/20, DEFAULT_TEXT_COLOR);

      if ( idx == hist->current)
        break;

      idx = (idx+1) % HIST_DEPTH;
    }

    idx = hist->last;
    scale = (maxP == minP) ? 1 : (50.0 / (float)(maxP - minP));
    scale = min( (float) 1.0, scale);
    mid = (maxP == minP) ? minP : (maxP + minP) / 2;

    Serial.printf("PRS %f / %i / %i\n[", scale, minP, maxP);
    for( i=0; i<HIST_DEPTH; i++) {
      yp = (float)(hist->bme[idx].prs-mid)*scale;
      display.drawPixel( i+1, y+25-yp, DEFAULT_TEXT_COLOR);

      Serial.printf("%i ", (int16_t) yp);
      if ( i%12 == 11) Serial.println();

      if ( idx == hist->current)
        break;

      idx = (idx+1) % HIST_DEPTH;
    }
    Serial.println("]");
  }
}

void drawNode( uint8_t x, uint8_t y, LoraNode *node, char* name) {
  uint8_t batLevel = 0;
  char status = '.';

  display.setFont( NULL);
  if ( node) {
    if ( node->vbat > 0) {
      batLevel = min( 50, (node->vbat -325) /2);
      sprintAt( 40, y, "%.2fV", (float)node->vbat / 100);
    } else {
        sprintAt( 40, y, "-.--V" );
    }
    switch( node->meta & STS_MMASK) {
      case STS_EMERG: status = '*'; break;
      case STS_ALERT: status = '+'; break;
      case STS_RELAX: status = '-'; break;
    }
    sprintAt( 30, y, "%c", status);
  }
  drawHBar( 72, y+1, 50, 5, batLevel);
  sprintAt( 0, y, name);
}

//-------------------------------------------------------

void heltec_ve(bool state) {
  if (state) Platform::VExtOn();
  else Platform::VExtOff();
}

void heltec_led(int percent) {
  if (percent) {
    // enable LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    // disable LED
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void heltec_deep_sleep(int seconds = 0) {
  
  //radio.sleep();
  Platform::prepareToSleep();

  if (seconds) {
      esp_sleep_enable_timer_wakeup((uint64_t) 1000 * 1000 * seconds);  // Microseconds
  }

  // Sleep now
  esp_deep_sleep_start();
}

float heltec_temperature() {
  float result = 0;

  // If temperature for given n below this value,
  // then this is the best measurement we have.
  int cutoffs[5] = { -30, -10, 80, 100, 2500 };
  
  #if ESP_ARDUINO_VERSION_MAJOR >= 3

    int range_start[] = { -40, -30, -10,  20,  50 };
    int range_end[]   = {  20,  50,  80, 100, 125 };
    temperature_sensor_handle_t temp_handle = NULL;
    for (int n = 0; n < 5; n++) {
      temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(range_start[n], range_end[n]);
      ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
      ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
      ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &result));
      ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
      ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_handle));
      if (result <= cutoffs[n]) break;
    }

  #else

    // We start with the coldest range, because those temps get spoiled 
    // the quickest by heat of processor waking up. 
    temp_sensor_dac_offset_t offsets[5] = {
      TSENS_DAC_L4,   // (-40°C ~  20°C, err <3°C)
      TSENS_DAC_L3,   // (-30°C ~  50°C, err <2°C)
      TSENS_DAC_L2,   // (-10°C ~  80°C, err <1°C)
      TSENS_DAC_L1,   // ( 20°C ~ 100°C, err <2°C)
      TSENS_DAC_L0    // ( 50°C ~ 125°C, err <3°C)
    };
    for (int n = 0; n < 5; n++) {
      temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
      temp_sensor.dac_offset = offsets[n];
      temp_sensor_set_config(temp_sensor);
      temp_sensor_start();
      temp_sensor_read_celsius(&result);
      temp_sensor_stop();
      if (result <= cutoffs[n]) break;
    }

  #endif

  return result;
}

float heltec_vbat() {
  pinMode(VBAT_CTRL, OUTPUT);
  digitalWrite(VBAT_CTRL, HIGH);
  delay(5);
  float vbat = analogRead(VBAT_ADC) / 537.7;
  // pulled up, no need to drive it
  pinMode(VBAT_CTRL, INPUT);

  return vbat;
}

const float min_voltage = 3.04;
const float max_voltage = 4.26;
const uint8_t scaled_voltage[100] = {
  254, 242, 230, 227, 223, 219, 215, 213, 210, 207,
  206, 202, 202, 200, 200, 199, 198, 198, 196, 196,
  195, 195, 194, 192, 191, 188, 187, 185, 185, 185,
  183, 182, 180, 179, 178, 175, 175, 174, 172, 171,
  170, 169, 168, 166, 166, 165, 165, 164, 161, 161,
  159, 158, 158, 157, 156, 155, 151, 148, 147, 145,
  143, 142, 140, 140, 136, 132, 130, 130, 129, 126,
  125, 124, 121, 120, 118, 116, 115, 114, 112, 112,
  110, 110, 108, 106, 106, 104, 102, 101, 99, 97,
  94, 90, 81, 80, 76, 73, 66, 52, 32, 7,
};

int heltec_battery_percent(float vbat = -1) {
  if (vbat == -1) {
    vbat = heltec_vbat();
  }
  for (int n = 0; n < sizeof(scaled_voltage); n++) {
    float step = (max_voltage - min_voltage) / 256;
    if (vbat > min_voltage + (step * scaled_voltage[n])) {
      return 100 - n;
    }
  }
  return 0;
}

void heltec_display_power(bool on) {
  heltec_ve( on);
}

void heltec_setup() {
  Serial.begin(115200);

  heltec_display_power(true);

  //display.landscape();
  display.setTextWrap(false); 
  display.setTextColor(DEFAULT_TEXT_COLOR);
  display.setFont( NULL); //DEFAULT_FONT);
  display.setTextSize(0);

  //display.drawRect( 200, 1, 48, 120, DEFAULT_TEXT_COLOR);

  // no longer needed as fixed in lib
  // gpio_hold_dis((gpio_num_t) PIN_LORA_NSS);
}

void heltec_loop() {
}

//-------------------------------------------------------

//---- Code ----

void setup() {
  byte i;

  //--
  restoreData( &cache);

  //---- start ----
  heltec_setup();
  heltec_led(LED_LOW);

  //--  
  localNode = (LoraNode *) up.addSensor( SensorData::SensorType::NODE, sizeof( LoraNode));

  // Obtain directly after deep sleep
  // May or may not reflect room temperature, sort of. 
  float temp = heltec_temperature();
  //Serial.printf("Temperature: %.1f C\n", temp);

  analogReadResolution(12);
  
  pinMode(VBAT_CTRL, OUTPUT);
  digitalWrite(VBAT_CTRL, LOW);
  delay(5);
  int vint = analogRead(VBAT_ADC);
  float vbat = vint * ADC_SCALE;
  // pulled up, no need to drive it
  pinMode(VBAT_CTRL, INPUT);

  //Serial.printf( "Bat: %i\n", vint);
  int vperc = heltec_battery_percent(vbat);
  //Serial.printf("%i%%\n", vperc);

  // 0 = external power source
  // 1 = lowest (empty battery)
  // 254 = highest (full battery)
  // 255 = unable to measure
  uint8_t battLevel = 0;
  battLevel = int((float)vperc * 2.54);

  Serial.printf("%.1fC / %.2fV / %3i%% / %i\n", temp, vbat, vperc, battLevel);

  //sprintAt( 0, 0, "Node\n%.1fC %.2fV %3i%%", temp, vbat, vperc);
  // prepare data -----------------------------

  localNode->cputemp = temp * 10;
  localNode->vbat = vbat * 100;

  Serial.printf( "esp temp/vbat[ %i %i ]\n", localNode->cputemp, localNode->vbat);

  drawNode( 0, 0, localNode, "View");

  //---- run ----

  //display.display();
  //display.update();

  persistData( &cache);

  // send (and receive data) ------------------------------------

  // initialize radio
  Serial.print("init ... ");

  radio.reset();
  int16_t state = radio.begin();

  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("error [%x] - postpone.\n", state);
    Serial.println(stateDecode( state));
    goToSleep();
  }

  node = persist.manage(&radio);

  if (!node->isActivated()) {
    Serial.println("RESTORE FAILED!");
    goToSleep();
  }

  Serial.println( "session restored!");
  
  Serial.print("[LoRaWAN] DevAddr: ");
  Serial.println((unsigned long)node->getDevAddr(), HEX);

  Serial.printf( "rssi %.1f snr %.1f frq %.1f\n", radio.getRSSI(), radio.getSNR(), radio.getFrequencyError());
  //heltec_led(LED_MID);

  // If we're still here, it means we joined, and we can send something

  // Enable the ADR algorithm (on by default which is preferable)
  node->setADR(true);
  // Set a datarate to start off with
  node->setDatarate(5);
  // Manages uplink intervals to the TTN Fair Use Policy
  node->setDutyCycle(true, LORA_DUTY_CYCLE);   // zero sets max val by law, TTN 30s/24h: [1250]ms/1h);
  // Update dwell time limits - 400ms is the limit for the US
  node->setDwellTime(true, 400);  // zero sets max val by law, 400);

  node->setDeviceStatus(battLevel);

  //-----------------------------------------------------------------
  
  heltec_led(LED_OFF);
  Serial.print( "Sending ... ");

  // Retrieve the last uplink frame counter
  uint32_t fCntUp = node->getFCntUp();
  Serial.println( "FCNTUP "+String( fCntUp));

  uint32_t mask = 0x01;
  uint32_t bitX = 0;
  for( uint8_t i = 0; i < 32; i++) {
    if (fCntUp & mask) display.fillRect( bitX, 77, 4, 3, DEFAULT_TEXT_COLOR);
    bitX += 4;
    mask <<= 1;
  }
  if (fCntUp & 0x40000000) display.fillRect( 120, 77, 1, 3, DEFAULT_TEXT_COLOR);
  if (fCntUp & 0x80000000) display.fillRect( 121, 77, 1, 3, DEFAULT_TEXT_COLOR);

  size_t downlinkSize = DOWN_SIZE;
  if(fCntUp == 1) {
    Serial.println(F("and requesting LinkCheck and DeviceTime"));
    node->sendMacCommandReq(RADIOLIB_LORAWAN_MAC_LINK_CHECK);
    node->sendMacCommandReq(RADIOLIB_LORAWAN_MAC_DEVICE_TIME);

    state = node->sendReceive( up.buffer, up.eod, fPort, down.buffer, &downlinkSize, true, &uplinkDetails, &downlinkDetails);
  } else {
    state = node->sendReceive( up.buffer, up.eod, fPort, down.buffer, &downlinkSize, false, &uplinkDetails, &downlinkDetails);
  }
  down.eod = (uint8_t) downlinkSize;

  if(state == RADIOLIB_ERR_NONE) {
    Serial.println("OK.");
    //heltec_led(LED_BRIGHT);
  } else if (state > 0) {
  // Check if a downlink was received 
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
    Serial.printf("OK. Downlink data! [%i] #%i\n", state, down.eod);

    // Did we get a downlink with data for us
    if ( down.eod > 0) {
      Serial.println(F("Downlink data: "));
      arrayDump( down.buffer, down.eod);

      handleDownlink( &down);

      persistData( &cache);
    } else {
      Serial.println(F("<MAC commands only>"));
    }

    //dumpDownlinkStats( state);

    //heltec_led(LED_BRIGHT);
  } else {
    Serial.printf("Error %d\n", state);
    //heltec_led(LED_LOW);
  }
  
  //-------
  heltec_led(LED_LOW);

  display.drawLine( 0, 79, 122, 79, DEFAULT_TEXT_COLOR);
  display.drawLine( 61, 80, 61, 252, DEFAULT_TEXT_COLOR);

  drawNode( 0, 8, &cache.data.ibc.node, "IBC");
  drawBME(  0, 80, &cache.data.ibc.bme, "IBC");

  drawNode( 0, 16,&cache.data.well.node, "Well");
  drawBME( 62, 80,&cache.data.well.bme, "Well");

  drawHist( 0, 25, &cache.data.well.hist, "Well");

  drawLevels( &cache.data.ibc.tof, &cache.data.well.tof);
  drawFill( &cache.data.ibc.fill, &cache.data.well.fill);

  heltec_led(LED_OFF);

  //display.display();
  display.update();

  goToSleep();    // Does not return, program starts over next round
}

void handleDownlink( SensorData *down) {
  uint8_t index = 0;
  uint8_t i = 0;
  uint8_t rmt, newMeta;;
  LoraToF* tof = 0;
  LoraFill* fill = 0;
  LoraNode* node = 0;
  LoraBME280 *bme = 0;
  LoraSensor* remoteStatus = 0;
  LoraDS18B20* onewire = 0;

  Serial.printf( "Downlink  [%i]:\n", down->eod);

  while( index < down->eod) {
    switch( down->buffer[ index]) {
      case SensorData::SensorType::STATUS:
          remoteStatus = (LoraSensor *) &down->buffer[index];
          index += sizeof( LoraSensor);

          localNode->meta = remoteStatus->meta;
          Serial.printf( "status [%i]\n", localNode->meta);
      break;

      case SensorData::SensorType::ID:
        index += sizeof( LoraID);
        Serial.println( "ID");
      break;

      case SensorData::SensorType::NODE:
        node = (LoraNode *) &down->buffer[index];
        index += sizeof( LoraNode);
        rmt = (node->meta & 0x1f);

        Serial.printf( "Node %i: %.2fV %.1fC\n", rmt, (float) node->vbat /100, (float)node->cputemp /10);
        //sprintAt( 0, rmt*50, "remote %i\n%.2fV %.1fC\n", rmt, (float) node->vbat /100, (float)node->cputemp /10);

        switch( rmt) {
          case 0:
            memcpy( &cache.data.ibc.node, node, sizeof( LoraNode));
          break;

          case 1:
            memcpy( &cache.data.well.node, node, sizeof( LoraNode));
          break;

          default:
            ;
        }
      break;

      case SensorData::SensorType::DS18B20:
        onewire = (LoraDS18B20 *) &down->buffer[index];
        index += sizeof( LoraDS18B20);
        rmt = (onewire->meta & 0x1f);
        Serial.printf( "1wire%i\n", rmt);
      break;

      case SensorData::SensorType::BME280:
        bme = (LoraBME280 *) &down->buffer[index];
        index += sizeof( LoraBME280);
        rmt = (bme->meta & 0x1f);

        switch( rmt) {
          case 0:
            memcpy( &cache.data.ibc.bme, bme, sizeof( LoraBME280));

            if ( cache.data.ibc.hist.current < 0) {
              cache.data.ibc.hist.current = 0;
            } else {
              cache.data.ibc.hist.current = (cache.data.ibc.hist.current+1) % HIST_DEPTH;
              if ( cache.data.ibc.hist.current == cache.data.ibc.hist.last)
                cache.data.ibc.hist.last = (cache.data.ibc.hist.last+1) % HIST_DEPTH;
            }

            memcpy( &cache.data.ibc.hist.bme[ cache.data.ibc.hist.current], bme, sizeof( LoraBME280));
            //cache.data.ibc.tof.dist = 550;
          break;

          case 1:
            memcpy( &cache.data.well.bme, bme, sizeof( LoraBME280));

            if ( cache.data.well.hist.current < 0) {
              cache.data.well.hist.current = 0;
            } else {
              cache.data.well.hist.current = (cache.data.well.hist.current+1) % HIST_DEPTH;
              if ( cache.data.well.hist.current == cache.data.well.hist.last)
                cache.data.well.hist.last = (cache.data.well.hist.last+1) % HIST_DEPTH;
            }

            memcpy( &cache.data.well.hist.bme[ cache.data.well.hist.current], bme, sizeof( LoraBME280));
            //cache.data.well.tof.dist = 2123;
          break;

          default:
            ;
        }

        Serial.printf( "BME  %i: %i C\n", rmt, bme->temp);
        Serial.printf( "      : %i h\n", bme->hmd);
        Serial.printf( "      : %i p\n", bme->prs);
        i++;
      break;
      
      case SensorData::SensorType::TOF:
        tof = (LoraToF *) &down->buffer[index];
        index += sizeof( LoraToF);
        rmt = (tof->meta & 0x1f);

        switch( rmt) {
          case 0:
            memcpy( &cache.data.ibc.tof, tof, sizeof( LoraToF));
            //cache.data.ibc.tof.dist = 550;
          break;

          case 1:
            memcpy( &cache.data.well.tof, tof, sizeof( LoraToF));
            //cache.data.well.tof.dist = 2123;
          break;

          default:
            ;
        }

        Serial.printf( "ToF  %i: %imm\n", rmt, tof->dist);
        //Serial.printf( "     %i: %imm\n", 0, cache.data.ibc.tof.dist);
        //Serial.printf( "     %i: %imm\n", 1, cache.data.well.tof.dist);
        //Serial.println( "TOF parsed");
        //Serial.println( tof->dist);
        i++;
      break;

      case SensorData::SensorType::GPS:
        index += sizeof( LoraGps);
        Serial.println( "  gps");
      break;

      case SensorData::SensorType::FILL:
        fill = (LoraFill *) &down->buffer[index];
        index += sizeof( LoraFill);
        rmt = (fill->meta & 0x1f);

        switch( rmt) {
          case 0:
            memcpy( &cache.data.ibc.fill, fill, sizeof( LoraFill));
          break;

          case 1:
            memcpy( &cache.data.well.fill, fill, sizeof( LoraFill));
          break;

          default:
            ;
        }

        Serial.printf( "Fill %i: %i%%\n", rmt, fill->percent/100);
        //Serial.printf( "     %i: %i%%\n", 0, cache.data.ibc.fill.percent);
        //Serial.printf( "     %i: %i%%\n", 1, cache.data.well.fill.percent);
        //Serial.println( "TOF parsed");
        //Serial.println( tof->dist);
        i++;
      break;

      default:
        Serial.printf( "xTra data. @%i [%02x] [%c]\n", index, down->buffer[ index], down->buffer[ index]);
        index = down->eod;
    }
  }
}

//-------------------------------------------------------------------

void loop() {
  heltec_loop();
}

//-------------------------------------------------------------------

void goToSleep() {
  uint32_t interval = 0;
  char* mState[] = { "DCARE", "RELAX", "ALERT", "EMERG"};

  // -------------------

  uint8_t meta = localNode->meta & STS_MMASK;
  switch( meta) {
    case STS_EMERG:
      cache.data.loraDelay = EMERG_DELAY;
      cache.data.loraDlyFb = EMERG_FB;
    break;

    case STS_ALERT:
      cache.data.loraDelay = ALERT_DELAY;
      cache.data.loraDlyFb = ALERT_FB;
    break;

    case STS_RELAX:
      cache.data.loraDelay = RELAX_DELAY;
      cache.data.loraDlyFb = RELAX_FB;
    break;

    case STS_DCARE:
    default:
      cache.data.loraDelay = MINIMUM_DELAY;
      cache.data.loraDlyFb = 0;
  }

  if ( cache.data.loraDlyFb > 0) {
    cache.data.loraDlyFb--;
  } else {
    cache.data.loraDelay = MINIMUM_DELAY;
  }

  // allows recall of the session after deepsleep
  if ( node) {
    persist.saveSession(node);

    // Calculate minimum duty cycle delay (per FUP & law!)
    interval = node->timeUntilUplink();
  }

  // And then pick it or our MINIMUM_DELAY, whichever is greater
  uint32_t delayMs = max(interval, (uint32_t) cache.data.loraDelay * 1000);

  Serial.printf("DeepSleep for [%s] [%d]-[%dk @%d]\n", mState[ STS_MSHIFT(meta)], interval, cache.data.loraDelay, cache.data.loraDlyFb);
  Serial.println( "----");

  delayMs = blinkMode( meta, delayMs);

  // and off to bed we go
  heltec_deep_sleep(delayMs/1000);
}

uint32_t blinkMode( uint8_t meta, uint32_t delayMs) {
  uint8_t loops = 0;
  //Serial.printf( "metamode [%02x]\n", meta);

  while( loops < 64) {
    //Serial.printf("(%d)", loops);

    switch( loops % 8) {
      case 6:
        if ( meta <= STS_EMERG) break;
      case 4:
        if ( meta <= STS_ALERT) break;
      case 2:
        if ( meta <= STS_RELAX) break;
      case 0:
        //Serial.printf( "[%i]", loops);
        heltec_led( LED_LOW);
        break;

      default:
        heltec_led(LED_OFF);
        //Serial.printf( "#%i#", loops);
    }

    loops++;
    delayMs -= 100;
    delay( 100);
  }

  return delayMs;
}

// result code to text - these are error codes that can be raised when using LoRaWAN
// however, RadioLib has many more - see https://jgromes.github.io/RadioLib/group__status__codes.html for a complete list
String stateDecode(const int16_t result) {
  switch (result) {
  case RADIOLIB_ERR_NONE:
    return "ERR_NONE";
  case RADIOLIB_ERR_CHIP_NOT_FOUND:
    return "ERR_CHIP_NOT_FOUND";
  case RADIOLIB_ERR_PACKET_TOO_LONG:
    return "ERR_PACKET_TOO_LONG";
  case RADIOLIB_ERR_RX_TIMEOUT:
    return "ERR_RX_TIMEOUT";
  case RADIOLIB_ERR_CRC_MISMATCH:
    return "ERR_CRC_MISMATCH";
  case RADIOLIB_ERR_INVALID_BANDWIDTH:
    return "ERR_INVALID_BANDWIDTH";
  case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
    return "ERR_INVALID_SPREADING_FACTOR";
  case RADIOLIB_ERR_INVALID_CODING_RATE:
    return "ERR_INVALID_CODING_RATE";
  case RADIOLIB_ERR_INVALID_FREQUENCY:
    return "ERR_INVALID_FREQUENCY";
  case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
    return "ERR_INVALID_OUTPUT_POWER";
  case RADIOLIB_ERR_NETWORK_NOT_JOINED:
	  return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
  case RADIOLIB_ERR_DOWNLINK_MALFORMED:
    return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
  case RADIOLIB_ERR_INVALID_REVISION:
    return "RADIOLIB_ERR_INVALID_REVISION";
  case RADIOLIB_ERR_INVALID_PORT:
    return "RADIOLIB_ERR_INVALID_PORT";
  case RADIOLIB_ERR_NO_RX_WINDOW:
    return "RADIOLIB_ERR_NO_RX_WINDOW";
  case RADIOLIB_ERR_INVALID_CID:
    return "RADIOLIB_ERR_INVALID_CID";
  case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
    return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
  case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
    return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
  case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
    return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
  case RADIOLIB_ERR_JOIN_NONCE_INVALID:
    return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
  /*
  case RADIOLIB_ERR_MIC_MISMATCH:
    return "RADIOLIB_ERR_MIC_MISMATCH";
  case RADIOLIB_ERR_MULTICAST_FCNT_INVALID:
    return "RADIOLIB_ERR_MULTICAST_FCNT_INVALID";
  */
  case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
    return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
  case RADIOLIB_ERR_CHECKSUM_MISMATCH:
    return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
  case RADIOLIB_ERR_NO_JOIN_ACCEPT:
    return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
  case RADIOLIB_LORAWAN_SESSION_RESTORED:
    return "RADIOLIB_LORAWAN_SESSION_RESTORED";
  case RADIOLIB_LORAWAN_NEW_SESSION:
    return "RADIOLIB_LORAWAN_NEW_SESSION";
  case RADIOLIB_ERR_NONCES_DISCARDED:
    return "RADIOLIB_ERR_NONCES_DISCARDED";
  case RADIOLIB_ERR_SESSION_DISCARDED:
    return "RADIOLIB_ERR_SESSION_DISCARDED";

  default:
    Serial.printf( "ERR: [%i]\n", result);
    return "---";
  }
  return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

void dumpDownlinkStats(const int16_t state) {
  // print RSSI (Received Signal Strength Indicator)
  Serial.print(F("[LoRaWAN] RSSI:\t\t"));
  Serial.print(radio.getRSSI());
  Serial.println(F(" dBm"));

  // print SNR (Signal-to-Noise Ratio)
  Serial.print(F("[LoRaWAN] SNR:\t\t"));
  Serial.print(radio.getSNR());
  Serial.println(F(" dB"));

  // print extra information about the event
  Serial.println(F("[LoRaWAN] Event information:"));
  Serial.print(F("[LoRaWAN] Confirmed:\t"));
  Serial.println(downlinkDetails.confirmed);
  Serial.print(F("[LoRaWAN] Confirming:\t"));
  Serial.println(downlinkDetails.confirming);
  Serial.print(F("[LoRaWAN] Datarate:\t"));
  Serial.println(downlinkDetails.datarate);
  Serial.print(F("[LoRaWAN] Frequency:\t"));
  Serial.print(downlinkDetails.freq, 3);
  Serial.println(F(" MHz"));
  Serial.print(F("[LoRaWAN] Frame count:\t"));
  Serial.println(downlinkDetails.fCnt);
  Serial.print(F("[LoRaWAN] Port:\t\t"));
  Serial.println(downlinkDetails.fPort);
  Serial.print(F("[LoRaWAN] Time-on-air: \t"));
  Serial.print(node->getLastToA());
  Serial.println(F(" ms"));
  Serial.print(F("[LoRaWAN] Rx window: \t"));
  Serial.println(state);

  uint8_t margin = 0;
  uint8_t gwCnt = 0;
  if(node->getMacLinkCheckAns(&margin, &gwCnt) == RADIOLIB_ERR_NONE) {
    Serial.print(F("[LoRaWAN] LinkCheck margin:\t"));
    Serial.println(margin);
    Serial.print(F("[LoRaWAN] LinkCheck count:\t"));
    Serial.println(gwCnt);
  }

  uint32_t networkTime = 0;
  uint8_t fracSecond = 0;
  if(node->getMacDeviceTimeAns(&networkTime, &fracSecond, true) == RADIOLIB_ERR_NONE) {
    Serial.print(F("[LoRaWAN] DeviceTime Unix:\t"));
    Serial.println(networkTime);
    Serial.print(F("[LoRaWAN] DeviceTime second:\t1/"));
    Serial.println(fracSecond);
  }
}
// helper function to display any issues
void debug(bool failed, const __FlashStringHelper* message, int state, bool halt) {
  if(failed) {
    Serial.print(message);
    Serial.print(" - ");
    Serial.print(stateDecode(state));
    Serial.print(" (");
    Serial.print(state);
    Serial.println(")");
    while(halt) { delay(1); }
  }
}

// helper function to display a byte array
void arrayDump(uint8_t *buffer, uint16_t len) {
  for(uint16_t c = 0; c < len; c++) {
    char b = buffer[c];
    if(b < 0x10) { Serial.print('0'); }
    Serial.print(b, HEX);
    Serial.print(' ');

    if ( c % 8 == 7) Serial.println();
  }
  Serial.println();
}

byte crcCheck( byte* buf, byte maxIdx) {
  byte crc = OneWire::crc8( buf, maxIdx);
  if ( crc != buf[maxIdx]) {
#ifdef CRC_DEBUG
    Serial.print("CRC err ");
    Serial.print( maxIdx, DEC);
    Serial.print(" ");
    Serial.print( crc, HEX);
    Serial.print( " != ");
    Serial.println( buf[maxIdx], HEX);
#endif
    return TRUE;
  }

  return FALSE;
}

uint32_t calculateCRC32(const uint8_t *data, size_t len) {
#ifdef CRC_DEBUG
  Serial.print( "crc32 @");
  Serial.print( (uint32_t) data);
  Serial.print( " #");
  Serial.print( len);
#endif
  uint32_t crc = 0xffffffff;
  while (len--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
#ifdef CRC_DEBUG
  Serial.print( " = ");
  Serial.print( crc,  16);
  Serial.println("");
#endif

  return crc;
}