
/*
 * CPETLite v0.1
 * July 2020
 */
// #include <time.h>

#include "sfm3019.h"
#include <ArduinoBLE.h>

#define BLE_UUID_SERVICE "C20F3661-94B1-41DA-ACB2-0AE53E5E7591"
// when using string characteristic, the length must be fixed, something like: "fl:123;hb:234;co:52232"
#define BLE_UUID_DATA    "C5DCF5FE-3E1A-4ACA-8D33-009443A4143C"
#define BLE_UUID_CMD     "88ADA8E5-9DA7-4A42-A382-B1ECF59B329F"

const uint16_t ERROR_BLE_INIT = 400;

const uint16_t ERROR_HEART_INIT = 400;

const uint16_t ERROR_FLOW_DRIVER_VER = 410;
const uint16_t ERROR_FLOW_GEN_RESET = 420;
const uint16_t ERROR_FLOW_PROBE = 430;
const uint16_t ERROR_FLOW_PRODUCT_ID = 440;
const uint16_t ERROR_FLOW_START_MEASUREMENT = 450;
const uint16_t ERROR_FLOW_MEASUREMENT = 460;
const uint16_t ERROR_FLOW_CONVERSION = 470;

const uint16_t ERROR_CO2_INIT = 400;
const uint16_t ERROR_CO2_MEASUREMENT = 460;


bool bleError = 0;

bool enableFlow = true;
bool flowInitialized = false;
uint16_t flowError = 0;

bool enableCO2 = true;
bool co2Initialized = false;
uint16_t co2Error = 0;

bool enableHeart = true;
bool heartInitialized = false;
uint16_t heartError = 0;

BLEService cpetLiteService(BLE_UUID_SERVICE);
BLEStringCharacteristic dataChar(BLE_UUID_DATA, BLERead | BLENotify, 20);
BLECharacteristic cmdChar(BLE_UUID_CMD, BLERead | BLEWrite, 20);

// ble string buffer
char dataBuffer[50] = {0, };

unsigned char BleCommand = 0b00000000;

// co2
// Nano 33 BLE comes with a proper h/w serial on P0 and P1 via Serial1
#define co2Serial Serial1
float co2_signal = 0.0;


int co2 = 0;
double multiplier = 10;// 1 for 2% =20000 PPM, 10 for 20% = 200,000 PPM
uint8_t co2Buffer[25];
uint8_t ind = 0;
uint8_t start_index = 0;

void fill_buffer();
void format_output();


// flow meter
SfmConfig sfm3019;


void initSensorsIfNeeded();
void initBLE();

// heart sensor
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>

int bpm = 0;
const int OUTPUT_TYPE = SERIAL_PLOTTER;
const int PULSE_INPUT = A0;
const int PULSE_BLINK = 13;    // Pin 13 is the on-board LED
const int PULSE_FADE = 5;
const int PULSE_THRESHOLD = 550;   // Adjust this number to avoid noise when idle
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;
PulseSensorPlayground pulseSensor;

BLEDevice central;


void setup() {
  Serial.begin(9600);
  delay(500);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    bleError = ERROR_BLE_INIT;
    while (1);
  }

  initBLE();
}

void initSensorsIfNeeded() {
  if (enableFlow && !flowInitialized) {
    initFlowSensor();
  }

  if (enableCO2 && !co2Initialized) {
    initCo2Sensor();
  }

  if (enableHeart && !heartInitialized) {
    initHeartSensor();
  }
}

void initCo2Sensor() {
  // http://co2meters.com/Documentation/Manuals/Manual_GC_0024_0025_0026_Revised8.pdf

  co2Serial.begin(9600); // Start serial communications with sensor
  
  co2Serial.println("K 0");  // Set Command mode, see User Manual pg 13
  co2Serial.println("M 6"); // send Mode for Z and z outputs
  // "Z xxxxx z xxxxx" (CO2 filtered and unfiltered)

  co2Serial.println("A 4"); // set digital filter value, see User Manual pg 35

  delay(6000); // warmup time, see User Manual pg 35

  co2Serial.println("K 2");  // 1 = streaming, 2 = polling mode

  int retry_count = 0;
  for (;;) {
    if (co2Serial.available()) {
      while(co2Serial.available()) {
        co2Serial.read();
      }
      break;

    } else {
      retry_count += 1;
      delay(1);
    }

    // after 10 seconds abort initializing
    if (retry_count > 10000) {
      Serial.println("CO2 Sensor failed to initialize");
      co2Error = ERROR_CO2_INIT;
      return;
    }
  }

  Serial.println("CO2 sensor ready");

  co2Initialized = true;
}

void initHeartSensor() {
  // Serial.begin(115200);

  // Configure the PulseSensor manager.
  pulseSensor.analogInput(PULSE_INPUT);
  // pulseSensor.blinkOnPulse(PULSE_BLINK);
  // pulseSensor.fadeOnPulse(PULSE_FADE);

  // pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(PULSE_THRESHOLD);

  // Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try changing USE_PS_INTERRUPT to false.
    */

    heartError = ERROR_HEART_INIT;
    return;
  }

  Serial.println("Heartbeat sensor ready");

  heartInitialized = true;
}

void initFlowSensor() {
  const char* driver_version = sfm_common_get_driver_version();
  if (driver_version) {
    Serial.print("\nSFM driver version: ");
    Serial.println(driver_version);
  } else {
    Serial.println("fatal: Getting driver version failed");
    flowError = ERROR_FLOW_DRIVER_VER;
    return;
  }
  sensirion_i2c_init();

  /* Reset all I2C devices */
  flowError = sensirion_i2c_general_call_reset();
  if (flowError) {
    Serial.println("General call reset failed");
    flowError = ERROR_FLOW_GEN_RESET;
    return;
  }

  /* Wait for the SFM3019 to initialize */
  sensirion_sleep_usec(SFM3019_SOFT_RESET_TIME_US);

  bool probe = sfm3019_probe();
  uint8_t retry_count = 0;
  while (probe && retry_count < 10) {
    retry_count += 1;
    sensirion_sleep_usec(100000);
  }

  if (probe) {
    Serial.println("SFM sensor probing failed");
    flowError = ERROR_FLOW_PROBE;
    return;
  }

  uint32_t product_number = 0;
  uint8_t serial_number[8] = {};
  flowError = sfm_common_read_product_identifier(SFM3019_I2C_ADDRESS,
      &product_number, &serial_number);
  if (flowError) {
    Serial.println("Failed to read product identifier");
    flowError = ERROR_FLOW_PRODUCT_ID;
    return;
  } else {
    Serial.print("product: 0x");
    Serial.print(product_number, HEX);
    Serial.print(" serial: 0x");
    for (size_t i = 0; i < 8; ++i) {
      Serial.print(serial_number[i], HEX);
    }
    Serial.println("");
  }

  sfm3019 = sfm3019_create();

  flowError = sfm_common_start_continuous_measurement(&sfm3019, SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_AIR);

  if (flowError) {
    Serial.println("Failed to start measurement");
    flowError = ERROR_FLOW_START_MEASUREMENT;
    return;
  }

  /* Wait for the first measurement to be available. Wait for
   * SFM3019_MEASUREMENT_WARM_UP_TIME_US instead for more reliable results */
  sensirion_sleep_usec(SFM3019_MEASUREMENT_INITIALIZATION_TIME_US);

  Serial.println("Flow sensor ready");

  flowInitialized = true;
}


int16_t flow_raw;
float flow = 0.0;

int16_t temperature_raw;
float temperature = 0.0;

uint16_t status;

void readFlowSensor() {
  flowError = sfm_common_read_measurement_raw(&sfm3019, &flow_raw, &temperature_raw, &status);

  if (flowError) {
    Serial.println("Error while reading measurement.");
    flowError = ERROR_FLOW_MEASUREMENT;
    return;
  }

  flowError = sfm_common_convert_flow_float(&sfm3019, flow_raw, &flow);
  if (flowError) {
    Serial.println("Error while converting flow");
    flowError = ERROR_FLOW_CONVERSION;
    return;
  }
}

void readCo2Sensor(void) {
  fill_buffer();  // function call that reacds CO2 sensor and fills buffer
  if (co2Error) {
    return;
  }

  //Serial.print("Buffer contains: ");
  //for(int j=0; j<ind; j++)Serial.print(co2Buffer[j],HEX);
  //start_index = 0;
  //format_output();
  //Serial.print(" Raw PPM        ");

  start_index = 8;  // In ASCII buffer, filtered value is offset from raw by 8 bytes
  format_output();
  //Serial.println(" Filtered PPM\n\n");
}

void readHeartSensor() {
  if (!pulseSensor.sawNewSample()) {
    return;
  }
  
  if (!pulseSensor.sawStartOfBeat()) {
    return;
  }

  bpm = pulseSensor.getBeatsPerMinute();
}

void fill_buffer(void) {
  co2Serial.println("Z");  // fetch latest measurement

  memset(co2Buffer, 0, sizeof(co2Buffer));

  int retry_count = 0;

  // fill buffer with sensor ascii data
  ind = 0;
  while (co2Buffer[ind-1] != 0x0A) {  // Read sensor and fill buffer up to 0XA = CR
    if (co2Serial.available()) {
      co2Buffer[ind] = co2Serial.read();
      ind++;
    } else {
      retry_count += 1;
      delay(1);
    }

    // after 2 seconds, abort reading
    if (retry_count > 2000) {
      co2Error = ERROR_CO2_MEASUREMENT;
      return;
    }
  }

  // co2Buffer now filled with sensor ascii data
  // ind contains the number of characters loaded into buffer up to 0xA =  CR
  ind = ind -2; // decrement buffer to exactly match last numerical character
}


// read buffer, extract 6 ASCII chars, convert to PPM
void format_output() {
  co2 = co2Buffer[15-start_index]-0x30;
  co2 = co2+((co2Buffer[14-start_index]-0x30)*10);
  co2 += (co2Buffer[13-start_index]-0x30)*100;
  co2 += (co2Buffer[12-start_index]-0x30)*1000;
  co2 += (co2Buffer[11-start_index]-0x30)*10000;
  
  co2_signal = co2*multiplier;
}

void initBLE() {
  pinMode(LED_BUILTIN, OUTPUT);

  BLE.setLocalName("CPETLite");
  BLE.setAdvertisedService(cpetLiteService);

  cpetLiteService.addCharacteristic(dataChar);
  cpetLiteService.addCharacteristic(cmdChar);

  BLE.addService(cpetLiteService);

  BLE.advertise();
  
  Serial.println("BLE ready");
}

uint16_t counter = 0;

String flowVal;
String co2Val;

void loop() {

  central = BLE.central();
  if (!central) {
    Serial.println("Waiting for BLE central");
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    return;
  }

  if (!central.connected()) {
    Serial.println("Waiting for BLE connections");
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    return;
  }

  if (cmdChar.written()) {
    cmdChar.readValue(&BleCommand, 1);

    Serial.print("Received command: ");
    for(uint8_t mask = 0x80; mask != 0; mask >>= 1) {
      if(BleCommand & mask) {
        Serial.print("1");
      } else {
        Serial.print("0");
      }
    }
    Serial.println("\n");

    if (BleCommand & (1 << 0)) {
      enableHeart = true;
    } else {
      enableHeart = false;
    }

    if (BleCommand & (1 << 1)) {
      enableFlow = true;
    } else {
      enableFlow = false;
    }

    if (BleCommand & (1 << 2)) {
      enableCO2 = true;
    } else {
      enableCO2 = false;
    }

    heartError = 0;
    flowError = 0;
    co2Error = 0;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  memset(dataBuffer, 0, sizeof(dataBuffer));

  initSensorsIfNeeded();
  if (heartError || flowError || co2Error) {
    sprintf(dataBuffer, "err:%d,%d,%d", heartError, flowError, co2Error);
    dataChar.writeValue(dataBuffer);

    delay(1000);
    return;
  }

  // heart sensor needs to be read at fastest interval possible
  // so we don't miss a spike, keep a short history of these values
  // to calculate BPM when emitting data
  if (enableHeart) {
    readHeartSensor();
  }

  // every 0.25 seconds, read co2 and flow and emit all to phone
  if (counter % 250 == 0) {
    if (enableCO2) {
      readCo2Sensor();
    }
    co2Val = String(co2_signal);

    if (enableFlow) {
      readFlowSensor();
    }
    flowVal = String(flow);

    sprintf(dataBuffer, "%d,%s,%s", bpm, flowVal.c_str(), co2Val.c_str());
    dataChar.writeValue(dataBuffer);
  }

  counter += 1;
  delay(1);
}
