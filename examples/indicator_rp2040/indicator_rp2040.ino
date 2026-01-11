#include <Arduino.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2cScd4x.h>
#include <VOCGasIndexAlgorithm.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PacketSerial.h>
#include "AHT20.h"
#include <Multichannel_Gas_GMXXX.h>
#include "Seeed_HM330X.h"

#define DEBUG 0

#define VERSION "v1.0.1"

#define SENSECAP "\n\
   _____                      _________    ____         \n\
  / ___/___  ____  ________  / ____/   |  / __ \\       \n\
  \\__ \\/ _ \\/ __ \\/ ___/ _ \\/ /   / /| | / /_/ /   \n\
 ___/ /  __/ / / (__  )  __/ /___/ ___ |/ ____/         \n\
/____/\\___/_/ /_/____/\\___/\\____/_/  |_/_/           \n\
--------------------------------------------------------\n\
 Version: %s \n\
--------------------------------------------------------\n\
"

AHT20 AHT;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;
VOCGasIndexAlgorithm voc_algorithm;

PacketSerial myPacketSerial;

String SDDataString = "";

// NEW Grove I2C sensors
HM330X hm3301;
uint8_t hm_buf[29];
GAS_GMXXX<TwoWire> gas;

static bool hm3301_ready = false;
static bool gas_ready = false;


//Type of transfer packet

#define PKT_TYPE_SENSOR_SCD41_CO2 0XB2
#define PKT_TYPE_SENSOR_SHT41_TEMP 0XB3
#define PKT_TYPE_SENSOR_SHT41_HUMIDITY 0XB4
#define PKT_TYPE_SENSOR_TVOC_INDEX 0XB5
#define PKT_TYPE_SENSOR_PM1_0  0xB6
#define PKT_TYPE_SENSOR_PM2_5  0xB7
#define PKT_TYPE_SENSOR_PM10   0xB8
#define PKT_TYPE_SENSOR_GM102B 0xB9
#define PKT_TYPE_SENSOR_GM302B 0xBA
#define PKT_TYPE_SENSOR_GM502B 0xBB
#define PKT_TYPE_SENSOR_GM702B 0xBC
#define PKT_TYPE_SENSOR_EXT_TEMP     0xBD
#define PKT_TYPE_SENSOR_EXT_HUMIDITY 0xBE
#define PKT_TYPE_CMD_COLLECT_INTERVAL 0xA0
#define PKT_TYPE_CMD_BEEP_ON 0xA1
#define PKT_TYPE_CMD_SHUTDOWN 0xA3




// sensor data send to  esp32
void sensor_data_send(uint8_t type, float data) {
  uint8_t data_buf[32] = { 0 };
  int index = 0;

  data_buf[0] = type;
  index++;

  memcpy(&data_buf[1], &data, sizeof(float));
  index += sizeof(float);

  myPacketSerial.send(data_buf, index);

#if DEBUG
  Serial.printf("---> send len:%d, data: ", index);
  for (int i = 0; i < index; i++) {
    Serial.printf("0x%x ", data_buf[i]);
  }
  Serial.println("");
#endif
}


void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}


static inline bool i2c_ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static inline bool hm3301_checksum_ok(const uint8_t *data29) {
  // Simple checksum used in Seeed examples: sum of first 28 bytes equals byte 28
  uint8_t sum = 0;
  for (int i = 0; i < 28; i++) sum += data29[i];
  return (sum == data29[28]);
}

void sensor_power_on(void) {

  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}

void sensor_power_off(void) {

  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
}

float temperature = 0;
float humidity = 0;

uint16_t defaultCompenstaionRh = 0x8000;
uint16_t defaultCompenstaionT = 0x6666;


uint16_t compensationRh = defaultCompenstaionRh;
uint16_t compensationT = defaultCompenstaionT;


//aht20: extern temp / humidity sensor
void sensor_aht_init(void) {
  AHT.begin();
}

void sensor_aht_get(void) {

  float humi, temp;

  int ret = AHT.getSensor(&humi, &temp);
  if (ret)  // GET DATA OK
  {
    Serial.print("sensor aht20 (external): humidity: ");
    Serial.print(humi * 100);
    Serial.print("%\t temerature: ");
    Serial.println(temp);
    temperature = temp;
    humidity = humi * 100;
    compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(humidity * 65535 / 100);
  } else  // GET DATA FAIL
  {
    Serial.println("GET DATA FROM AHT20 FAIL");
    compensationRh = defaultCompenstaionRh;
    compensationT = defaultCompenstaionT;
  }

  SDDataString += "aht20_ext,";
  if (ret) {
    SDDataString += String(temperature);
    SDDataString += ',';
    SDDataString += String(humidity);
    SDDataString += ',';

    // external values use new packet types
    sensor_data_send(PKT_TYPE_SENSOR_EXT_TEMP, temperature);
    sensor_data_send(PKT_TYPE_SENSOR_EXT_HUMIDITY, humidity);
  } else {
    SDDataString += "-,-,";
  }
}

void sensor_sgp40_init(void) {
  uint16_t error;
  char errorMessage[256];

  sgp40.begin(Wire);

  uint16_t serialNumber[3];
  uint8_t serialNumberSize = 3;

  error = sgp40.getSerialNumber(serialNumber, serialNumberSize);

  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      uint16_t value = serialNumber[i];
      Serial.print(value < 4096 ? "0" : "");
      Serial.print(value < 256 ? "0" : "");
      Serial.print(value < 16 ? "0" : "");
      Serial.print(value, HEX);
    }
    Serial.println();
  }

  uint16_t testResult;
  error = sgp40.executeSelfTest(testResult);
  if (error) {
    Serial.print("Error trying to execute executeSelfTest(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (testResult != 0xD400) {
    Serial.print("executeSelfTest failed with error: ");
    Serial.println(testResult);
  }
}

void sensor_sgp40_get(void) {
  uint16_t error;
  char errorMessage[256];
  uint16_t srawVoc = 0;

  Serial.print("sensor sgp40: ");

  error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
  if (error) {
    Serial.print("Error trying to execute measureRawSignal(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SRAW_VOC:");
    Serial.println(srawVoc);
  }

  SDDataString += "sgp40,";
  if (error) {
    SDDataString += "-,";
  } else {
    SDDataString += String(srawVoc);
    SDDataString += ',';

    int32_t voc_index = voc_algorithm.process(srawVoc);
    Serial.print("VOC Index: ");
    Serial.println(voc_index);

    sensor_data_send(PKT_TYPE_SENSOR_TVOC_INDEX, (float)voc_index);
  }
}


// -------------------- NEW: HM3301 / HM330X (PM) --------------------
void sensor_hm3301_init(void) {
  // HM3301 / HM330X is typically at 0x40
  if (!i2c_ping(0x40)) {
    Serial.println("HM3301 not found on I2C (0x40)");
    hm3301_ready = false;
    return;
  }
  // hm3301.init() returns 0 on success in Seeed examples
  if (hm3301.init()) {
    Serial.println("HM3301 init failed");
    hm3301_ready = false;
    return;
  }
  hm3301_ready = true;
  Serial.println("HM3301 ready (0x40)");
}

void sensor_hm3301_get(void) {
  Serial.print("sensor hm3301: ");

  SDDataString += "hm3301,";
  if (!hm3301_ready) {
    Serial.println("not ready");
    SDDataString += "-,-,-,";
    return;
  }

  // hm3301.read_sensor_value() returns 0 on success in Seeed examples
  if (hm3301.read_sensor_value(hm_buf, 29)) {
    Serial.println("read fail");
    SDDataString += "-,-,-,";
    return;
  }

  // Optional checksum guard (avoid sending garbage)
  if (!hm3301_checksum_ok(hm_buf)) {
    Serial.println("checksum fail");
    SDDataString += "-,-,-,";
    return;
  }

  // Atmospheric mapping (big-endian):
  // pm1  = buf[10..11], pm2.5 = buf[12..13], pm10 = buf[14..15]
  uint16_t pm1  = (uint16_t(hm_buf[10]) << 8) | hm_buf[11];
  uint16_t pm25 = (uint16_t(hm_buf[12]) << 8) | hm_buf[13];
  uint16_t pm10 = (uint16_t(hm_buf[14]) << 8) | hm_buf[15];

  Serial.print("PM1.0=");
  Serial.print(pm1);
  Serial.print(" PM2.5=");
  Serial.print(pm25);
  Serial.print(" PM10=");
  Serial.println(pm10);

  SDDataString += String(pm1);
  SDDataString += ',';
  SDDataString += String(pm25);
  SDDataString += ',';
  SDDataString += String(pm10);
  SDDataString += ',';

  sensor_data_send(PKT_TYPE_SENSOR_PM1_0, (float)pm1);
  sensor_data_send(PKT_TYPE_SENSOR_PM2_5, (float)pm25);
  sensor_data_send(PKT_TYPE_SENSOR_PM10, (float)pm10);
}

// -------------------- NEW: Seeed MultiGas v2 (GMXXX) --------------------
void sensor_multigas_init(void) {
  if (!i2c_ping(0x08)) {
    Serial.println("MultiGas v2 not found on I2C (0x08)");
    gas_ready = false;
    return;
  }
  gas.begin(Wire, 0x08);
  gas_ready = true;
  Serial.println("MultiGas v2 ready (0x08)");
}

void sensor_multigas_get(void) {
  Serial.print("sensor multigas: ");

  SDDataString += "multigas,";
  if (!gas_ready) {
    Serial.println("not ready");
    SDDataString += "-,-,-,-,";
    return;
  }

  // Library returns unsigned int; send as float as requested
  unsigned int gm102b = gas.getGM102B();
  unsigned int gm302b = gas.getGM302B();
  unsigned int gm502b = gas.getGM502B();
  unsigned int gm702b = gas.getGM702B();

  Serial.print("GM102B=");
  Serial.print(gm102b);
  Serial.print(" GM302B=");
  Serial.print(gm302b);
  Serial.print(" GM502B=");
  Serial.print(gm502b);
  Serial.print(" GM702B=");
  Serial.println(gm702b);

  SDDataString += String(gm102b);
  SDDataString += ',';
  SDDataString += String(gm302b);
  SDDataString += ',';
  SDDataString += String(gm502b);
  SDDataString += ',';
  SDDataString += String(gm702b);
  SDDataString += ',';

  sensor_data_send(PKT_TYPE_SENSOR_GM102B, (float)gm102b);
  sensor_data_send(PKT_TYPE_SENSOR_GM302B, (float)gm302b);
  sensor_data_send(PKT_TYPE_SENSOR_GM502B, (float)gm502b);
  sensor_data_send(PKT_TYPE_SENSOR_GM702B, (float)gm702b);
}


void sensor_scd4x_init(void) {
  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire, 0x62);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  uint64_t serialNumber;
  error = scd4x.getSerialNumber(serialNumber);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("Serial number: ");
    Serial.println((uint32_t)(serialNumber >> 32), HEX);
    Serial.println((uint32_t)(serialNumber & 0xFFFFFFFF), HEX);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
}

void sensor_scd4x_get(void) {
  uint16_t error;
  char errorMessage[256];

  Serial.print("sensor scd4x: ");
  Serial.print("[internal] ");
  // Read Measurement
  uint16_t co2;
  float temperature;
  float humidity;
  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (co2 == 0) {
    Serial.println("Invalid sample detected, skipping.");
  } else {
    Serial.print("Co2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.println(humidity);
  }

  SDDataString += "scd4x,";
  if (error) {
    SDDataString += "-,-,-,";
  } else {
    SDDataString += String(co2);
    SDDataString += ',';
    SDDataString += String(temperature);
    SDDataString += ',';
    SDDataString += String(humidity);
    SDDataString += ',';


    sensor_data_send(PKT_TYPE_SENSOR_SCD41_CO2, (float)co2);

    // internal temperature/humidity are from SCD4x
    sensor_data_send(PKT_TYPE_SENSOR_SHT41_TEMP, temperature);
    sensor_data_send(PKT_TYPE_SENSOR_SHT41_HUMIDITY, humidity);
  }
}



//Buzzer
#define Buzzer 19 //Buzzer GPIO

void beep_init(void) {
  pinMode(Buzzer, OUTPUT);
}
void beep_off(void) {
  digitalWrite(19, LOW);
}
void beep_on(void) {
  analogWrite(Buzzer, 127);
  delay(50);
  analogWrite(Buzzer, 0);
}

void grove_adc_get(void) {
  String dataString = "";
  int adc0 = analogRead(26);
  dataString += String(adc0);
  dataString += ',';
  int adc1 = analogRead(27);
  dataString += String(adc1);
  Serial.print("grove adc: ");
  Serial.println(dataString);
}



// recv cmd from esp32
bool shutdown_flag = false;

void onPacketReceived(const uint8_t *buffer, size_t size) {

#if DEBUG
  Serial.printf("<--- recv len:%d, data: ", size);
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%x ", buffer[i]);
  }
  Serial.println("");
#endif

  switch (buffer[0]) {
    case PKT_TYPE_CMD_SHUTDOWN:
      {
        Serial.println("cmd shutdown");
        shutdown_flag = true;
        sensor_power_off();
        break;
      }
    default:
      break;
  }
}

int cnt = 0;
int i = 0;
bool sd_init_flag = 0;

void setup() {
  Serial.begin(115200);

  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  sensor_power_on();

  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();

  // NEW Grove I2C sensors
  sensor_multigas_init();
  sensor_hm3301_init();

  const int chipSelect = 13;
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  if (!SD.begin(chipSelect, 1000000, SPI1)) {
    Serial.println("Card failed, or not present");
    sd_init_flag = 0;
  } else {
    Serial.println("card initialized.");
    sd_init_flag = 1;
  }

  sensor_aht_init();
  sensor_sgp40_init();
  sensor_scd4x_init();

  int32_t index_offset;
  int32_t learning_time_offset_hours;
  int32_t learning_time_gain_hours;
  int32_t gating_max_duration_minutes;
  int32_t std_initial;
  int32_t gain_factor;
  voc_algorithm.get_tuning_parameters(
    index_offset, learning_time_offset_hours, learning_time_gain_hours,
    gating_max_duration_minutes, std_initial, gain_factor);

  Serial.println("\nVOC Gas Index Algorithm parameters");
  Serial.print("Index offset:\t");
  Serial.println(index_offset);
  Serial.print("Learing time offset hours:\t");
  Serial.println(learning_time_offset_hours);
  Serial.print("Learing time gain hours:\t");
  Serial.println(learning_time_gain_hours);
  Serial.print("Gating max duration minutes:\t");
  Serial.println(gating_max_duration_minutes);
  Serial.print("Std inital:\t");
  Serial.println(std_initial);
  Serial.print("Gain factor:\t");
  Serial.println(gain_factor);


  beep_init();
  delay(500);
  beep_on();

  Serial.printf(SENSECAP, VERSION);
}


void loop() {
  if (i > 500) {
    i = 0;

    SDDataString = "";
    Serial.printf("\r\n\r\n--------- start measure %d-------\r\n", cnt);

    SDDataString += String(cnt);
    SDDataString += ',';

    cnt++;
    sensor_aht_get();
    sensor_sgp40_get();
    sensor_hm3301_get();
    sensor_multigas_get();
    sensor_scd4x_get();
    grove_adc_get();

    if (sd_init_flag) {
      File dataFile = SD.open("datalog.csv", FILE_WRITE);
      // if the file is available, write to it:
      if (dataFile) {
        dataFile.println(SDDataString);
        dataFile.close();
        // print to the serial port too:
        Serial.print("sd write: ");
        Serial.println(SDDataString);
      } else {
        // if the file isn't open, pop up an error:
        Serial.println("error opening datalog.txt");
      }
    }
  }

  i++;

  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
  }
  delay(10);

  // while( shutdown_flag) {
  //    delay(10);
  // }
}

