#include <Wire.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <at24c32.h>
#include <ArduinoJson.h>

// ========== I2C & Sensor Config ==========
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Alamat I2C
const uint8_t BH1750_I2C_ADDR = 0x23;
const uint8_t BME280_I2C_ADDR = 0x76;
const uint8_t RTC_I2C_ADDR = 0x68;
const uint8_t EEPROM_I2C_ADDR = 0x57;

// Inisialisasi Sensor Objek
BH1750 lightMeter(BH1750_I2C_ADDR);
Adafruit_BME280 bme;
RTC_DS3231 rtc;
AT24C32 eeprom(EEPROM_I2C_ADDR);

// ========== KONFIGURASI MQ-135 & KALIBRASI ==========
const int MQ135_PIN = 34;
const float MQ135_RL = 10.0; // Load Resistor dalam KiloOhms (biasanya 10k atau 20k)
// Variabel Kalibrasi Global
float MQ135_R0 = 10.0;       // Resistansi udara bersih (akan di-load dari EEPROM)
float MQ135_GAMMA = 0.70;    // Slope karakteristik gas

// Alamat EEPROM untuk MQ135 (Mulai dari 50 agar aman dari eepromCounter di alamat 0)
const int EEPROM_ADDR_R0 = 50;

// GPS
static const int RXPin = 16;
static const int TXPin = 17;
static const uint32_t GPSBaud = 9600;
HardwareSerial ss(1);
TinyGPSPlus gps;

// LoRa Config
#define SCK 18
#define MISO 19
#define MOSI 23
#define NSS 5
#define RST 14
#define DIO0 4

// MQ-135 ADC Limits (Untuk mapping fallback)
const int ADC_CLEAN = 300;
const int ADC_POLUTED = 800;

// ========== RTOS VARIABLES ==========
TaskHandle_t TaskSensorRead, TaskSystemGPS, TaskDataProcessor, TaskLoRaSender;
QueueHandle_t sensorRawQueue, systemRawQueue, processedDataQueue;
SemaphoreHandle_t rtcMutex, eepromMutex, loraMutex;

// Data Structures
struct SensorRawData {
  float lux;
  float temp;
  float hum;
  float pres;
  int air_quality; // Persentase kebersihan udara
  int eeprom_count;
  char rtc_time[25];
};

struct SystemRawData {
  char latitude[20];
  char longitude[20];
  uint32_t cpu_freq;
  uint32_t ram_used;
  long uptime_sec;
};

struct ProcessedData {
  char sensorJSON[512];
  char systemJSON[512];
  unsigned long timestamp;
};

// Global Variables
int eepromCounter = 0;
unsigned int eepromAddress = 0; // Alamat counter loop

// ========== HELPER FUNCTIONS: KALIBRASI MQ-135 ==========

// Fungsi Membaca Resistansi Sensor Saat Ini (Rs)
float getMQ135Resistance() {
  int adcValue = analogRead(MQ135_PIN);
  if(adcValue == 0) return 999999; // Hindari pembagian dengan nol
  
  // Konversi ADC ke Voltase (ESP32 ADC 12-bit: 0-4095, Vref 3.3V)
  float voltage = adcValue * (3.3 / 4095.0);
  
  // Rumus Resistansi Sensor: Rs = (Vcc - Vout) / Vout * RL
  float rs = ((3.3 - voltage) / voltage) * MQ135_RL;
  return rs;
}

// Load Kalibrasi dari EEPROM saat Booting
void loadCalibration() {
  float loadedR0;
  
  // Ambil data float dari EEPROM
  eeprom.get(EEPROM_ADDR_R0, loadedR0);
  
  // Validasi data (jika EEPROM kosong/nan, gunakan default)
  if (isnan(loadedR0) || loadedR0 <= 0 || loadedR0 > 100000) {
    Serial.println("⚠️ MQ135: Tidak ada kalibrasi tersimpan, menggunakan default.");
    MQ135_R0 = 10.0; // Default R0
  } else {
    MQ135_R0 = loadedR0;
    Serial.print("✅ MQ135: Kalibrasi dimuat dari EEPROM. R0: ");
    Serial.println(MQ135_R0);
  }
}

// Fungsi Simpan Kalibrasi ke EEPROM
void saveCalibration(float newR0) {
  if (xSemaphoreTake(eepromMutex, portMAX_DELAY) == pdTRUE) {
    eeprom.put(EEPROM_ADDR_R0, newR0);
    xSemaphoreGive(eepromMutex);
    Serial.print("💾 MQ135: Kalibrasi baru disimpan! R0: ");
    Serial.println(newR0);
  }
}

// Fungsi Utama Membaca Kualitas Udara (0-100%)
int readAirQuality() {
  float rs = getMQ135Resistance();
  
  // --- AUTO CALIBRATION LOGIC ---
  // Jika resistansi saat ini (Rs) jauh lebih tinggi dari R0 tersimpan,
  // artinya udara sekarang lebih bersih dari "udara bersih" yang kita tahu sebelumnya.
  // Maka, kita update baseline R0.
  if (rs > MQ135_R0) {
    MQ135_R0 = rs; // Update nilai di RAM
    // Kita simpan ke EEPROM (Opsional: beri jeda/threshold agar tidak terlalu sering write)
    static unsigned long lastSave = 0;
    if (millis() - lastSave > 10000) { // Max save tiap 10 detik jika drift
      saveCalibration(MQ135_R0);
      lastSave = millis();
    }
  }

  // Hitung Ratio
  float ratio = rs / MQ135_R0; // Semakin mendekati 1.0, semakin bersih
  
  // Konversi ke Persentase (Estimasi sederhana untuk index)
  // Ratio 1.0 = 100% Bersih. Ratio 0.1 = Polusi berat.
  // Kita gunakan pendekatan logaritmik atau linear sederhana
  int qualityPercent = (int)(ratio * 100.0);
  
  qualityPercent = constrain(qualityPercent, 0, 100);
  return qualityPercent;
}

// ========== HELPER FUNCTIONS: SYSTEM ==========

void getRTCTime(char* buffer) {
  if (xSemaphoreTake(rtcMutex, portMAX_DELAY) == pdTRUE) {
    DateTime now = rtc.now();
    sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d", 
             now.day(), now.month(), now.year(),
             now.hour(), now.minute(), now.second());
    xSemaphoreGive(rtcMutex);
  }
}

void updateEEPROMCounter() {
  if (xSemaphoreTake(eepromMutex, portMAX_DELAY) == pdTRUE) {
    eeprom.get(eepromAddress, eepromCounter);
    eepromCounter++;
    eeprom.put(eepromAddress, eepromCounter);
    xSemaphoreGive(eepromMutex);
  }
}

// ========== TASK 1: SENSOR READ ==========
void taskSensorRead(void *parameter) {
  Serial.println("🎯 Task Sensor Read started");
  SensorRawData sensorData;
  unsigned long lastRead = 0;
  const unsigned long SENSOR_INTERVAL = 2000;
  
  while (1) {
    unsigned long now = millis();
    if (now - lastRead >= SENSOR_INTERVAL) {
      // Baca semua sensor
      sensorData.lux = lightMeter.readLightLevel();
      sensorData.temp = bme.readTemperature();
      sensorData.hum = bme.readHumidity();
      sensorData.pres = bme.readPressure() / 100.0F;
      
      // Baca MQ-135 dengan fungsi baru yang sudah dikalibrasi
      sensorData.air_quality = readAirQuality();

      // Update EEPROM counter dan waktu RTC
      updateEEPROMCounter();
      sensorData.eeprom_count = eepromCounter;
      getRTCTime(sensorData.rtc_time);

      // Kirim ke queue
      if (xQueueSend(sensorRawQueue, &sensorData, 0) == pdTRUE) {
        Serial.print("📊 Sensor data collected. AirQuality: ");
        Serial.print(sensorData.air_quality);
        Serial.println("%");
      }
      
      lastRead = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ========== TASK 2: SYSTEM & GPS READ ==========
void taskSystemGPS(void *parameter) {
  Serial.println("🖥️ Task System GPS started");
  SystemRawData systemData;
  unsigned long lastRead = 0;
  const unsigned long SYSTEM_INTERVAL = 3000;
  
  while (1) {
    unsigned long now = millis();
    // Process GPS data continuously
    while (ss.available() > 0) {
      gps.encode(ss.read());
    }
    
    if (now - lastRead >= SYSTEM_INTERVAL) {
      // Baca data sistem
      systemData.cpu_freq = getCpuFrequencyMhz();
      systemData.ram_used = ESP.getHeapSize() - ESP.getFreeHeap();
      systemData.uptime_sec = millis() / 1000;

      // Baca GPS
      if (gps.location.isValid() && gps.satellites.value() >= 3) {
        dtostrf(gps.location.lat(), 10, 6, systemData.latitude);
        dtostrf(gps.location.lng(), 10, 6, systemData.longitude);
      } else {
        strcpy(systemData.latitude, "-8.160632");
        strcpy(systemData.longitude, "113.724054");
      }
      
      // Kirim ke queue
      if (xQueueSend(systemRawQueue, &systemData, 0) == pdTRUE) {
        // Serial.println("🛰️ System/GPS data collected");
      }
      
      lastRead = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

// ========== TASK 3: DATA PROCESSOR ==========
void taskDataProcessor(void *parameter) {
  Serial.println("🔄 Task Data Processor started");
  SensorRawData sensorData;
  SystemRawData systemData;
  ProcessedData processedData;
  
  while (1) {
    if (xQueueReceive(sensorRawQueue, &sensorData, portMAX_DELAY) == pdTRUE &&
        xQueueReceive(systemRawQueue, &systemData, portMAX_DELAY) == pdTRUE) {
      
      // Buat JSON untuk sensor data
      DynamicJsonDocument sensorDoc(512);
      sensorDoc["rtc_time"] = sensorData.rtc_time;
      sensorDoc["lux"] = sensorData.lux;
      sensorDoc["temp"] = sensorData.temp;
      sensorDoc["hum"] = sensorData.hum;
      sensorDoc["pres"] = sensorData.pres;
      sensorDoc["air_clean_perc"] = sensorData.air_quality;
      sensorDoc["eeprom_count"] = sensorData.eeprom_count;
      
      serializeJson(sensorDoc, processedData.sensorJSON, sizeof(processedData.sensorJSON));
      
      // Buat JSON untuk system data
      DynamicJsonDocument systemDoc(512);
      systemDoc["latitude"] = systemData.latitude;
      systemDoc["longitude"] = systemData.longitude;
      systemDoc["cpu_freq"] = systemData.cpu_freq;
      systemDoc["ram_used"] = systemData.ram_used;
      systemDoc["uptime_sec"] = systemData.uptime_sec;
      
      serializeJson(systemDoc, processedData.systemJSON, sizeof(processedData.systemJSON));
      processedData.timestamp = millis();
      
      // Kirim data yang sudah diproses ke queue sender
      if (xQueueSend(processedDataQueue, &processedData, 0) == pdTRUE) {
        Serial.println("📦 Data processed JSON ready");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========== TASK 4: LORA SENDER ==========
void taskLoRaSender(void *parameter) {
  Serial.println("📤 Task LoRa Sender started");
  ProcessedData sendData;
  unsigned long lastSend = 0;
  const unsigned long SEND_INTERVAL = 4000;
  
  while (1) {
    unsigned long now = millis();
    if (xQueueReceive(processedDataQueue, &sendData, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (now - lastSend >= SEND_INTERVAL) {
        
        String loraPayload = "SENSOR:" + String(sendData.sensorJSON) + "|SYSTEM:" + String(sendData.systemJSON);

        if (xSemaphoreTake(loraMutex, portMAX_DELAY) == pdTRUE) {
          LoRa.beginPacket();
          LoRa.print(loraPayload);
          LoRa.endPacket();
          xSemaphoreGive(loraMutex);

          Serial.println("✅ LoRa Data Sent!");
          Serial.print("   Payload Size: ");
          Serial.println(loraPayload.length());
          Serial.println();
          
          lastSend = now;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== WeatherTech Transceiver (MQ135 Calibrated) ===");
  
  // Inisialisasi I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Inisialisasi sensor
  if (!lightMeter.begin()) Serial.println("❌ BH1750 init failed!");
  if (!bme.begin(BME280_I2C_ADDR, &Wire)) Serial.println("❌ BME280 init failed!");
  
  // Inisialisasi GPS
  ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  // Inisialisasi LoRa
  Serial.println("📡 Memulai Inisialisasi LoRa...");
  SPI.begin(SCK, MISO, MOSI, NSS);
  LoRa.setPins(NSS, RST, DIO0);
  while (!LoRa.begin(433E6)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ LoRa OK!");
  
  // Inisialisasi RTC
  if (!rtc.begin()) Serial.println("❌ RTC init failed!");
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // Inisialisasi EEPROM & Counter
  eeprom.get(eepromAddress, eepromCounter);
  if (eepromCounter == -1 || eepromCounter > 50000) {
    eepromCounter = 0;
    eeprom.put(eepromAddress, eepromCounter);
  }

  // ========== RTOS INIT ==========
  sensorRawQueue = xQueueCreate(3, sizeof(SensorRawData));
  systemRawQueue = xQueueCreate(3, sizeof(SystemRawData));
  processedDataQueue = xQueueCreate(2, sizeof(ProcessedData));
  
  rtcMutex = xSemaphoreCreateMutex();
  eepromMutex = xSemaphoreCreateMutex();
  loraMutex = xSemaphoreCreateMutex();

  // === LOAD KALIBRASI MQ135 ===
  // Mengambil nilai R0 yang tersimpan agar pembacaan konsisten setelah restart
  // Kita perlu menggunakan Mutex karena eeprom diakses oleh beberapa task
  if (xSemaphoreTake(eepromMutex, portMAX_DELAY) == pdTRUE) {
      loadCalibration();
      xSemaphoreGive(eepromMutex);
  }

  // ========== TASK CREATION ==========
  xTaskCreatePinnedToCore(taskLoRaSender, "LoRaSender", 8192, NULL, 4, &TaskLoRaSender, 1);
  xTaskCreatePinnedToCore(taskDataProcessor, "DataProcessor", 8192, NULL, 3, &TaskDataProcessor, 1);
  xTaskCreatePinnedToCore(taskSensorRead, "SensorRead", 6144, NULL, 2, &TaskSensorRead, 0);
  xTaskCreatePinnedToCore(taskSystemGPS, "SystemGPS", 6144, NULL, 1, &TaskSystemGPS, 0);

  Serial.println("✅ RTOS Started Successfully!");
}

void loop() {
  vTaskDelete(NULL);
}