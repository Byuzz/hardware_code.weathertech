#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>

// ========== WiFi & MQTT Config ==========
const char* ssid = "Your_WiFi";
const char* password = "Your_Password"; 

//set akun HiveMQ
String MQTT_HOST = "##";
int MQTT_PORT = 8883;
String MQTT_USER = "##";
String MQTT_PASS = "##"; 
String DEVICE_NAME = "weathertech";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

// Root CA certificate
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// Pin LoRa
#define SCK 18
#define MISO 19
#define MOSI 23
#define NSS 5
#define RST 14
#define DIO0 4

// ==== TAMBAHAN KONTROL KIPAS ====
#define FAN_PIN 33

enum FanMode {
  MODE_OFF,
  MODE_ON,
  MODE_AUTO
};

// Global Variable untuk Kipas
FanMode currentFanMode = MODE_AUTO; // Default AUTO
float lastKnownTemp = 0.0;          // Menyimpan suhu terakhir dari sensor

// RTOS Variabel
TaskHandle_t TaskGatewaySystem, TaskLoRaReceiver, TaskDataCollector, TaskMQTTScheduler, TaskFanControl; // Ditambah TaskFanControl
QueueHandle_t sensorQueue, systemQueue, gatewayQueue, publishQueue;
SemaphoreHandle_t jsonMutex;

// Data Structures - GUNAKAN CHAR ARRAY
struct SensorData {
  char rtc_time[25];
  float lux;
  float temp;
  float hum;
  float pres;
  int air_clean_perc;
  int eeprom_count;
};

struct SystemData {
  char latitude[20];
  char longitude[20];
  uint32_t cpu_freq;
  uint32_t ram_used;
  long uptime_sec;
};

struct GatewayData {
  uint32_t g_cpu_freq;
  uint32_t g_ram_used;
  long g_uptime_sec;
};

struct PublishData {
  char topic[50];
  char payload[512];
};

// Global Variables
GatewayData currentGatewayData;
bool newGatewayData = false;
bool mqttConnected = false;
long lastMqttReconnectAttempt = 0;
const long MQTT_RECONNECT_INTERVAL = 5000;

// ======== Helper Functions ========
void safePrint(String message) {
  Serial.println(message);
}

// ======== MQTT Callback (Penerima Pesan) ========
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  safePrint("📩 Message arrived [" + String(topic) + "]: " + message);

  // Cek jika topiknya controlkipas
  if (String(topic) == "/weathertech/controlkipas") {
    if (message == "ON") {
      currentFanMode = MODE_ON;
      safePrint("💨 Fan Mode set to: ON");
    } else if (message == "OFF") {
      currentFanMode = MODE_OFF;
      safePrint("💨 Fan Mode set to: OFF");
    } else if (message == "AUTO") {
      currentFanMode = MODE_AUTO;
      safePrint("💨 Fan Mode set to: AUTO");
    }
  }
}

// ======== MQTT Functions ========
void initMQTT() {
  wifiClient.setCACert(root_ca);
  mqttClient.setServer(MQTT_HOST.c_str(), MQTT_PORT);
  mqttClient.setCallback(mqttCallback); // Set Callback disini
  mqttClient.setBufferSize(2048);
}

bool mqttConnect() {
  safePrint("🔗 Connecting to MQTT...");
  
  wifiClient.stop();
  delay(1000);
  
  if (mqttClient.connect(DEVICE_NAME.c_str(), MQTT_USER.c_str(), MQTT_PASS.c_str())) {
    safePrint("✅ MQTT CONNECTED!");
    
    // Subscribe ke topik control kipas
    mqttClient.subscribe("/weathertech/controlkipas");
    safePrint("✅ Subscribed to /weathertech/controlkipas");

    mqttConnected = true;
    return true;
  } else {
    safePrint("❌ MQTT FAILED! Error: " + String(mqttClient.state()));
    mqttConnected = false;
    return false;
  }
}

void mqttHandle() {
  if (!mqttClient.connected()) {
    mqttConnected = false;
    long now = millis();
    if (now - lastMqttReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
      lastMqttReconnectAttempt = now;
      safePrint("🔄 Attempting MQTT reconnection...");
      if (mqttConnect()) {
        lastMqttReconnectAttempt = 0;
      }
    }
  } else {
    mqttClient.loop();
    if (!mqttConnected) {
      mqttConnected = true;
      safePrint("✅ MQTT connection re-established");
    }
  }
}

// ======== TASK 1: Gateway System Data ========
void taskGatewaySystem(void *parameter) {
  safePrint("🚀 Task Gateway System started");
  while (1) {
    // Update gateway system data
    GatewayData gwData;
    gwData.g_cpu_freq = getCpuFrequencyMhz();
    gwData.g_ram_used = ESP.getHeapSize() - ESP.getFreeHeap();
    gwData.g_uptime_sec = millis() / 1000;

    // Kirim ke gateway queue
    xQueueSend(gatewayQueue, &gwData, portMAX_DELAY);
    
    // safePrint("📊 Gateway data updated");
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// ======== TASK 2: LoRa Receiver ========
void taskLoRaReceiver(void *parameter) {
  safePrint("📡 Task LoRa Receiver started");
  while (1) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String incoming = "";
      while (LoRa.available()) {
        incoming += (char)LoRa.read();
      }
      
      safePrint("📨 Raw LoRa: " + incoming);
      // Parse data LoRa
      int sensorIndex = incoming.indexOf("SENSOR:");
      int systemIndex = incoming.indexOf("SYSTEM:");
      int separatorIndex = incoming.indexOf("|");
      
      if (sensorIndex != -1 && systemIndex != -1 && separatorIndex != -1 && separatorIndex > sensorIndex) {
        String sensorJSON = incoming.substring(sensorIndex + 7, separatorIndex);
        String systemJSON = incoming.substring(systemIndex + 7);
        
        safePrint("📋 Sensor JSON: " + sensorJSON);
        
        // Parse JSON dan convert ke struct
        DynamicJsonDocument sensorDoc(512);
        DeserializationError sensorError = deserializeJson(sensorDoc, sensorJSON);
        
        if (!sensorError) {
          SensorData sensorData;
          // Copy data ke char array
          const char* rtc_time = sensorDoc["rtc_time"];
          if (rtc_time) strncpy(sensorData.rtc_time, rtc_time, sizeof(sensorData.rtc_time)-1);
          sensorData.rtc_time[sizeof(sensorData.rtc_time)-1] = '\0';
          
          sensorData.lux = sensorDoc["lux"] | 0.0;
          sensorData.temp = sensorDoc["temp"] | 0.0;
          sensorData.hum = sensorDoc["hum"] | 0.0;
          sensorData.pres = sensorDoc["pres"] | 0.0;
          sensorData.air_clean_perc = sensorDoc["air_clean_perc"] | 0;
          sensorData.eeprom_count = sensorDoc["eeprom_count"] | 0;

          xQueueSend(sensorQueue, &sensorData, portMAX_DELAY);
          safePrint("📤 Sensor data queued");
        } else {
          safePrint("❌ Sensor JSON Error: " + String(sensorError.c_str()));
        }

        // Parse system data
        DynamicJsonDocument systemDoc(512);
        DeserializationError systemError = deserializeJson(systemDoc, systemJSON);
        
        if (!systemError) {
          SystemData systemData;
          const char* lat = systemDoc["latitude"];
          const char* lng = systemDoc["longitude"];
          if (lat) strncpy(systemData.latitude, lat, sizeof(systemData.latitude)-1);
          if (lng) strncpy(systemData.longitude, lng, sizeof(systemData.longitude)-1);
          systemData.latitude[sizeof(systemData.latitude)-1] = '\0';
          systemData.longitude[sizeof(systemData.longitude)-1] = '\0';
          
          systemData.cpu_freq = systemDoc["cpu_freq"] | 0;
          systemData.ram_used = systemDoc["ram_used"] | 0;
          systemData.uptime_sec = systemDoc["uptime_sec"] | 0;

          xQueueSend(systemQueue, &systemData, portMAX_DELAY);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ======== TASK 3: Data Collector ========
void taskDataCollector(void *parameter) {
  safePrint("🔄 Task Data Collector started");
  
  SensorData sensorData;
  SystemData systemData;
  GatewayData gatewayData;
  
  while (1) {
    // Tunggu data sensor dan system
    if (xQueueReceive(sensorQueue, &sensorData, portMAX_DELAY) == pdTRUE &&
        xQueueReceive(systemQueue, &systemData, portMAX_DELAY) == pdTRUE) {
      
      // === UPDATE GLOBAL TEMP UNTUK KIPAS ===
      // Menggunakan Mutex agar aman
      if (xSemaphoreTake(jsonMutex, portMAX_DELAY) == pdTRUE) {
         lastKnownTemp = sensorData.temp; // Update suhu terbaru dari sensor
         xSemaphoreGive(jsonMutex);
      }
      // ======================================

      // Coba ambil gateway data terbaru (non-blocking)
      bool hasGateway = (xQueueReceive(gatewayQueue, &gatewayData, 0) == pdTRUE);
      
      if (xSemaphoreTake(jsonMutex, portMAX_DELAY) == pdTRUE) {
        // 1. SENSOR DATA
        PublishData sensorPublish;
        strcpy(sensorPublish.topic, "/weathertech/sensor_data");
        
        DynamicJsonDocument sensorDoc(512);
        sensorDoc["rtc_time"] = sensorData.rtc_time;
        sensorDoc["lux"] = sensorData.lux;
        sensorDoc["temp"] = sensorData.temp;
        sensorDoc["hum"] = sensorData.hum;
        sensorDoc["pres"] = sensorData.pres;
        sensorDoc["air_clean_perc"] = sensorData.air_clean_perc;
        sensorDoc["eeprom_count"] = sensorData.eeprom_count;
        
        serializeJson(sensorDoc, sensorPublish.payload, sizeof(sensorPublish.payload));
        xQueueSend(publishQueue, &sensorPublish, portMAX_DELAY);

        // 2. SYSTEM DATA  
        PublishData systemPublish;
        strcpy(systemPublish.topic, "/weathertech/system_data");
        
        DynamicJsonDocument systemDoc(512);
        systemDoc["latitude"] = systemData.latitude;
        systemDoc["longitude"] = systemData.longitude;
        systemDoc["cpu_freq"] = systemData.cpu_freq;
        systemDoc["ram_used"] = systemData.ram_used;
        systemDoc["uptime_sec"] = systemData.uptime_sec;
        
        serializeJson(systemDoc, systemPublish.payload, sizeof(systemPublish.payload));
        xQueueSend(publishQueue, &systemPublish, portMAX_DELAY);
        
        // 3. GATEWAY DATA (jika ada)
        if (hasGateway) {
          PublishData gatewayPublish;
          strcpy(gatewayPublish.topic, "/weathertech/gateway_system");
          
          DynamicJsonDocument gatewayDoc(512);
          gatewayDoc["g_cpu_freq"] = gatewayData.g_cpu_freq;
          gatewayDoc["g_ram_used"] = gatewayData.g_ram_used;
          gatewayDoc["g_uptime_sec"] = gatewayData.g_uptime_sec;
          
          serializeJson(gatewayDoc, gatewayPublish.payload, sizeof(gatewayPublish.payload));
          xQueueSend(publishQueue, &gatewayPublish, portMAX_DELAY);
        }
        
        xSemaphoreGive(jsonMutex);
        safePrint("📦 Data batch created" + String(hasGateway ? " (with gateway)" : ""));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======== TASK 4: MQTT Scheduler ========
void taskMQTTScheduler(void *parameter) {
  safePrint("⏰ Task MQTT Scheduler started");
  PublishData publishData;
  while (1) {
    if (xQueueReceive(publishQueue, &publishData, portMAX_DELAY) == pdTRUE) {
      if (!mqttClient.connected()) {
        safePrint("❌ MQTT not connected, skipping: " + String(publishData.topic));
        continue;
      }
      
      if (strlen(publishData.payload) == 0) {
        continue;
      }
      
      bool success = mqttClient.publish(publishData.topic, publishData.payload);
      if (success) {
        safePrint("✅ PUBLISHED: " + String(publishData.topic));
      } else {
        safePrint("❌ PUBLISH FAILED: " + String(publishData.topic));
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ======== TASK 5: Fan Control Logic (BARU) ========
void taskFanControl(void *parameter) {
  safePrint("💨 Task Fan Control started");
  
  // Setup Pin Mode
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW); // Default mati

  while (1) {
    bool fanState = false;
    float currentTemp = 0.0;

    // Ambil suhu terakhir dengan aman
    if (xSemaphoreTake(jsonMutex, portMAX_DELAY) == pdTRUE) {
       currentTemp = lastKnownTemp;
       xSemaphoreGive(jsonMutex);
    }

    // Logika Utama
    switch (currentFanMode) {
      case MODE_ON:
        fanState = true;
        break;
        
      case MODE_OFF:
        fanState = false;
        break;
        
      case MODE_AUTO:
        // Jika suhu diatas 35, hidupkan kipas
        if (currentTemp > 35.0) {
          fanState = true;
        } else {
          fanState = false;
        }
        break;
    }

    // Eksekusi ke Hardware
    if (fanState) {
      digitalWrite(FAN_PIN, HIGH);
    } else {
      digitalWrite(FAN_PIN, LOW);
    }

    // Delay checking (update setiap 1 detik)
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ======== Setup ========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== WeatherTech Gateway (Memory Safe) ===");
  // WiFi Connection
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan WiFi");
  int wifiTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTimeout < 20) {
    delay(500);
    Serial.print(".");
    wifiTimeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi terkoneksi!");
    Serial.println("IP Address: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n❌ WiFi gagal terkoneksi!");
  }

  // MQTT Initialization
  initMQTT();

  // LoRa Initialization
  LoRa.setPins(NSS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("❌ Gagal memulai LoRa!");
    while (1);
  }
  Serial.println("✅ LoRa initialized!");

  // RTOS Initialization
  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  systemQueue = xQueueCreate(5, sizeof(SystemData));
  gatewayQueue = xQueueCreate(3, sizeof(GatewayData));
  publishQueue = xQueueCreate(10, sizeof(PublishData));
  jsonMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreatePinnedToCore(taskGatewaySystem, "TaskGatewaySystem", 4096, NULL, 1, &TaskGatewaySystem, 0);
  xTaskCreatePinnedToCore(taskLoRaReceiver, "TaskLoRaReceiver", 8192, NULL, 2, &TaskLoRaReceiver, 0);
  xTaskCreatePinnedToCore(taskDataCollector, "TaskDataCollector", 8192, NULL, 3, &TaskDataCollector, 0);
  xTaskCreatePinnedToCore(taskMQTTScheduler, "TaskMQTTScheduler", 8192, NULL, 4, &TaskMQTTScheduler, 1);
  
  // Task Baru Fan Control
  xTaskCreatePinnedToCore(taskFanControl, "TaskFanControl", 2048, NULL, 2, &TaskFanControl, 1);

  Serial.println("✅ Memory-Safe Gateway Started!");
}

void loop() {
  mqttHandle();
  vTaskDelay(pdMS_TO_TICKS(1000));
}