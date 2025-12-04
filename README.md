# 🌤️ WeatherTech - IoT Environmental Monitoring System

**WeatherTech** adalah sistem pemantauan lingkungan berbasis ESP32 yang menggunakan komunikasi **LoRa (Long Range)** untuk area tanpa internet dan **MQTT/WiFi** untuk pengiriman data ke cloud.

Sistem ini terdiri dari dua perangkat utama:
1.  **Transceiver Node:** Mengambil data sensor (Cuaca, Kualitas Udara, GPS) dan mengirimkannya via LoRa.
2.  **Gateway Node:** Menerima data LoRa, menampilkan status, dan meneruskannya ke server MQTT via WiFi.

---

## 🚀 Fitur Utama

* **Dual-Core Processing (RTOS):** Menggunakan FreeRTOS untuk manajemen multitasking (Sensor Reading, LoRa Comm, GPS, MQTT) yang stabil.
* **Long Range Communication:** Komunikasi antar device menggunakan modul LoRa 433MHz.
* **Auto-Calibration:** Algoritma kalibrasi otomatis untuk sensor kualitas udara (MQ-135) dengan penyimpanan EEPROM.
* **Memory Safe:** Manajemen memori menggunakan Semaphore/Mutex untuk mencegah *race condition* antar task.
* **Remote Control:** Fitur kontrol kipas (Fan) pada Gateway yang bisa dikendalikan via MQTT.
* **JSON Data Structure:** Pengiriman data menggunakan format JSON yang efisien.

---

## 📂 Struktur Folder

Pastikan struktur folder Anda seperti berikut agar dapat dikompilasi di Arduino IDE:

```text
Repo-Root/
├── Gateway_Done/          # Kode untuk ESP32 Gateway (Penerima + MQTT)
│   └── Gateway_Done.ino
│
└── Transciever/           # Kode untuk ESP32 Sensor Node (Pengirim)
    └── Transciever.ino
🛠️ Hardware yang Dibutuhkan1. Transceiver (Sensor Node)MCU: ESP32 DevKit V1LoRa: Ra-02 SX1278 (433MHz)Sensor Cahaya: BH1750 (I2C)Sensor Cuaca: BME280 (Temp, Hum, Pressure - I2C)Kualitas Udara: MQ-135 (Analog)RTC: DS3231 (Waktu Real-time)GPS: Neo-6M2. Gateway (Receiver Node)MCU: ESP32 DevKit V1LoRa: Ra-02 SX1278 (433MHz)Actuator: Kipas/Fan 5V (via Relay/MOSFET)🔌 Pin Mapping (Wiring)LoRa Module (Kedua Device)Pin LoRaPin ESP32NSSGPIO 5RSTGPIO 14DIO0GPIO 4MOSIGPIO 23MISOGPIO 19SCKGPIO 18Transceiver SensorsI2C (BH1750, BME280, RTC, EEPROM): SDA (21), SCL (22)MQ-135: GPIO 34 (Analog Input)GPS: RX (16), TX (17)Gateway ActuatorFan Control: GPIO 33📦 Library DependenciesInstall library berikut melalui Arduino IDE Library Manager:LoRa by Sandeep MistryArduinoJson by Benoit BlanchonPubSubClient (untuk MQTT)Adafruit BME280 LibraryAdafruit Unified SensorBH1750 by Christopher LawsRTClib by AdafruitTinyGPSPlus⚙️ Konfigurasi & InstalasiClone Repository:Bashgit clone [https://github.com/USERNAME_ANDA/hardware_code.weathertech.git](https://github.com/USERNAME_ANDA/hardware_code.weathertech.git)
Setting WiFi & MQTT (PENTING):Buka file Gateway_Done.ino, cari bagian berikut dan sesuaikan dengan kredensial Anda:C++const char* ssid = "NAMA_WIFI_ANDA";
const char* password = "PASSWORD_WIFI_ANDA";
String MQTT_HOST = "broker.hivemq.com"; // Ganti dengan broker Anda
Upload:Buka Transciever.ino -> Upload ke ESP32 Sensor Node.Buka Gateway_Done.ino -> Upload ke ESP32 Gateway Node.📡 MQTT TopicsTopicArahFungsi/weathertech/sensor_dataPublishMengirim data sensor (Lux, Temp, MQ135, dll)/weathertech/system_dataPublishMengirim data sistem (GPS, RAM, Uptime)/weathertech/gateway_systemPublishStatus kesehatan Gateway/weathertech/controlkipasSubscribeKontrol Kipas: ON, OFF, AUTO📝 Catatan PengembangProject ini dikembangkan untuk tujuan edukasi dan implementasi IoT di area agrikultur/perkotaan. Kode menggunakan pendekatan Non-Blocking dengan FreeRTOS untuk memastikan performa maksimal.Author: Bayu/WeatherTech

***

### Cara Menambahkan File ini ke GitHub:

1.  Buat file baru di folder `Project_WeatherTech` Anda, beri nama `README.md`.
2.  Paste teks di atas ke dalamnya.
3.  Simpan file.
4.  Jalankan perintah ini di terminal:
    ```bash
    git add README.md
    git commit -m "Menambahkan dokumentasi README"
    git push origin main
    ```
