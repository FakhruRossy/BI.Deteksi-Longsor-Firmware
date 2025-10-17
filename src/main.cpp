#include <Arduino.h>
#include <Wire.h> 
#include <MPU6050.h> // Library MPU dari Jrowberg
#include <Kalman.h>  // Library Kalman dari TKJElectronics

// ===================================
// 1. DEFINISI VARIABEL & KONSTANTA
// ===================================

MPU6050 mpu;
Kalman kalmanX; 
Kalman kalmanY;

// GANTI NILAI INI dengan hasil kalibrasi di lokasi riil saat tanah stabil!
const float SUDUT_PITCH_AWAL = 0.50; 
const float SUDUT_ROLL_AWAL  = 1.20; 

// Ambang Batas Perubahan Sudut (Threshold)
const float THRESHOLD_WASPADA = 1.0; // Contoh: Perubahan 1.0 derajat
const float THRESHOLD_AWAS    = 3.0; // Contoh: Perubahan 3.0 derajat

long timer; // Untuk menghitung delta time (dt)

// ===================================
// 2. FUNGSI SETUP (Inisialisasi I2C dan Sensor)
// ===================================

void setup() {
    Serial.begin(115200);

    // Inisialisasi I2C untuk ESP32 (Menggunakan pin default: SDA=21, SCL=22)
    Wire.begin(); 
    
    // Inisialisasi MPU
    Serial.println("Inisialisasi MPU6050 di ESP32...");
    mpu.initialize();

    // Verifikasi koneksi
    while (!mpu.testConnection()) {
        Serial.println("Koneksi MPU6050 gagal! Cek wiring ke 3.3V, SDA=21, SCL=22.");
        delay(1000);
    }
    Serial.println("MPU6050 terhubung.");

    // --- KALIBRASI AWAL UNTUK KALMAN ---
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Hitung Sudut Awal dari Akselerometer (untuk inisialisasi Filter Kalman)
    float accAngleX = atan2(ay, az) * 180 / PI; 
    float accAngleY = atan2(ax, az) * 180 / PI; 

    // Set nilai awal Kalman dengan Sudut dari Accelerometer
    kalmanX.setAngle(accAngleX); 
    kalmanY.setAngle(accAngleY);

    timer = micros();

    Serial.println("-------------------------------------");
    Serial.println("Sistem Deteksi Longsor ESP32 Aktif! ⛰️");
    Serial.print("Referensi Pitch Awal: "); Serial.print(SUDUT_PITCH_AWAL, 2); Serial.println(" deg");
    Serial.print("Referensi Roll Awal: "); Serial.print(SUDUT_ROLL_AWAL, 2); Serial.println(" deg");
    Serial.println("-------------------------------------");
    
    // Opsional: Di sini Anda bisa menambahkan inisialisasi Wi-Fi / IoT
}

// ===================================
// 3. FUNGSI LOOP (Pengolahan Data & Deteksi)
// ===================================

void loop() {
    // A. PEMBACAAN DAN HITUNG DELTA TIME
    long currentTime = micros();
    float dt = (float)(currentTime - timer) / 1000000.0;
    timer = currentTime;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // B. MENGHITUNG NILAI INPUT UNTUK KALMAN
    
    // Sudut dari Akselerometer
    float accAngleX = atan2(ay, az) * 180 / PI; // Roll
    float accAngleY = atan2(ax, az) * 180 / PI; // Pitch

    // Kecepatan Sudut dari Gyroscope (Konversi dari nilai mentah)
    float gyroRateX = (float)gx / 131.0; 
    float gyroRateY = (float)gy / 131.0; 

    // C. APLIKASI KALMAN FILTER
    float kalAngleRoll = kalmanX.getAngle(accAngleX, gyroRateX, dt);
    float kalAnglePitch = kalmanY.getAngle(accAngleY, gyroRateY, dt);

    // D. LOGIKA DETEKSI LONGSOR (THRESHOLDING)
    
    // Hitung Perubahan Sudut
    float deltaRoll = abs(kalAngleRoll - SUDUT_ROLL_AWAL);
    float deltaPitch = abs(kalAnglePitch - SUDUT_PITCH_AWAL);

    // Ambil perubahan terbesar
    float deltaMax = max(deltaRoll, deltaPitch);

    Serial.print("Roll: "); Serial.print(kalAngleRoll, 2);
    Serial.print(" | Pitch: "); Serial.print(kalAnglePitch, 2);
    Serial.print(" | Delta Max: "); Serial.print(deltaMax, 2);
    
    // -------------------------------------
    // PEMERIKSAAN THRESHOLD
    // -------------------------------------

    if (deltaMax >= THRESHOLD_AWAS) {
        Serial.println(" --> !!! AWAS: LONGSOR SEGERA !!! 🚨");
        // Tambahkan fungsi notifikasi darurat Wi-Fi/Telegram/Blynk
        
    } else if (deltaMax >= THRESHOLD_WASPADA) {
        Serial.println(" --> WASPADA: Pergeseran terdeteksi. ⚠️");
        // Tambahkan fungsi notifikasi peringatan
        
    } else {
        Serial.println(" --> AMAN ✅");
    }

    delay(100); 
}