#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include <WebServer.h>

#include "MPU_sensor.h"
#include "FFT_module.h"

// ================== L298N Motor ==================
#define ENA 5
#define IN1 18
#define IN2 19

// ================== Encoder ==================
#define ENCODER_A 34
#define ENCODER_B 35
volatile long encoderCount = 0;
int lastEncoded = 0;

// Encoder interrupt
void IRAM_ATTR handleEncoder() {
    int MSB = digitalRead(ENCODER_A);
    int LSB = digitalRead(ENCODER_B);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderCount++;
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderCount--;

    lastEncoded = encoded;
}

// ===================== WIFI =====================
const char* ssid = "Tone";
const char* password = "12345678";

// ===================== WEB =====================
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// ===================== LOOP CONFIG =====================
const unsigned long SEND_RATE_MS = 100;
const unsigned long FFT_RATE_MS  = 3000;

// ===================== FUNCTION PROTOTYPE =====================
String getContentType(String filename);
bool handleFileRequest(String path);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);

// ================== PID CONTROL ==================
//float setpoint = 25;   
float Kp = 10.0, Ki = 1.5, Kd = 5.0;



float pid_integral = 0;
float pid_lastError = 0;
unsigned long pid_lastTime = 0;

float angleX = 0;   // đã lọc


// ================== MOTOR CONTROL ==================
void forward(int speed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(0, constrain(speed, 0, 255));
}

void backward(int speed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(0, constrain(speed, 0, 255));
}

void stopMotor() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(0, 0);
}
// ================== PID CONTROL ==================
void balancePID(float angle) {
    unsigned long now = millis();
    float dt = (now - pid_lastTime) / 1000.0;
    if (dt <= 0 || dt > 0.1) {
        pid_lastTime = now;
        return;
    }
    pid_lastTime = now;

    // ===== SETPOINT =====
    float setpoint = 10.0;  // cân bằng tại 0°

    // ===== ERROR =====
    float error = setpoint - angle;   // đơn vị: độ

    // ===== INTEGRAL (chống windup) =====
    pid_integral += error * dt;
    pid_integral = constrain(pid_integral, -20.0, 20.0);

    // ===== DERIVATIVE =====
    float derivative = (error - pid_lastError) / dt;
    pid_lastError = error;

    // ===== PID OUTPUT =====
    float output = Kp * error + Ki * pid_integral + Kd * derivative;
    output = constrain(output, -255, 255);

    // ===== DEAD ZONE =====
    if (abs(error) < 10) {
        stopMotor();
        return;
    }

    // ===== MOTOR DRIVE =====
    if (output > 10) {
        forward((int)output);
    } else {
        backward((int)(-output));
    }
}


void updateAngleAndPID() {
    int16_t ax, ay, az;
    readAccel(ax, ay, az);

    float ax_g = ax / ACCEL_SCALE;
    float ay_g = ay / ACCEL_SCALE;
    float az_g = az / ACCEL_SCALE;

    // Góc thô
    float angleX_raw = atan2(ay_g, az_g) * 180.0 / PI;

    // Low-pass filter
    angleX = 0.9 * angleX + 0.1 * angleX_raw;

    // PID
    balancePID(angleX);
}




// ===================== FILE HANDLING =====================
String getContentType(String filename) {
    if (filename.endsWith(".html")) return "text/html";
    if (filename.endsWith(".js")) return "application/javascript";
    if (filename.endsWith(".css")) return "text/css";
    if (filename.endsWith(".png")) return "image/png";
    if (filename.endsWith(".jpg")) return "image/jpeg";
    return "text/plain";
}

bool handleFileRequest(String path) {
    if (path.endsWith("/")) path += "index.html";
    if (!SPIFFS.exists(path)) return false;

    File file = SPIFFS.open(path, "r");
    if (!file) return false;
    server.streamFile(file, getContentType(path));
    file.close();
    return true;
}

// ===================== WEBSOCKET EVENT =====================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
    if (type == WStype_CONNECTED) {
        Serial.printf("Client connected: %s\n", webSocket.remoteIP(num).toString().c_str());
    } 
    else if (type == WStype_DISCONNECTED) {
        Serial.println("Client disconnected");
    }
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    delay(300);

    // ====== MPU ======
    Wire.begin(23, 22);// SDA, SCL
    MPU6050_Init();

    // ====== WIFI ======
    WiFi.begin(ssid, password);
    Serial.println("Connecting WiFi...");

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi failed!");
        while (1) delay(100);
    }

    Serial.println("\nWiFi Connected!");
    Serial.println(WiFi.localIP());

    // ====== SPIFFS ======
    SPIFFS.begin(true);
    server.onNotFound([]() {
        if (!handleFileRequest(server.uri())) server.send(404, "text/plain", "File Not Found");
    });
    server.begin();

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);


    // ====== Motor ======
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    ledcSetup(0, 20000, 8);
    ledcAttachPin(ENA, 0);
    stopMotor();
    
    // ====== Encoder ======
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), handleEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), handleEncoder, CHANGE);

      Serial.println("System Ready!");
}

// ===================== LOOP =====================
void loop() {
    server.handleClient();
    webSocket.loop();

    // ================== TIMER ==================
    static unsigned long lastSend = 0;
    static unsigned long lastFFT  = 0;
    static unsigned long lastPID  = 0;

    // ================== BIẾN CHUNG ==================
    static double rmsValue = 0;
    static double peakFreq = 0;
    static double peakAmp  = 0;

    // =================================================
    // ========== 1. PID GÓC – CHẠY NHANH (100 Hz) ======
    // =================================================
     

if (millis() - lastPID >= 10) { // 100 Hz
    lastPID = millis();
    updateAngleAndPID();
}


    // =================================================
    // ========== 2. GỬI DỮ LIỆU REALTIME (100 ms) ======
    // =================================================
    if (millis() - lastSend >= SEND_RATE_MS) {
        lastSend = millis();

        int16_t ax, ay, az;
        readAccel(ax, ay, az);

        float ax_g = ax / ACCEL_SCALE;
        float ay_g = ay / ACCEL_SCALE;
        float az_g = az / ACCEL_SCALE;

        float angleX = atan2(ay_g, az_g) * 180.0 / PI;
        float angleY = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0 / PI;

        String json = "{";
        json += "\"ax_g\":" + String(ax_g, 3) + ",";
        json += "\"ay_g\":" + String(ay_g, 3) + ",";
        json += "\"az_g\":" + String(az_g, 3) + ",";
        json += "\"angleX\":" + String(angleX, 2) + ",";
        json += "\"angleY\":" + String(angleY, 2) + ",";
        json += "\"rms\":" + String(rmsValue, 4);
        json += "}";

        webSocket.broadcastTXT(json);
    }

    // =================================================
    // ========== 3. FFT – CHẠY CHẬM (2 GIÂY) ===========
    // =================================================
    if (millis() - lastFFT >= 2000) {
        lastFFT = millis();

        collectSamples();
        rmsValue = computeRMS();
        peakFreq = computeFFT_Frequency();

        peakAmp = 0;
        for (int i = 1; i < SAMPLE_SIZE / 2; i++)
            if (vReal[i] > peakAmp) peakAmp = vReal[i];

        String json = "{";
json += "\"fft\":true,";
json += "\"rms\":" + String(rmsValue, 4) + ",";
json += "\"peak_freq\":" + String(peakFreq, 2) + ",";
json += "\"peak_amp\":" + String(peakAmp, 4) + ",";

json += "\"fft_spectrum_x\":[";
for (int i = 0; i < SAMPLE_SIZE / 2; i++) {
    json += String(vRealX[i], 4);
    if (i < SAMPLE_SIZE / 2 - 1) json += ",";
}
json += "],";

json += "\"fft_spectrum_y\":[";
for (int i = 0; i < SAMPLE_SIZE / 2; i++) {
    json += String(vRealY[i], 4);
    if (i < SAMPLE_SIZE / 2 - 1) json += ",";
}
json += "],";

json += "\"fft_spectrum_z\":[";
for (int i = 0; i < SAMPLE_SIZE / 2; i++) {
    json += String(vRealZ[i], 4);
    if (i < SAMPLE_SIZE / 2 - 1) json += ",";
}
json += "]";

json += "}";

webSocket.broadcastTXT(json);

    }
}
