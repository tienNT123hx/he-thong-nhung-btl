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
float setpoint = 20;   
float Kp = 8.0, Ki = 0.30, Kd = 4.0;

float integral = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;
unsigned long lastPID = 0;

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
    float dt = (now - lastPID) / 1000.0;
    if (dt <= 0) return;
    lastPID = now;

    // ====== SETPOINT ======
    float setpoint = 25.0;   // cân bằng

    // ====== ERROR ======
    float error = setpoint - angle;   // ĐƠN VỊ: độ (°)

    // ====== INTEGRAL ======
    integral += error * dt;
    integral = constrain(integral, -30.0, 30.0);  // chống windup

    // ====== DERIVATIVE ======
    float derivative = (error - lastError) / dt;
    lastError = error;

    // ====== PID OUTPUT ======
    float output = Kp * error + Ki * integral + Kd * derivative;

    output = constrain(output, -255, 255);

    // ====== DEADZONE ======
    if (abs(error) < 25) {   // lệch < 0.5°
        stopMotor();
        return;
    }

    // ====== MOTOR CONTROL ======
    if (output > 25) {
        forward(abs(output));
    } else {
        backward(abs(output));
    }
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
    if (millis() - lastPID >= 10) {   // 10 ms
        lastPID = millis();

        int16_t ax, ay, az;
        readAccel(ax, ay, az);

        float ax_g = ax / ACCEL_SCALE;
        float ay_g = ay / ACCEL_SCALE;
        float az_g = az / ACCEL_SCALE;

        // Roll angle (trục X)
        float angleX = atan2(ay_g, az_g) * 180.0 / PI;

        balancePID(angleX);   
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
        json += "\"fft_spectrum_z\":[";

        for (int i = 0; i < SAMPLE_SIZE / 2; i++) {
            json += String(vReal[i], 4);
            if (i < SAMPLE_SIZE / 2 - 1) json += ",";
        }
        json += "]}";

        webSocket.broadcastTXT(json);
    }
}
