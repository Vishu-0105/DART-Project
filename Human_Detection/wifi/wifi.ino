#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <MPU6050.h>

// WiFi Setup
const char* ssid = "Juhi's Galaxy";
const char* password = "topcat@123";
WiFiServer server(1234);

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// GPS Setup
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);

// MPU6050 Setup
MPU6050 mpu;

// Flame Sensor
#define FLAME_SENSOR_PIN 34
#define BUZZER_PIN 19

// Ultrasonic Sensor
#define TRIG_PIN 26
#define ECHO_PIN 25

// IR Sensor (Human Detection)
#define IR_SENSOR_PIN 32

// Motor Driver (L298N)
#define IN1 4
#define IN2 5
#define IN3 18
#define IN4 23
#define ENA 13
#define ENB 12

// Variables
long duration;
int distance;
int speedValue = 150;

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting...");
    }
    Serial.println("Connected! IP: " + WiFi.localIP().toString());
    server.begin();

    GPS_Serial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Wire.begin();
    mpu.initialize();

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("System Booting...");
    display.display();
    delay(2000);

    pinMode(FLAME_SENSOR_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    stopMotors();
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        Serial.println("Client connected!");
        while (client.connected()) {
            updateSensors(client);
            delay(1000);
        }
        client.stop();
        Serial.println("Client disconnected");
    }
}

void updateSensors(WiFiClient &client) {
    while (GPS_Serial.available() > 0) gps.encode(GPS_Serial.read());

    String gpsInfo = "GPS: ";
    if (gps.location.isValid()) {
        gpsInfo += String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6);
    } else {
        gpsInfo += "No Signal";
    }

    // Read MPU6050 Data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    String mpuData = "MPU6050 -> Acc: (" + String(ax) + ", " + String(ay) + ", " + String(az) + ") Gyro: (" + String(gx) + ", " + String(gy) + ", " + String(gz) + ")";

    int flameState = digitalRead(FLAME_SENSOR_PIN);
    int humanDetected = digitalRead(IR_SENSOR_PIN);
    distance = measureDistance();

    // Update OLED Display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Status: Monitoring");
    display.println(gpsInfo);
    display.println("MPU Data:");
    display.println("Acc: " + String(ax) + "," + String(ay) + "," + String(az));
    display.println("Gyro: " + String(gx) + "," + String(gy) + "," + String(gz));
    display.print("Dist: "); display.print(distance); display.println(" cm");
    display.print("Flame: "); display.println(flameState == LOW ? "DETECTED!" : "Safe");
    display.print("Human: "); display.println(humanDetected == LOW ? "Detected!" : "None");
    display.display();

    // Send Data to Client
    client.print(gpsInfo + " | " + mpuData + " | Distance: " + String(distance) + " cm | Flame: " + (flameState == LOW ? "DETECTED!" : "Safe") + " | Human: " + (humanDetected == LOW ? "Detected!" : "None") + "\n");

    if (humanDetected == LOW) buzzThreeTimes();
    controlMotors();
}

long measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH, 20000);
    return duration == 0 ? -1 : duration * 0.034 / 2;
}

void buzzThreeTimes() {
    for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, 2000);
        delay(200);
        noTone(BUZZER_PIN);
        delay(200);
    }
}

void controlMotors() {
    if (distance > 20) {
        moveForward();
    } else if (distance > 10) {
        stopMotors();
    } else {
        moveBackward();
        delay(500);
        turnRight();
        delay(500);
        stopMotors();
    }
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speedValue);
    analogWrite(ENB, speedValue);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speedValue);
    analogWrite(ENB, speedValue);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}