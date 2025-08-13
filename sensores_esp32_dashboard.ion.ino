#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MPU6050.h>

// Configurações WiFi
const char* ssid = ""; // Insira nome da rede wifi dentro das ""
const char* password = ""; // Insira senha da rede wifi dentro das ""

// Configurações Supabase
const char* supabaseUrl = "https://fhpirrbqwvzjekayezbz.supabase.co";
const char* supabaseKey = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImZocGlycmJxd3Z6amVrYXllemJ6Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTQ5OTgyNjMsImV4cCI6MjA3MDU3NDI2M30.sBJoCmbuqf5YhgAYgGKQC4a2TjWvinGTwOaFQznMuj4";

// Pinos dos sensores
#define ONE_WIRE_BUS 4
#define TRIG_PIN 5
#define ECHO_PIN 18

// Inicializar sensores
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temperatureSensor(&oneWire);
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  
  // Inicializar sensores
  temperatureSensor.begin();
  Wire.begin();
  mpu.initialize();
  
  // Configurar pinos ultrassônico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Conectar WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando ao WiFi...");
  }
  Serial.println("WiFi conectado!");
}

void loop() {
  // Ler temperatura
  temperatureSensor.requestTemperatures();
  float temperature = temperatureSensor.getTempCByIndex(0);
  
  // Ler acelerômetro
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float vibration = sqrt(ax*ax + ay*ay + az*az) / 16384.0; // Converter para g
  
  // Ler sensor ultrassônico
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // Converter para cm
  
  // Enviar dados
  sendSensorData("temperature", temperature, "°C");
  sendSensorData("vibration", vibration, "g");
  sendSensorData("ultrasonic", distance, "cm");
  
  delay(5000); // Enviar a cada 5 segundos
}

void sendSensorData(String sensorType, float value, String unit) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(String(supabaseUrl) + "/rest/v1/sensor_readings");
    
    http.addHeader("Content-Type", "application/json");
    http.addHeader("apikey", supabaseKey);
    http.addHeader("Authorization", "Bearer " + String(supabaseKey));
    
    StaticJsonDocument<200> doc;
    doc["device_id"] = "ESP32_ELEVATOR_01";
    doc["sensor_type"] = sensorType;
    doc["value"] = value;
    doc["unit"] = unit;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    int httpResponseCode = http.POST(jsonString);
    
    if (httpResponseCode > 0) {
      Serial.println("Dados enviados: " + sensorType + " = " + String(value) + unit);
    } else {
      Serial.println("Erro ao enviar dados: " + String(httpResponseCode));
    }
    
    http.end();
}
}
