#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MPU6050.h>

// Configurações WiFi
const char* ssid = "SEU_WIFI";
const char* password = "SUA_SENHA";

// Configurações Supabase - SECURE VERSION
const char* supabaseUrl = "https://fhpirrbqwvzjekayezbz.supabase.co";
const char* supabaseKey = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImZocGlycmJxd3Z6amVrYXllemJ6Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NTQ5OTgyNjMsImV4cCI6MjA3MDU3NDI2M30.sBJoCmbuqf5YhgAYgGKQC4a2TjWvinGTwOaFQznMuj4";
// SECURITY NOTE: In production, store this token securely and rotate it regularly
const char* deviceToken = "ESP32_SECURE_TOKEN_2024";

// Pinos dos sensores
#define ONE_WIRE_BUS 4
#define TRIG_PIN 5
#define ECHO_PIN 18
#define TRIG_PIN_2 19
#define ECHO_PIN_2 21

// Configurações do MPU6050
#define MPU6050_ACCEL_RANGE MPU6050_ACCEL_FS_2  // ±2g range
#define MPU6050_GYRO_RANGE MPU6050_GYRO_FS_250   // ±250°/s
#define CALIBRATION_SAMPLES 1000
#define FILTER_ALPHA 0.8  // Filtro passa-baixa
#define VIBRATION_THRESHOLD 0.3  // Limite de vibração (g)

// Inicializar sensores
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temperatureSensor(&oneWire);
MPU6050 mpu;

// Variáveis de calibração do MPU6050
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Variáveis para filtro de dados
float filteredVibration = 0;
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000; // Leitura a cada 1 segundo

// Variáveis para cálculo de velocidade
float previousAccelZ = 0;
float velocity = 0;
unsigned long previousTime = 0;

// Estrutura para armazenar leituras do MPU6050
struct MPUData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float totalAccel;
  float vibrationLevel;
  float velocity;
};

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando sistema de monitoramento de elevador...");
  
  // Inicializar sensores
  temperatureSensor.begin();
  Wire.begin();
  
  // Inicializar e configurar MPU6050
  if (!initializeMPU6050()) {
    Serial.println("Erro ao inicializar MPU6050!");
    while(1);
  }
  
  // Configurar pinos ultrassônicos
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  
  // Conectar WiFi
  connectToWiFi();
  
  // Calibrar MPU6050
  calibrateMPU6050();
  
  Serial.println("Sistema pronto para monitoramento!");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Ler sensores no intervalo definido
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Ler e processar dados dos sensores
    float temperature = readTemperature();
    MPUData mpuData = readMPU6050();
    float distance1 = readUltrasonic(TRIG_PIN, ECHO_PIN);
    float distance2 = readUltrasonic(TRIG_PIN_2, ECHO_PIN_2);
    
    // Aplicar filtro na vibração
    filteredVibration = (FILTER_ALPHA * filteredVibration) + 
                       ((1 - FILTER_ALPHA) * mpuData.vibrationLevel);
    
    // Detectar anomalias
    checkVibrationAnomaly(filteredVibration);
    
    // Enviar dados para Supabase
    sendSensorData("temperature", temperature, "°C");
    sendSensorData("vibration", filteredVibration, "g");
    sendSensorData("velocity", mpuData.velocity, "m/s");
    sendSensorData("ultrasonic_1", distance1, "cm");
    sendSensorData("ultrasonic_2", distance2, "cm");
    
    // Log local para debug
    Serial.printf("Temp: %.1f°C | Vibração: %.3fg | Velocidade: %.2fm/s\n", 
                  temperature, filteredVibration, mpuData.velocity);
    Serial.printf("Dist1: %.1fcm | Dist2: %.1fcm\n", distance1, distance2);
    Serial.printf("Accel X: %.2f, Y: %.2f, Z: %.2f | Total: %.3fg\n",
                  mpuData.accelX, mpuData.accelY, mpuData.accelZ, mpuData.totalAccel);
  }
  
  delay(100); // Pequeno delay para não sobrecarregar o loop
}

bool initializeMPU6050() {
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    return false;
  }
  
  // Configurar ranges do MPU6050 para elevadores
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_RANGE);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_RANGE);
  
  // Configurar filtro passa-baixa interno
  mpu.setDLPFMode(MPU6050_DLPF_BW_20); // Filtro 20Hz
  
  Serial.println("MPU6050 inicializado com sucesso!");
  return true;
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Conectando ao WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi conectado!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Falha na conexão WiFi!");
  }
}

void calibrateMPU6050() {
  Serial.println("Iniciando calibração do MPU6050...");
  Serial.println("Mantenha o sensor imóvel por 10 segundos!");
  
  delay(2000); // Aguardar estabilização
  
  long sumAX = 0, sumAY = 0, sumAZ = 0;
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    sumAX += ax;
    sumAY += ay;
    sumAZ += az;
    sumGX += gx;
    sumGY += gy;
    sumGZ += gz;
    
    delay(5);
  }
  
  // Calcular offsets
  accelOffsetX = sumAX / CALIBRATION_SAMPLES;
  accelOffsetY = sumAY / CALIBRATION_SAMPLES;
  accelOffsetZ = (sumAZ / CALIBRATION_SAMPLES) - 16384; // -1g para Z
  
  gyroOffsetX = sumGX / CALIBRATION_SAMPLES;
  gyroOffsetY = sumGY / CALIBRATION_SAMPLES;
  gyroOffsetZ = sumGZ / CALIBRATION_SAMPLES;
  
  Serial.println("Calibração concluída!");
  Serial.printf("Offsets - AX:%.0f AY:%.0f AZ:%.0f GX:%.0f GY:%.0f GZ:%.0f\n",
                accelOffsetX, accelOffsetY, accelOffsetZ, 
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

float readTemperature() {
  temperatureSensor.requestTemperatures();
  float temp = temperatureSensor.getTempCByIndex(0);
  
  // Verificar se a leitura é válida
  if (temp == DEVICE_DISCONNECTED_C) {
    Serial.println("Erro: Sensor de temperatura desconectado!");
    return -999;
  }
  
  return temp;
}

MPUData readMPU6050() {
  MPUData data;
  int16_t ax, ay, az, gx, gy, gz;
  
  // Ler dados brutos
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Aplicar calibração e converter para unidades físicas
  data.accelX = (ax - accelOffsetX) / 16384.0; // Converter para g
  data.accelY = (ay - accelOffsetY) / 16384.0;
  data.accelZ = (az - accelOffsetZ) / 16384.0;
  
  data.gyroX = (gx - gyroOffsetX) / 131.0; // Converter para °/s
  data.gyroY = (gy - gyroOffsetY) / 131.0;
  data.gyroZ = (gz - gyroOffsetZ) / 131.0;
  
  // Calcular magnitude total da aceleração
  data.totalAccel = sqrt(data.accelX * data.accelX + 
                        data.accelY * data.accelY + 
                        data.accelZ * data.accelZ);
  
  // Calcular nível de vibração (desvio da aceleração normal 1g)
  data.vibrationLevel = abs(data.totalAccel - 1.0);
  
  // Calcular velocidade usando integração da aceleração Z
  unsigned long currentTime = millis();
  if (previousTime > 0) {
    float deltaTime = (currentTime - previousTime) / 1000.0; // Converter para segundos
    float accelZ_ms2 = data.accelZ * 9.81; // Converter g para m/s²
    
    // Integração simples da aceleração para velocidade
    velocity += accelZ_ms2 * deltaTime;
    
    // Aplicar amortecimento para corrigir drift
    velocity *= 0.98;
  }
  
  data.velocity = abs(velocity);
  previousAccelZ = data.accelZ;
  previousTime = currentTime;
  
  return data;
}

float readUltrasonic(int trigPin, int echoPin) {
  // Enviar pulso
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Medir tempo de resposta
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  
  if (duration == 0) {
    Serial.printf("Timeout no sensor ultrassônico (pinos %d/%d)!\n", trigPin, echoPin);
    return -1;
  }
  
  // Converter para centímetros
  float distance = duration * 0.034 / 2;
  
  // Filtrar leituras inválidas
  if (distance < 2 || distance > 400) {
    Serial.printf("Leitura ultrassônica fora da faixa válida (pinos %d/%d)!\n", trigPin, echoPin);
    return -1;
  }
  
  return distance;
}

void checkVibrationAnomaly(float vibration) {
  static unsigned long lastAlert = 0;
  const unsigned long ALERT_INTERVAL = 30000; // Alertar no máximo a cada 30s
  
  if (vibration > VIBRATION_THRESHOLD) {
    unsigned long currentTime = millis();
    if (currentTime - lastAlert > ALERT_INTERVAL) {
      Serial.printf("ALERTA: Vibração alta detectada! %.3fg (limite: %.3fg)\n", 
                    vibration, VIBRATION_THRESHOLD);
      lastAlert = currentTime;
      
      // Aqui você poderia enviar um alerta específico para o dashboard
      sendAlert("high_vibration", vibration);
    }
  }
}

void sendSensorData(String sensorType, float value, String unit) {
  // Verificar se o valor é válido
  if (value == -999 || value == -1) {
    Serial.println("Valor inválido, não enviando: " + sensorType);
    return;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado! Tentando reconectar...");
    connectToWiFi();
    return;
  }
  
  HTTPClient http;
  http.begin(String(supabaseUrl) + "/rest/v1/sensor_readings");
  http.setTimeout(10000); // Timeout 10s
  
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", supabaseKey);
  http.addHeader("Authorization", "Bearer " + String(supabaseKey));
  http.addHeader("x-device-token", deviceToken);
  
  StaticJsonDocument<256> doc;
  doc["device_id"] = "ESP32_ELEVATOR_01";
  doc["sensor_type"] = sensorType;
  doc["value"] = value;
  doc["unit"] = unit;
  doc["timestamp"] = "now()";
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode == 201) {
    Serial.println("✓ " + sensorType + ": " + String(value) + unit + " enviado");
  } else {
    Serial.printf("✗ Erro ao enviar %s: HTTP %d\n", 
                  sensorType.c_str(), httpResponseCode);
    
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println("Resposta: " + response);
    }
  }
  
  http.end();
}

void sendAlert(String alertType, float value) {
  if (WiFi.status() != WL_CONNECTED) return;
  
  HTTPClient http;
  http.begin(String(supabaseUrl) + "/rest/v1/sensor_readings");
  
  http.addHeader("Content-Type", "application/json");
  http.addHeader("apikey", supabaseKey);
  http.addHeader("Authorization", "Bearer " + String(supabaseKey));
  http.addHeader("x-device-token", deviceToken);
  
  StaticJsonDocument<256> doc;
  doc["device_id"] = "ESP32_ELEVATOR_01";
  doc["sensor_type"] = "alert_" + alertType;
  doc["value"] = value;
  doc["unit"] = "alert";
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  http.POST(jsonString);
  http.end();
  
  Serial.println("Alerta enviado: " + alertType);
}
