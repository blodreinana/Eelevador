
# Anemômetro com ESP32 + Módulo SIM (GPRS) + Sensor de Chuva

Este documento reúne **tudo que você precisa** para configurar, programar e testar um sistema que lê dados de um anemômetro (sensor de vento) e de um sensor de chuva conectados a um ESP32 e envia essas leituras via HTTP para um servidor remoto usando um módulo SIM (operadora Claro) em rede GPRS. 
---

## 🔹 Descrição Geral

- **Objetivo:**  
  1. Medir a frequência de pulso do anemômetro (em Hz).  
  2. Indicar faixas de vento (fraco, moderado, forte, tempestade) por LEDs.  
  3. Ativar sirene e motor em condição de tempestade.  
  4. Ler intensidade de chuva (sensor MH-RD) e classificar em “Sem chuva”, “Garoa”, “Chuva” ou “Chuva forte”.  
  5. Conectar ao GPRS da Claro, montar um _payload_ JSON com as leituras e enviar via HTTP POST para um servidor (PHP, Node.js, Python, etc.).

- **Fluxo de funcionamento:**  
  1. **Inicialização**: configuração de pinos no ESP32 e do módulo SIM (APN, registro na rede, GPRS).  
  2. **Loop principal (a cada 1 segundo)**:  
     - Captura e zera contagem de pulsos do anemômetro → calcula frequência em Hz → aciona LEDs/motor/sirene conforme faixa de vento.  
     - Lê sensor de chuva em ADC (GPIO 34) e em digital (GPIO 4) → classifica intensidade.  
     - Monta JSON com `vento_hz`, `chuva_adc` e `chuva_nivel` → envia via HTTP POST ao servidor remoto.  
  3. **Em tempestade (freq > 9 Hz)**: pisca LEDs vermelho/branco e varia frequência da sirene (600 Hz até 1000 Hz de forma oscilante).

---

## 📦 Estrutura Recomendada do Repositório

```

├── README.md         ← ESTE ARQUIVO (tudo contido aqui)
└── anemometro.cpp    ← Sketch completo (código principal do ESP32)

```

> Neste exemplo, **README.md** conterá todas as instruções e explicações. O arquivo **anemometro.cpp** conterá o código para o ESP32. Se você preferir, pode renomear `anemometro.cpp` para `anemometro.ino` se for usar Arduino IDE.

---

## ⚙️ Requisitos de Hardware

1. ### ESP32  
   - Placa do tipo NodeMCU-32S, Wemos ESP32 ou similar.  

2. ### Módulo SIM GPRS (operadora Claro)  
   - Pode ser SIM800L, SIM900, SIM5320 etc.  
   - **Fonte de alimentação**: capaz de fornecer até 2 A nos picos de transmissão. Normalmente estes módulos usam **4 V**.  
   - **Capacitor de desacoplamento**:  
     - 100 µF (Eletrolítico) e 10 µF (Cerâmico) próximos aos terminais de alimentação do módulo SIM.  
   - **Antena GSM** adequada (conectada ao conector SMA) para garantir bom sinal.  

3. ### Anemômetro com sensor Hall  
   - Provê pulsos por rotação; cada pulso corresponde a um determinado ângulo do rotor.  
   - Conecte o fio de sinal ao **GPIO 26** (pino `hallPin`) do ESP32 e o GND em comum com o ESP32.

4. ### Sensor de chuva MH-RD  
   - Saída analógica (A0) → **GPIO 34** (ADC1_CH6) do ESP32.  
   - Saída digital (D0, comparador interno) → **GPIO 4** do ESP32.  
   - Alimentação do sensor MH-RD (tipicamente 3.3 V do ESP32).  

5. ### Buzzer (piezo) para sirene  
   - Conectado ao **GPIO 25** do ESP32.  

6. ### LEDs para indicar faixas de vento  
   - LED Verde → **GPIO 12** (indica vento fraco, 0–2 Hz).  
   - LED Amarelo → **GPIO 13** (indica vento moderado, 2–5 Hz).  
   - LED Vermelho → **GPIO 32** (indica vento forte, 5–9 Hz).  
   - LED Branco → **GPIO 33** (pisca em tempestade, > 9 Hz).  

7. ### Ponte H (para acionar motor em tempestade)  
   - IN3 → **GPIO 27** do ESP32.  
   - IN4 → **GPIO 14** do ESP32.  
   - Os pinos de saída do driver da ponte H (motor) vão conforme o hardware que você usar (por ex. L298N, L293D etc.).  

---

## 🔌 Diagrama de Ligações (Wiring)

```

ESP32                   Módulo SIM GPRS         Componentes
────────                ───────────────          ────────────

GPIO16 (RX1)  ←─── TXD (SIM TXD)                 │
GPIO17 (TX1)  ───→ RXD (SIM RXD)                 │
GND           ────→ GND                           ├─ Anemômetro (sensor Hall, GND em comum)
4 V (VIN BALANCE) ─→ VCC (SIM800L)               │
└─ Sensor de Chuva MH-RD: A0 em GPIO34, D0 em GPIO4

GPIO26        ───→ Sinal Hall (Anemômetro)       └─ Buzzer (Piezo) em GPIO25
GPIO25        ───→ Buzzer (Piezo Sirene)         └─ LEDs e Ponte H (motor)

GPIO12        ───→ LED Verde (Vento fraco 0–2 Hz)
GPIO13        ───→ LED Amarelo (Vento moderado 2–5 Hz)
GPIO32        ───→ LED Vermelho (Vento forte 5–9 Hz)
GPIO33        ───→ LED Branco (Tempestade, > 9 Hz)

GPIO27        ───→ IN3 (Ponte H)
GPIO14        ───→ IN4 (Ponte H)

GPIO34        ───→ Saída Analógica do Sensor de Chuva (MH-RD A0)
GPIO4         ───→ Saída Digital do Sensor de Chuva (MH-RD D0)

GND (ESP32) ────┐
│
GND (SIM GPRS) ┘

````

> **Importante:**  
> - **Alimentação do módulo SIM** deve ser **4 V** (geralmente obtida de um regulador buck a partir de 5 V).  
> - **Capacitores** (100 µF e 10 µF) próximos ao módulo SIM ajudam a estabilizar a tensão nos picos de corrente.  
> - Os pinos **RX1/TX1** do ESP32 (GPIO 16 e 17) são usados para comunicação AT com o módulo SIM. Ajuste se necessário.  
> - Todos os GNDs devem estar em comum.

---

## 📥 Requisitos de Software

1. **Arduino IDE** (versão 1.8.13 ou superior) ou **PlatformIO**  
2. **Bibliotecas** (via Arduino Library Manager):  
   - **TinyGSM** (https://github.com/vshymanskyy/TinyGSM)  
   - **HTTPClient** (já incluída no core ESP32)  
   - (Opcional) **ArduinoJson**, caso queira usar para montar o JSON de forma mais robusta.  

3. **Configurar o ambiente ESP32**:  
   - No Arduino IDE, abra **Boards Manager**, procure por “ESP32” e instale “ESP32 by Espressif Systems”.  
   - No PlatformIO, no `platformio.ini` inclua algo como:  
     ```
     [env:esp32dev]
     platform = espressif32
     board    = esp32dev
     framework = arduino
     lib_deps = 
       vshymanskyy/TinyGSM
       arduino-libraries/ArduinoJson
     ```

4. **Ser capaz de editar e fazer upload de sketch (.cpp ou .ino) para o ESP32**.

---

## 📝 Arquivo de Configuração (no código)

Dentro do código do ESP32 (abaixo), ajuste:

```cpp
// 1. APN da Claro (Brasil)
const char apn[]  = "claro.com.br";
const char user[] = "";   // geralmente vazio
const char passw[]= "";   // geralmente vazio

// 2. URL do servidor HTTP para receber o POST
const char serverUrl[] = "http://SEU_SERVIDOR.com.br/api/anemometro";

// 3. Pinos RX/TX que ligam ESP32 ao módulo SIM
#define SIM_RX_PIN 16   // Ex.: GPIO16 (RX1) do ESP32
#define SIM_TX_PIN 17   // Ex.: GPIO17 (TX1) do ESP32

// 4. Baud rate do módulo SIM (ajuste se for 9600 ou outro)
#define SIM_BAUD    115200
````

> **Observações Importantes:**
>
> * Se o chip SIM exigir **PIN**, descomente e ajuste:
>
>   ```cpp
>   // SerialSIM.println("AT+CPIN=\"1234\"");
>   // delay(2000);
>   ```
> * Caso seu servidor aceite apenas **parâmetros GET** ao invés de JSON no corpo, substitua o bloco de `POST` por algo como:
>
>   ```cpp
>   String url = String(serverUrl) 
>                + "?vento_hz=" + String(freq, 2) 
>                + "&chuva_adc=" + String(rainRaw);
>   http.begin(client, url);
>   int httpCode = http.GET();
>   ```
> * Módulos SIM800L não suportam **HTTPS** (TLS) sem firmware especial. Use `http://` (sem “s”).

---

## 🖥️ Código Completo para o ESP32 (anemometro.cpp)

```cpp
// ==================================================
//                  Biblioteca TinyGSM
// ==================================================
#define TINY_GSM_MODEM_SIM800   // ou SIM800L, SIM900, SIM5320 etc.

#include <TinyGsmClient.h>
#include <HTTPClient.h>

// ==================================================
//               Definição de Hardware
// ==================================================

// Pinos do anemômetro e atuadores
const uint8_t hallPin         = 26; // Sensor Hall (anemômetro)
const uint8_t buzzerPin       = 25; // Buzzer (sirene de emergência)

const uint8_t ledVerdePin     = 12; // 0–2 Hz → vento fraco
const uint8_t ledAmareloPin   = 13; // 2–5 Hz → vento moderado
const uint8_t ledVermelhoPin  = 32; // 5–9 Hz → vento forte
const uint8_t ledBrancoPin    = 33; // piscante em tempestade

const uint8_t motorIn3Pin     = 27; // IN3 da ponte H
const uint8_t motorIn4Pin     = 14; // IN4 da ponte H

// Sensor de chuva (MH-RD)
const uint8_t rainA0Pin       = 34; // ADC1_CH6 (entrada analógica)
const uint8_t rainDigitalPin  = 4;  // saída digital (comparador)

volatile unsigned int pulseCount = 0; // conta de pulsos do anemômetro
float multiplier                = 2.0; // ajuste caso não alcance 10 Hz

unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 1000; // intervalo de 1 segundo

// Controle de piscar LEDs em tempestade
unsigned long blinkMillis = 0;
bool blinkState         = false;

// Controle de sirene variável
int sirenFreq           = 600;
int sirenDir            = 1;
unsigned long sirenLastMillis = 0;
const int sirenMinFreq   = 600;
const int sirenMaxFreq   = 1000;
const unsigned long sirenIntervalMs = 20;

// Estado do vento: 0=fraco, 1=moderado, 2=forte, 3=tempestade
uint8_t windState = 0;

// ==================================================
//              Configuração do Módulo SIM
// ==================================================
#define SIM_RX_PIN 16   // RX1 (GPIO16) do ESP32
#define SIM_TX_PIN 17   // TX1 (GPIO17) do ESP32
#define SIM_BAUD    115200

HardwareSerial SerialSIM(1);       // Serial1 do ESP32
TinyGsm modem(SerialSIM);         // Instancia TinyGSM com SerialSIM
TinyGsmClient client(modem);      // Cliente TCP/IP embutido no TinyGSM
HTTPClient http;                  // Para requisições HTTP

// APN da Claro (Brasil)
const char apn[]  = "claro.com.br";
const char user[] = "";    // geralmente vazio
const char passw[]= "";    // geralmente vazio

// URL do servidor HTTP que irá receber o JSON
const char serverUrl[] = "http://SEU_SERVIDOR.com.br/api/anemometro";

// ==================================================
//             Função de Interrupção Hall
// ==================================================
void IRAM_ATTR onHallPulse() {
  pulseCount++;
}

// ==================================================
//                       setup()
// ==================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Inicializa pinos de LEDs, buzzer e motor
  pinMode(ledVerdePin,    OUTPUT);
  pinMode(ledAmareloPin,  OUTPUT);
  pinMode(ledVermelhoPin, OUTPUT);
  pinMode(ledBrancoPin,   OUTPUT);

  pinMode(buzzerPin,      OUTPUT);
  pinMode(motorIn3Pin,    OUTPUT);
  pinMode(motorIn4Pin,    OUTPUT);

  // Desliga tudo inicialmente
  digitalWrite(ledVerdePin,    LOW);
  digitalWrite(ledAmareloPin,  LOW);
  digitalWrite(ledVermelhoPin, LOW);
  digitalWrite(ledBrancoPin,   LOW);
  digitalWrite(buzzerPin,      LOW);
  digitalWrite(motorIn3Pin,    LOW);
  digitalWrite(motorIn4Pin,    LOW);

  // Configura anemômetro (sensor Hall)
  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), onHallPulse, RISING);

  // Configura sensor de chuva
  pinMode(rainA0Pin, INPUT);
  pinMode(rainDigitalPin, INPUT);

  // Configura ADC
  analogReadResolution(12);      // resolução 0–4095
  analogSetAttenuation(ADC_11db);// leitura até ~3.3 V

  // Inicia variáveis de tempo
  lastMeasurementTime = millis();
  blinkMillis         = millis();
  sirenLastMillis     = millis();

  // ==================================================
  //            Inicializa módulo SIM/GPRS
  // ==================================================
  Serial.println("=== Inicializando módulo SIM/GPRS ===");
  SerialSIM.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(3000); // Aguarda inicialização do módulo SIM

  // Testa comunicação AT
  Serial.print("Enviando AT… ");
  if (!modem.testAT()) {
    Serial.println("Falha! Verifique conexões e alimentação do módulo SIM.");
    while (true) { delay(1000); }
  }
  Serial.println("OK");

  // Se o chip exigir PIN, descomente e ajuste:
  // SerialSIM.println("AT+CPIN=\"1234\"");
  // delay(2000);

  // Aguarda registro na rede GSM (até 60 s)
  Serial.print("Aguardando registro na rede (até 60 s)… ");
  if (!modem.waitForNetwork(60000L)) {
    Serial.println("Falha ao registrar na rede GSM.");
    while (true) { delay(1000); }
  }
  Serial.println("Registrado!");

  // Conecta ao GPRS usando APN da Claro
  Serial.print("Conectando ao GPRS (APN: ");
  Serial.print(apn);
  Serial.print(")… ");
  bool gprsOK = modem.gprsConnect(apn, user, passw);
  if (!gprsOK) {
    Serial.println("Falha ao conectar GPRS.");
    while (true) { delay(1000); }
  }
  Serial.println("Conectado ao GPRS!");
}

// ==================================================
//                       loop()
// ==================================================
void loop() {
  unsigned long currentMillis = millis();

  // 1) A cada 1 segundo: calcula frequência do vento e lê chuva
  if (currentMillis - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime += measurementInterval;

    // Captura e zera contagem de pulsos do anemômetro
    noInterrupts();
    unsigned int count = pulseCount;
    pulseCount = 0;
    interrupts();

    // Calcula frequência em Hz
    float freq = count * multiplier;
    Serial.print("Vento (Hz): ");
    Serial.println(freq, 2);

    // Aciona LEDs, motor e sirene conforme faixa de Hz
    if (freq <= 2.0) {
      // Vento fraco
      windState = 0;
      digitalWrite(ledVerdePin,    HIGH);
      digitalWrite(ledAmareloPin,  LOW);
      digitalWrite(ledVermelhoPin, LOW);
      digitalWrite(ledBrancoPin,   LOW);
      noTone(buzzerPin);
      digitalWrite(motorIn3Pin, LOW);
      digitalWrite(motorIn4Pin, LOW);
      blinkState = false;
    }
    else if (freq <= 5.0) {
      // Vento moderado
      windState = 1;
      digitalWrite(ledVerdePin,    LOW);
      digitalWrite(ledAmareloPin,  HIGH);
      digitalWrite(ledVermelhoPin, LOW);
      digitalWrite(ledBrancoPin,   LOW);
      noTone(buzzerPin);
      digitalWrite(motorIn3Pin, LOW);
      digitalWrite(motorIn4Pin, LOW);
      blinkState = false;
    }
    else if (freq <= 9.0) {
      // Vento forte
      windState = 2;
      digitalWrite(ledVerdePin,    LOW);
      digitalWrite(ledAmareloPin,  LOW);
      digitalWrite(ledVermelhoPin, HIGH);
      digitalWrite(ledBrancoPin,   LOW);
      noTone(buzzerPin);
      digitalWrite(motorIn3Pin, LOW);
      digitalWrite(motorIn4Pin, LOW);
      blinkState = false;
    }
    else {
      // Tempestade (> 9 Hz)
      windState = 3;

      // Inicia sirene variável
      sirenFreq       = sirenMinFreq;
      sirenDir        = 1;
      sirenLastMillis = currentMillis;
      tone(buzzerPin, sirenFreq);

      // Motor a potência máxima
      digitalWrite(motorIn3Pin, HIGH);
      digitalWrite(motorIn4Pin, LOW);

      // Prepara piscagem de LEDs
      blinkMillis = currentMillis;
      blinkState  = false;
      digitalWrite(ledVerdePin,    LOW);
      digitalWrite(ledAmareloPin,  LOW);
      digitalWrite(ledVermelhoPin, LOW);
      digitalWrite(ledBrancoPin,   LOW);
    }

    // Leitura do sensor de chuva
    int rainRaw = analogRead(rainA0Pin);       // valor ADC 0–4095
    int d0State = digitalRead(rainDigitalPin); // HIGH/LOW

    const char* rainLevel;
    if (rainRaw >= 3500) {
      rainLevel = "Sem chuva";
    }
    else if (rainRaw >= 2000) {
      rainLevel = "Garoa";
    }
    else if (rainRaw >= 1700) {
      rainLevel = "Chuva";
    }
    else {
      rainLevel = "Chuva forte";
    }

    Serial.print("Chuva (ADC): ");
    Serial.print(rainRaw);
    Serial.print(" → Intensidade: ");
    Serial.println(rainLevel);
    Serial.print("D0 (digital): ");
    Serial.println(d0State == HIGH ? "HIGH (acima do limiar)" : "LOW (abaixo do limiar)");
    Serial.println("------------------------------");

    // 2) Envia dados via GPRS para servidor
    if (modem.isGprsConnected()) {
      // Monta JSON manualmente (poderia usar ArduinoJson)
      String body = "{";
      body += "\"vento_hz\":";
      body += String(freq, 2);
      body += ",\"chuva_adc\":";
      body += String(rainRaw);
      body += ",\"chuva_nivel\":\"";
      body += rainLevel;
      body += "\"}";

      Serial.println("Enviando dados para servidor…");
      Serial.print("URL: ");
      Serial.println(serverUrl);

      http.begin(client, serverUrl);
      http.addHeader("Content-Type", "application/json");
      int httpCode = http.POST(body);

      if (httpCode > 0) {
        Serial.print("HTTP status: ");
        Serial.println(httpCode);
        String response = http.getString();
        Serial.print("Resposta do servidor: ");
        Serial.println(response);
      }
      else {
        Serial.print("Falha HTTP, código: ");
        Serial.println(httpCode);
      }
      http.end();
    }
    else {
      Serial.println("GPRS desconectado; tentando reconectar…");
      modem.gprsConnect(apn, user, passw);
    }
  }

  // 3) Se estiver em tempestade, atualiza sirene e piscagem LEDs
  if (windState == 3) {
    // Varia frequência da sirene de 600 a 1000 Hz
    if (currentMillis - sirenLastMillis >= sirenIntervalMs) {
      sirenLastMillis += sirenIntervalMs;
      sirenFreq += sirenDir * 10;
      if (sirenFreq >= sirenMaxFreq) {
        sirenFreq = sirenMaxFreq;
        sirenDir  = -1;
      }
      else if (sirenFreq <= sirenMinFreq) {
        sirenFreq = sirenMinFreq;
        sirenDir  = 1;
      }
      tone(buzzerPin, sirenFreq);
    }

    // Pisca LEDs vermelho/branco a cada 500 ms
    if (currentMillis - blinkMillis >= 500) {
      blinkMillis += 500;
      blinkState = !blinkState;
      if (blinkState) {
        digitalWrite(ledVermelhoPin, HIGH);
        digitalWrite(ledBrancoPin,   LOW);
      }
      else {
        digitalWrite(ledVermelhoPin, LOW);
        digitalWrite(ledBrancoPin,   HIGH);
      }
    }
  }
  else {
    noTone(buzzerPin);
  }
}
```

---

## 🚀 Como Carregar e Testar no ESP32

1. **Clone este repositório** (ou crie um novo e copie todo este conteúdo):

   ```bash
   git clone https://github.com/SEU_USUARIO/SEU_REPOSITORIO.git
   cd SEU_REPOSITORIO
   ```

2. **Abra o Arduino IDE** (ou PlatformIO) e abra o arquivo `anemometro.cpp`. Se você estiver usando Arduino IDE, renomeie `anemometro.cpp` para `anemometro.ino`.

3. **Instale as bibliotecas necessárias** (Menu **Sketch → Include Library → Manage Libraries…**):

   * `TinyGSM`
   * (Opcional) `ArduinoJson`

4. **Ajuste as configurações no código** (APN, serverUrl, pinos RX/TX e baud rate) conforme as instruções na seção “Arquivo de Configuração”.

5. **Compile e envie** o sketch para o ESP32.

6. **Abra o Serial Monitor** em 115200 bps (ou no baud configurado) e observe:

   * Mensagens de inicialização do módulo SIM (“Enviando AT… OK”).
   * Registro na rede GSM (“Registrado!”).
   * Conexão GPRS (“Conectado ao GPRS!”).
   * A cada 1 segundo, impressão de “Vento (Hz): …”, “Chuva (ADC): … → Intensidade: …”.
   * Mensagem “Enviando dados para servidor…” e “HTTP status: 200” se o servidor responder corretamente.

7. **Verifique seu servidor** (PHP/Node.js/Python) para receber os dados.

---

## 🌐 Exemplo de Endpoint no Servidor (PHP)

Este é um exemplo simples de código PHP para receber o JSON e gravar em um arquivo de log:

```php
<?php
// Salve como: api/anemometro/index.php

header("Access-Control-Allow-Origin: *");
header("Content-Type: application/json; charset=UTF-8");

// Lê o corpo JSON
$input = file_get_contents("php://input");
$data  = json_decode($input, true);

if (!$data) {
  http_response_code(400);
  echo json_encode(["error" => "JSON inválido"]);
  exit;
}

$ventoHz    = isset($data["vento_hz"])    ? floatval($data["vento_hz"])    : 0;
$chuvaAdc   = isset($data["chuva_adc"])   ? intval($data["chuva_adc"])     : 0;
$chuvaNivel = isset($data["chuva_nivel"]) ? $data["chuva_nivel"] : "";

$log = date("Y-m-d H:i:s")
     . " | Vento (Hz): $ventoHz"
     . " | Chuva (ADC): $chuvaAdc"
     . " | Nível: $chuvaNivel" . PHP_EOL;

file_put_contents("leituras.txt", $log, FILE_APPEND);

http_response_code(200);
echo json_encode([
  "status"      => "OK",
  "vento_hz"    => $ventoHz,
  "chuva_adc"   => $chuvaAdc,
  "chuva_nivel" => $chuvaNivel
]);
```

* Coloque esse script em `api/anemometro/index.php` no seu servidor.
* Garanta que o diretório tenha permissão de escrita para que `leituras.txt` seja criado/atualizado.

---

## 🛠️ Dicas de Debug e Ajustes Finais

1. **Alimentação do módulo SIM**

   * Módulos GSM exigem picos de até 2 A. Use um regulador de 5 V→4 V com boa capacidade de corrente.
   * Capacitores de 100 µF e 10 µF próximos ao módulo ajudam a estabilizar a tensão.

2. **Teste AT manualmente (via Serial Monitor)**

   * Configure Serial Monitor para a **porta do ESP32**, mas mude para os pinos do SIM (RX1/TX1) e baud igual a `SIM_BAUD`.
   * Envie `AT` e aguarde `OK`.
   * Envie `AT+CSQ` → verifica a força do sinal (ex.: `+CSQ: 15,0`).
   * Envie `AT+CREG?` → verifica registro na rede (ex.: `+CREG: 0,1`).
   * Envie `AT+CGATT?` → verifica se o GPRS está “attached” (ex.: `+CGATT: 1`).

3. **Checar APN e GPRS**

   * Exemplo de comando manual AT para configurar APN:

     ```
     AT+CGDCONT=1,"IP","claro.com.br"
     AT+CGATT=1
     ```
   * Se o seu SIM não conectar ao GPRS, reveja o APN, usuário/senha (se necessário) e saldo/ativação do chip.

4. **HTTP sem SSL**

   * SIM800L tradicional não suporta HTTPS nativamente. Use exclusivamente `http://`.
   * Se precisar de TLS, utilize módulos mais avançados (SIM7600) ou intermedeie com um proxy HTTP local.

5. **Ajustar `multiplier` do anemômetro**

   * Se a contagem de pulsos não corresponder exatamente aos Hz reais, experimente valores de `multiplier` (ex.: 1.8, 2.2 etc.) até calibrar.

6. **Servidor local (para testes)**

   * Se estiver testando localmente (e não tiver IP público), utilize ferramentas como **ngrok** para expor uma URL temporária:

     ```bash
     ngrok http 80
     ```
   * Copie o endereço gerado (ex.: `http://abcdef.ngrok.io/api/anemometro`) e ajuste `serverUrl` no código do ESP32.

---

## 📚 Referências

* [TinyGSM Library (GitHub)](https://github.com/vshymanskyy/TinyGSM)
* [HTTPClient (ESP32 Core)](https://github.com/espressif/arduino-esp32)
* [Documentação de Hardware SIM800L](https://www.elecrow.com/download/SIM800L%20Hardware%20Design%20V1.03.pdf)
* Exemplos de APN Claro:

  * `claro.com.br`
  * `claroflex.com.br` (para algumas regiões)

---

## 📝 Licença

Este projeto está licenciado sob a **MIT License**. Sinta-se à vontade para usar, copiar e modificar conforme sua necessidade.

---

###### Desenvolvido por Mateus Bustamante (2025)

```
```
