// =====================================================================
//  ТЭН Controller v1.2 — 1 канал + Wi-Fi
//  Управление 1 ТЭН (1 кВт, 220 В)
//  Датчик: К-термопара через MAX6675
//  Контроль тока: токовое кольцо (ADC)
//  Индикация: Жёлтый / Зелёный / Красный через 74HC595
//  Wi-Fi: точка доступа "Thermo", веб-интерфейс
//  MCU: ESP-32 Pocket 32 (Dongsen Tech)
// =====================================================================

#include <WiFi.h>
#include <WebServer.h>
// Captive portal убит
#include <max6675.h>

// ======================== КОНФИГУРАЦИЯ ================================

// --- Wi-Fi ---
const char *AP_SSID     = "Thermo";
const char *AP_PASS     = "";           // Пустой = открытая точка

// --- Пины MAX6675 ---
#define MAX6675_SCK       18
#define MAX6675_SO        19
#define MAX6675_CS         5

MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);

// --- Пин реле (active-LOW) ---
#define RELAY_PIN         21

// --- Пин токового кольца (ADC1) ---
#define CURRENT_PIN       34

// --- Пины 74HC595 (3 светодиода) ---
#define SR_DS             26
#define SR_SHCP           27
#define SR_STCP           14

// --- Температурные пороги (°C) ---
#define TEMP_OFF          450.0
#define TEMP_ON           435.0

// --- Ток ---
#define CURRENT_THRESH    100
#define CURRENT_CHECK     0    // 0 = отключить, 1 = включить

// --- Тайминги (мс) ---
#define READ_INTERVAL     500
#define FAULT_DELAY       3000
#define THERMO_ERR_MAX    5

// ======================== ТИПЫ ДАННЫХ =================================

enum HeaterState : uint8_t {
  ST_IDLE         = 0,
  ST_HEATING      = 1,
  ST_IN_RANGE     = 2,
  ST_COOLING      = 3,
  ST_FAULT        = 4,
  ST_THERMO_ERR   = 5
};

struct Heater {
  uint8_t  csPin;
  uint8_t  relayPin;
  uint8_t  currentPin;
  float    temperature;
  int      rawCurrent;
  bool     thermoOk;
  bool     relayOn;
  HeaterState state;
  unsigned long relayOnTime;
  uint8_t  thermoErrCount;
  bool     faultLatched;
};

// ======================== ГЛОБАЛЬНЫЕ ДАННЫЕ ===========================

static Heater h;
WebServer server(80);
// DNSServer убит

// ======================== ТЕРМОПАРА ====================================

float readThermocouple() {
  return thermocouple.readCelsius();
}

// ======================== ТОК ==========================================

int readCurrentAvg(uint16_t samples) {
  (void)analogRead(CURRENT_PIN);
  (void)analogRead(CURRENT_PIN);
  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += (uint32_t)analogRead(CURRENT_PIN);
    delayMicroseconds(200);
  }
  return (int)(sum / samples);
}

// ======================== LED (74HC595) =================================

void updateLEDs() {
  uint8_t mask = 0;
  switch (h.state) {
    case ST_HEATING:    mask = (1 << 0); break;
    case ST_IN_RANGE:   mask = (1 << 1); break;
    case ST_FAULT:      mask = (1 << 2); break;
    case ST_THERMO_ERR:
      if ((millis() / 300) & 1) mask = (1 << 2);
      break;
    default: break;
  }
  digitalWrite(SR_STCP, LOW);
  for (int i = 2; i >= 0; i--) {
    digitalWrite(SR_SHCP, LOW);
    digitalWrite(SR_DS, (mask >> i) & 0x01);
    digitalWrite(SR_SHCP, HIGH);
  }
  digitalWrite(SR_STCP, HIGH);
}

// ======================== РЕЛЕ ==========================================

void setRelay(bool on) {
  h.relayOn = on;
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  if (on) h.relayOnTime = millis();
}

// ======================== ЛОГИКА УПРАВЛЕНИЯ ===========================

void processHeater() {
  if (h.faultLatched) {
    setRelay(false);
    h.state = ST_FAULT;
    return;
  }

  float temp = readThermocouple();
  h.thermoOk = !isnan(temp);
  h.temperature = temp;

  if (!h.thermoOk) {
    h.thermoErrCount++;
    if (h.thermoErrCount >= THERMO_ERR_MAX) {
      setRelay(false);
      h.state = ST_THERMO_ERR;
      return;
    }
  } else {
    h.thermoErrCount = 0;
  }

  h.rawCurrent = readCurrentAvg(10);
  if (CURRENT_CHECK && h.relayOn) {
    if (millis() - h.relayOnTime > FAULT_DELAY) {
      h.rawCurrent = readCurrentAvg(40);
      if (h.rawCurrent < CURRENT_THRESH) {
        h.faultLatched = true;
        setRelay(false);
        h.state = ST_FAULT;
        Serial.printf("[АВАРИЯ] Ток отсутствует! ADC=%d\n", h.rawCurrent);
        return;
      }
    }
  }

  if (!h.thermoOk) return;

  if (h.temperature >= TEMP_OFF) {
    if (h.relayOn) setRelay(false);
    h.state = ST_COOLING;
  } else if (h.temperature <= TEMP_ON) {
    if (!h.relayOn) {
      setRelay(true);
      h.faultLatched = false;
      h.thermoErrCount = 0;
    }
    h.state = ST_HEATING;
  } else {
    h.state = h.relayOn ? ST_HEATING : ST_IN_RANGE;
  }
}

// ======================== СЕРИАЛЬНЫЕ КОМАНДЫ ===========================

void handleCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "RESET") {
    h.faultLatched = false;
    h.thermoErrCount = 0;
    h.state = ST_IDLE;
    Serial.println("[CMD] Авария сброшена");
  } else if (cmd == "STOP") {
    setRelay(false);
    h.state = ST_IDLE;
    h.faultLatched = false;
    Serial.println("[CMD] ТЭН остановлен");
  } else if (cmd == "ON") {
    setRelay(true);
    h.faultLatched = false;
    h.thermoErrCount = 0;
    Serial.println("[CMD] Реле ВКЛ");
  } else if (cmd == "OFF" || cmd == "STOP") {
    setRelay(false);
    h.faultLatched = true;
    h.state = ST_FAULT;
    Serial.println("[CMD] АВАРИЙНЫЙ СТОП");
  } else if (cmd == "STATUS") {
    printStatus();
  }
}

// ======================== ДИАГНОСТИКА (Serial) ========================

const char* stateToStr(HeaterState s) {
  switch (s) {
    case ST_IDLE:       return "ОЖИДАНИЕ";
    case ST_HEATING:    return "НАГРЕВ";
    case ST_IN_RANGE:   return "ДИАПАЗОН";
    case ST_COOLING:    return "ОСТИВАНИЕ";
    case ST_FAULT:      return "АВАРИЯ";
    case ST_THERMO_ERR: return "ОШИБКА Т/П";
    default:            return "???";
  }
}

void printStatus() {
  Serial.println("============ СТАТУС ============");
  if (isnan(h.temperature)) {
    Serial.printf("ТЭН | ERR | Ток:%4d | Реле:%s | %s\n",
                  h.rawCurrent, h.relayOn ? "ВКЛ" : "ВЫКЛ", stateToStr(h.state));
  } else {
    Serial.printf("ТЭН | %3dC | Ток:%4d | Реле:%s | %s\n",
                  (int)h.temperature, h.rawCurrent, h.relayOn ? "ВКЛ" : "ВЫКЛ", stateToStr(h.state));
  }
  Serial.println("================================");
}

// ======================== ВЕБ-ИНТЕРФЕЙС ================================

// API: JSON со статусом
void handleApi() {
  String json = "{";
  if (isnan(h.temperature)) {
    json += "\"temp\":null,";
  } else {
    json += "\"temp\":" + String((int)h.temperature) + ",";
  }
  json += "\"relay\":" + String(h.relayOn ? "true" : "false") + ",";
  json += "\"current\":" + String(h.rawCurrent) + ",";
  json += "\"state\":\"" + String(stateToStr(h.state)) + "\",";
  json += "\"uptime\":" + String(millis() / 1000);
  json += "}";
  server.send(200, "application/json", json);
}

// API: управление — только аварийный стоп
void handleCmd() {
  if (server.hasArg("stop")) {
    setRelay(false);
    h.faultLatched = true;
    h.state = ST_FAULT;
    server.send(200, "text/plain", "OK: stop");
  } else {
    server.send(400, "text/plain", "Unknown command");
  }
}

// Главная страница
void handleRoot() {
  // Цвет состояния
  const char* stateColor;
  switch (h.state) {
    case ST_HEATING:    stateColor = "#f59e0b"; break;  // жёлтый
    case ST_IN_RANGE:   stateColor = "#22c55e"; break;  // зелёный
    case ST_FAULT:      stateColor = "#ef4444"; break;  // красный
    case ST_THERMO_ERR: stateColor = "#ef4444"; break;  // красный
    case ST_COOLING:    stateColor = "#6b7280"; break;  // серый
    default:            stateColor = "#6b7280"; break;
  }

  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Thermo Control</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,system-ui,sans-serif;background:#0f172a;color:#e2e8f0;min-height:100vh;display:flex;align-items:center;justify-content:center}
.card{background:#1e293b;border-radius:16px;padding:32px;text-align:center;min-width:300px;box-shadow:0 4px 24px rgba(0,0,0,0.4)}
.temp{font-size:72px;font-weight:700;line-height:1.1;margin:16px 0}
.state{font-size:18px;padding:6px 16px;border-radius:20px;display:inline-block;margin:8px 0}
.info{font-size:13px;color:#94a3b8;margin-top:12px}
.btn{display:inline-block;padding:10px 20px;margin:4px;border:none;border-radius:8px;font-size:14px;font-weight:600;cursor:pointer;transition:opacity .15s}
.btn:hover{opacity:0.85}
.btn:active{opacity:0.7}
.btn-on{background:#22c55e;color:#fff}
.btn-off{background:#ef4444;color:#fff}
.btn-rst{background:#3b82f6;color:#fff}
</style>
</head>
<body>
<div class="card">
<h2 style="color:#94a3b8;font-weight:400;font-size:14px;letter-spacing:1px">THERMO CONTROL</h2>
<div class="temp" id="temp">--.-</div>
<div class="state" id="state" style="background:)rawliteral";
  html += String(stateColor) + ";color:#fff\">";
  html += String(stateToStr(h.state));
  html += R"rawliteral(</div>
<div class="info">
  <div>Реле: <span id="relay">--</span></div>
  <div>Ток ADC: <span id="current">--</span></div>
  <div>Uptime: <span id="uptime">--</span></div>
</div>
<div style="margin-top:20px">
  <button class="btn btn-off" onclick="cmd('stop')">СТОП</button>
</div>
</div>
<script>
function cmd(c){var x=new XMLHttpRequest();x.open('GET','/cmd?'+c,1);x.send()}
function upd(){
  var x=new XMLHttpRequest();
  x.onload=function(){
    var d=JSON.parse(x.responseText);
    document.getElementById('temp').textContent=d.temp!==null?d.temp+' °C':'ОШИБКА';
    document.getElementById('state').textContent=d.state;
    var sc='#6b7280';
    if(d.state==='НАГРЕВ')sc='#f59e0b';
    if(d.state==='ДИАПАЗОН')sc='#22c55e';
    if(d.state==='АВАРИЯ'||d.state==='ОШИБКА Т/П')sc='#ef4444';
    document.getElementById('state').style.background=sc;
    document.getElementById('relay').textContent=d.relay?'ВКЛ':'ВЫКЛ';
    document.getElementById('current').textContent=d.current;
    var u=d.uptime;var m=Math.floor(u/60);var s=u%60;
    document.getElementById('uptime').textContent=(m<10?'0':'')+m+':'+(s<10?'0':'')+s;
  };
  x.open('GET','/api',1);x.send();
}
setInterval(upd,1000);
upd();
</script>
</body>
</html>)rawliteral";

  server.send(200, "text/html", html);
}

// ======================== ИНИЦИАЛИЗАЦИЯ ===============================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n========================================");
  Serial.println("   ТЭН Controller v1.2 — 1 канал + Wi-Fi");
  Serial.println("   ESP-32 Pocket 32 (Dongsen Tech)");
  Serial.println("========================================\n");

  // --- Wi-Fi точка доступа ---
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  Serial.printf("Wi-Fi AP: \"%s\"\n", AP_SSID);
  Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());

  // --- Веб-сервер ---
  server.on("/", handleRoot);
  server.on("/api", handleApi);
  server.on("/cmd", handleCmd);



  server.begin();
  Serial.println("HTTP server started");



  // --- Реле (safe state = ВЫКЛ) ---
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  // --- 74HC595 ---
  pinMode(SR_DS, OUTPUT);
  pinMode(SR_SHCP, OUTPUT);
  pinMode(SR_STCP, OUTPUT);
  digitalWrite(SR_DS, LOW);
  digitalWrite(SR_SHCP, LOW);
  digitalWrite(SR_STCP, LOW);
  for (int i = 0; i < 3; i++) {
    digitalWrite(SR_SHCP, LOW);
    digitalWrite(SR_DS, LOW);
    digitalWrite(SR_SHCP, HIGH);
  }
  digitalWrite(SR_STCP, HIGH);

  // --- ADC ---
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // --- Канал ---
  h.csPin          = MAX6675_CS;
  h.relayPin       = RELAY_PIN;
  h.currentPin     = CURRENT_PIN;
  h.temperature    = NAN;
  h.rawCurrent     = 0;
  h.thermoOk       = false;
  h.relayOn        = false;
  h.state          = ST_IDLE;
  h.relayOnTime    = 0;
  h.thermoErrCount = 0;
  h.faultLatched   = false;

  // --- Первичное чтение ---
  float t = readThermocouple();
  if (!isnan(t)) {
    h.temperature = t;
    Serial.printf("Термопара: %.1f C\n", t);
  } else {
    Serial.println("Термопара: ОБРЫВ");
  }

  Serial.println("Запуск...\n");
}

// ======================== ГЛАВНЫЙ ЦИКЛ ===============================

static unsigned long lastRead = 0;

void loop() {
  server.handleClient();
  handleCommands();

  if (millis() - lastRead >= READ_INTERVAL) {
    lastRead = millis();
    processHeater();
    updateLEDs();
    printStatus();
  }
}
