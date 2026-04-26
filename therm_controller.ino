// =====================================================================
//  ТЭН Controller v1.0
//  Независимое управление 4 ТЭН (1 кВт, 220 В)
//  Датчики: К-термопары через MAX6675
//  Контроль тока: токовые кольца (ADC)
//  Индикация: Жёлтый / Зелёный / Красный через 74HC595
//  MCU: ESP-32 Pocket 32 (Dongsen Tech)
// =====================================================================

// SPI не подключаем — используем bit-bang для MAX6675

// ======================== КОНФИГУРАЦИЯ ================================

// --- Пины шины SPI для MAX6675 (общие для всех 4 модулей) ---
#define MAX6675_SCK      18    // Тактирование
#define MAX6675_SO       19    // Данные (MISO)
#define MAX6675_CS_1      5    // Выбор чипа — ТЭН #1
#define MAX6675_CS_2     17    // Выбор чипа — ТЭН #2
#define MAX6675_CS_3     16    // Выбор чипа — ТЭН #3
#define MAX6675_CS_4      4    // Выбор чипа — ТЭН #4

// --- Пины управления реле (active-LOW: LOW = реле ВКЛ) ---
#define RELAY_1          21    // ТЭН #1
#define RELAY_2          22    // ТЭН #2
#define RELAY_3          23    // ТЭН #3
#define RELAY_4          13    // ТЭН #4

// --- Пины токовых колец (аналоговый вход, ADC1) ---
#define CURRENT_PIN_1    34    // ADC1_CH6
#define CURRENT_PIN_2    35    // ADC1_CH7
#define CURRENT_PIN_3    36    // ADC1_CH0
#define CURRENT_PIN_4    39    // ADC1_CH3

// --- Пины сдвигового регистра 74HC595 для светодиодов ---
#define SR_DS            26    // Serial Data In
#define SR_SHCP          27    // Shift Register Clock
#define SR_STCP          14    // Storage Register Clock (Latch)

// --- Температурные пороги (градусы Цельсия) ---
#define TEMP_OFF         450.0 // Отключить ТЭН при достижении
#define TEMP_ON          435.0 // Включить ТЭН при снижении

// --- Порог обнаружения тока (единицы ADC, 0–4095) ---
//     Требуется калибровка под конкретные токовые кольца!
//     При 1 кВт / 220 В номинальный ток ~4.5 А.
//     С подстроечным резистором обычно даёт 200–800 единиц ADC.
#define CURRENT_THRESH   100   // Ниже этого — «тока нет»

// --- Тайминги (миллисекунды) ---
#define READ_INTERVAL    500   // Цикл опроса датчиков
#define FAULT_DELAY      3000  // Подтверждение аварии (мс после включения)
#define THERMO_CONVERSION 250  // Минимальная пауза между чтениями MAX6675

// --- Максимальное число последовательных ошибок термопары ---
#define THERMO_ERR_MAX   5     // После N ошибок — блокировка канала

// ======================== ТИПЫ ДАННЫХ =================================

// Состояние канала ТЭН
enum HeaterState : uint8_t {
  ST_IDLE         = 0,  // Начальное состояние / температура ниже 435, ждём
  ST_HEATING      = 1,  // Нагрев (реле ВКЛ, ток ОК)
  ST_IN_RANGE     = 2,  // Диапазон 435–450, реле ВЫКЛ
  ST_COOLING      = 3,  // Выше 450, ждём остывания
  ST_FAULT        = 4,  // АВАРИЯ: реле ВКЛ, ток отсутствует
  ST_THERMO_ERR   = 5   // Ошибка чтения термопары
};

// Дескриптор одного канала
struct Heater {
  // --- Аппаратные пины ---
  uint8_t  csPin;        // CS MAX6675
  uint8_t  relayPin;     // Управление реле
  uint8_t  currentPin;   // Токовое кольцо (ADC)

  // --- Датчики ---
  float    temperature;  // Температура с термопары (°C)
  int      rawCurrent;   // Сырое значение ADC (0–4095)
  bool     thermoOk;     // Флаг валидности чтения термопары

  // --- Управление ---
  bool     relayOn;      // Текущее состояние реле (true = ВКЛ)
  HeaterState state;     // Конечный автомат

  // --- Диагностика ---
  unsigned long relayOnTime;    // Timestamp включения реле (для fault delay)
  uint8_t  thermoErrCount;      // Счётчик ошибок термопары
  bool     faultLatched;        // Авария зафиксирована (требует сброс)
};

// ======================== ГЛОБАЛЬНЫЕ ДАННЫЕ ===========================

static Heater heaters[4];
static const uint8_t csPins[]    = { MAX6675_CS_1, MAX6675_CS_2, MAX6675_CS_3, MAX6675_CS_4 };
static const uint8_t relayPins[] = { RELAY_1, RELAY_2, RELAY_3, RELAY_4 };
static const uint8_t curPins[]   = { CURRENT_PIN_1, CURRENT_PIN_2, CURRENT_PIN_3, CURRENT_PIN_4 };

// ======================== ЧТЕНИЕ ТЕРМОПАРЫ =============================

// Чтение одной термопары через MAX6675 (bit-bang SPI).
// rawOut — если не NULL, туда записывается сырое 16-битное значение.
// Возвращает температуру в °C или NAN при ошибке.
float readThermocouple(uint8_t csPin, uint16_t *rawOut = nullptr) {
  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

  uint16_t raw = 0;
  for (int i = 15; i >= 0; i--) {
    digitalWrite(MAX6675_SCK, LOW);
    delayMicroseconds(1);
    if (digitalRead(MAX6675_SO)) raw |= (1 << i);
    digitalWrite(MAX6675_SCK, HIGH);
    delayMicroseconds(1);
  }
  digitalWrite(csPin, HIGH);

  // Сохранить сырые данные для отладки
  if (rawOut) *rawOut = raw;

  // D2 = 1 → термопара оборвана или не подключена
  if (raw & 0x04) {
    delay(THERMO_CONVERSION);
    return NAN;
  }

  // D3 — не используется в MAX6675
  raw >>= 3;

  // Каждый LSB = 0.25 °C
  delay(THERMO_CONVERSION);
  return raw * 0.25f;
}

// ======================== ЧТЕНИЕ ТОКА ==================================

// Многократное усреднение для подавления помех от ШИМ/сети 50 Гц.
// Возвращает среднее значение ADC (0–4095).
int readCurrentAvg(uint8_t pin, uint16_t samples) {
  // Выбросить первые 2 отсчёта (устранение переходного процесса ADC)
  (void)analogRead(pin);
  (void)analogRead(pin);

  uint32_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += (uint32_t)analogRead(pin);
    delayMicroseconds(200);  // ~5 кГц частота дискретизации
  }
  return (int)(sum / samples);
}

// ======================== УПРАВЛЕНИЕ СВЕТОДИОДАМИ ======================

// 74HC595: 12 светодиодов (3 на канал).
// Порядок бит (от старшего к младшему):
//   Q7 Q6 Q5 | Q4 Q3 Q2 | Q1 Q0 | Q11 Q10 Q9 | Q8
//        ТЭН4       ТЭН3       ТЭН2        ТЭН1
// Внутри канала: Жёлтый(Q0) Зелёный(Q1) Красный(Q2)
//
// ТЭН1: биты 0,1,2   ТЭН2: биты 3,4,5
// ТЭН3: биты 6,7,8   ТЭН4: биты 9,10,11

void updateLEDs() {
  uint16_t mask = 0;

  for (int i = 0; i < 4; i++) {
    uint8_t base = i * 3;

    switch (heaters[i].state) {
      case ST_HEATING:
        mask |= (1 << (base + 0));    // Жёлтый — идёт нагрев
        break;

      case ST_IN_RANGE:
        mask |= (1 << (base + 1));    // Зелёный — в диапазоне
        break;

      case ST_COOLING:
        // Не горит — выше 450, ждём остывания
        break;

      case ST_FAULT:
        mask |= (1 << (base + 2));    // Красный — авария (ТЭН сгорел)
        break;

      case ST_THERMO_ERR:
        // Мигание красным (реализовано через чётные/нечётные циклы)
        if ((millis() / 300) & 1) {
          mask |= (1 << (base + 2));  // Мигающий красный
        }
        break;

      default:
        break;
    }
  }

  // Выдвигаем 12 бит в сдвиговый регистр
  digitalWrite(SR_STCP, LOW);
  for (int i = 11; i >= 0; i--) {
    digitalWrite(SR_SHCP, LOW);
    digitalWrite(SR_DS, (mask >> i) & 0x01);
    digitalWrite(SR_SHCP, HIGH);
  }
  digitalWrite(SR_STCP, HIGH);   // Защёлкнуть результат
}

// ======================== УПРАВЛЕНИЕ РЕЛЕ ==============================

void setRelay(Heater &h, bool on) {
  h.relayOn = on;
  // active-LOW: LOW = реле замкнуто (ТЭН включён)
  digitalWrite(h.relayPin, on ? LOW : HIGH);

  if (on) {
    h.relayOnTime = millis();  // Запомнить время включения для fault delay
  }
}

// ======================== ЛОГИКА УПРАВЛЕНИЯ ===========================

// Конечный автомат для одного канала ТЭН
void processHeater(Heater &h, int index) {
  // ---------- 1. Авария зафиксирована — блокировка ----------
  if (h.faultLatched) {
    setRelay(h, false);
    h.state = ST_FAULT;
    return;
  }

  // ---------- 2. Чтение температуры ----------
  uint16_t rawSPI = 0;
  float temp = readThermocouple(h.csPin, &rawSPI);
  h.thermoOk = !isnan(temp);
  h.temperature = temp;

  // Отладка: сырой SPI + распаковка бит
  bool d2  = (rawSPI >> 2) & 1;   // 1 = термопара оборвана
  bool d3  = (rawSPI >> 3) & 1;   //_DEVICE_ID (всегда 0 у MAX6675)
 int tempBits = rawSPI >> 3;       // Биты D15..D3 = температура (0..4095)
  Serial.printf("[DBG] ТЭН #%d: raw=0x%04X | D2=%d D3=%d tempBits=%d",
                index + 1, rawSPI, d2, d3, tempBits);
  if (!isnan(temp))
    Serial.printf(" -> %.1fC", temp);
  if (d2)
    Serial.print(" OPEN");
  Serial.println();

  if (!h.thermoOk) {
    h.thermoErrCount++;
    if (h.thermoErrCount >= THERMO_ERR_MAX) {
      // Слишком много ошибок — отключить и показать ошибку
      setRelay(h, false);
      h.state = ST_THERMO_ERR;
      return;
    }
  } else {
    h.thermoErrCount = 0;
  }

  // ---------- 3. Проверка тока при включённом реле ----------
  if (h.relayOn) {
    // Не проверять ток сразу после включения (реле ещё переключается)
    if (millis() - h.relayOnTime > FAULT_DELAY) {
      h.rawCurrent = readCurrentAvg(h.currentPin, 40);

      if (h.rawCurrent < CURRENT_THRESH) {
        // Ток отсутствует — ТЭН сгорел или обрыв цепи!
        h.faultLatched = true;
        setRelay(h, false);
        h.state = ST_FAULT;
        Serial.printf("[АВАРИЯ] ТЭН #%d: ток отсутствует! ADC=%d (порог %d)\n",
                      index + 1, h.rawCurrent, CURRENT_THRESH);
        return;
      }
    }
  } else {
    // Реле выключено — всё равно читаем ток для диагностики
    h.rawCurrent = readCurrentAvg(h.currentPin, 20);
  }

  // ---------- 4. Конечный автомат температурного регулирования ----------
  if (!h.thermoOk) return;  // Ждём корректного чтения температуры

  if (h.temperature >= TEMP_OFF) {
    // --- Выше порога отключения ---
    if (h.relayOn) {
      setRelay(h, false);
      Serial.printf("[ТЭН %d] %.1f°C >= %.0f°C -> ОТКЛЮЧЕНИЕ\n",
                    index + 1, h.temperature, TEMP_OFF);
    }
    h.state = ST_COOLING;

  } else if (h.temperature <= TEMP_ON) {
    // --- Ниже порога включения ---
    if (!h.relayOn) {
      setRelay(h, true);
      h.faultLatched = false;      // Сброс аварии (попробуем снова)
      h.thermoErrCount = 0;        // Сброс ошибок термопары
      Serial.printf("[ТЭН %d] %.1f°C <= %.0f°C -> ВКЛЮЧЕНИЕ\n",
                    index + 1, h.temperature, TEMP_ON);
    }
    h.state = ST_HEATING;

  } else {
    // --- В гистерезисном диапазоне (435–450) ---
    if (h.relayOn) {
      h.state = ST_HEATING;        // Ещё не достигли 450, продолжаем
    } else {
      h.state = ST_IN_RANGE;       // Уже ниже 450, ждём снижения до 435
    }
  }
}

// ======================== ДИАГНОСТИКА (Serial) ========================

void printStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 2000) return;  // Раз в 2 секунды
  lastPrint = millis();

  Serial.println("\n============ СТАТУС ============");

  for (int i = 0; i < 4; i++) {
    const char *stateName;
    const char *ledName;

    switch (heaters[i].state) {
      case ST_IDLE:       stateName = "ОЖИДАНИЕ  "; ledName = "---"; break;
      case ST_HEATING:    stateName = "НАГРЕВ    "; ledName = "ЖЁЛТ"; break;
      case ST_IN_RANGE:   stateName = "ДИАПАЗОН  "; ledName = "ЗЕЛЁН"; break;
      case ST_COOLING:    stateName = "ОСТИВАНИЕ "; ledName = "---"; break;
      case ST_FAULT:      stateName = "АВАРИЯ !!!"; ledName = "КРАСН"; break;
      case ST_THERMO_ERR: stateName = "ОШИБКА Т/П"; ledName = "МИГАЮЩ"; break;
      default:            stateName = "НЕИЗВЕСТНО"; ledName = "???"; break;
    }

    if (isnan(heaters[i].temperature)) {
      Serial.printf("ТЭН #%d | ERR   | Ток: %4d | Реле: %s | %s | [%s]\n",
                    i + 1, heaters[i].rawCurrent,
                    heaters[i].relayOn ? "ВКЛ" : "ВЫКЛ",
                    stateName, ledName);
    } else {
      Serial.printf("ТЭН #%d | %5.1fC | Ток: %4d | Реле: %s | %s | [%s]\n",
                    i + 1, heaters[i].temperature, heaters[i].rawCurrent,
                    heaters[i].relayOn ? "ВКЛ" : "ВЫКЛ",
                    stateName, ledName);
    }
  }
  Serial.println("================================\n");
}

// ======================== СЕРИАЛЬНЫЕ КОМАНДЫ ===========================

// Команды управления через Serial Monitor:
//   STATUS        — вывести статус
//   RESET n       — сбросить аварию на канале n (1–4)
//   CAL n         — калибровка: включить ТЭН #n и показать ток
//   STOP n        — экстренное отключение ТЭН #n
//   STOP ALL      — отключить все ТЭН
//   THRESH v      — установить порог тока (0–4095)

void handleCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "STATUS") {
    printStatus();
    return;
  }

  if (cmd.startsWith("RESET ")) {
    int n = cmd.substring(6).toInt();
    if (n >= 1 && n <= 4) {
      heaters[n - 1].faultLatched = false;
      heaters[n - 1].thermoErrCount = 0;
      heaters[n - 1].state = ST_IDLE;
      Serial.printf("[CMD] Авария на ТЭН #%d сброшена\n", n);
    }
    return;
  }

  if (cmd.startsWith("STOP ")) {
    String arg = cmd.substring(5);
    if (arg == "ALL") {
      for (int i = 0; i < 4; i++) {
        setRelay(heaters[i], false);
        heaters[i].state = ST_IDLE;
        heaters[i].faultLatched = false;
      }
      Serial.println("[CMD] Все ТЭН экстренно отключены!");
    } else {
      int n = arg.toInt();
      if (n >= 1 && n <= 4) {
        setRelay(heaters[n - 1], false);
        heaters[n - 1].faultLatched = false;
        heaters[n - 1].state = ST_IDLE;
        Serial.printf("[CMD] ТЭН #%d экстренно отключён\n", n);
      }
    }
    return;
  }

  if (cmd.startsWith("CAL ")) {
    int n = cmd.substring(4).toInt();
    if (n >= 1 && n <= 4) {
      Serial.printf("[КАЛИБРОВКА] Включаем ТЭН #%d. Читаем ток...\n", n);
      setRelay(heaters[n - 1], true);
      delay(FAULT_DELAY + 500);  // Ждём стабилизации

      for (int j = 0; j < 10; j++) {
        int val = readCurrentAvg(heaters[n - 1].currentPin, 50);
        Serial.printf("  ADC=%d  (порог сейчас: %d)\n", val, CURRENT_THRESH);
        delay(500);
      }

      setRelay(heaters[n - 1], false);
      Serial.println("[КАЛИБРОВКА] ТЭН отключён. Установите CURRENT_THRESH.");
    }
    return;
  }

  if (cmd.startsWith("THRESH ")) {
    // Только информационно — порог задаётся константой в коде
    int v = cmd.substring(7).toInt();
    Serial.printf("[INFO] Для изменения порога отредактируйте CURRENT_THRESH в коде (сейчас: %d, вы указали: %d)\n",
                  CURRENT_THRESH, v);
    return;
  }

  Serial.println("[CMD] Неизвестная команда. Доступные: STATUS, RESET n, STOP n|ALL, CAL n, THRESH v");
}

// ======================== ИНИЦИАЛИЗАЦИЯ ===============================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n========================================");
  Serial.println("   ТЭН Controller v1.0");
  Serial.println("   4x ТЭН 1 кВт | ESP-32 Pocket 32");
  Serial.println("========================================\n");

  // --- Инициализация SPI пинов для MAX6675 ---
  pinMode(MAX6675_SCK, OUTPUT);
  digitalWrite(MAX6675_SCK, LOW);
  pinMode(MAX6675_SO, INPUT);

  // --- Инициализация пинов CS термопар ---
  for (int i = 0; i < 4; i++) {
    pinMode(csPins[i], OUTPUT);
    digitalWrite(csPins[i], HIGH);  // Deselect
  }

  // --- Инициализация пинов реле ---
  // ВАЖНО: устанавливаем HIGH сразу при старте!
  // Для active-LOW модулей реле HIGH = реле ВЫКЛ (безопасное состояние)
  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], HIGH);  // Гарантированно ВЫКЛ
  }

  // --- Инициализация сдвигового регистра 74HC595 ---
  pinMode(SR_DS, OUTPUT);
  pinMode(SR_SHCP, OUTPUT);
  pinMode(SR_STCP, OUTPUT);
  digitalWrite(SR_DS, LOW);
  digitalWrite(SR_SHCP, LOW);
  digitalWrite(SR_STCP, LOW);

  // Очистить все светодиоды
  for (int i = 0; i < 12; i++) {
    digitalWrite(SR_SHCP, LOW);
    digitalWrite(SR_DS, LOW);
    digitalWrite(SR_SHCP, HIGH);
  }
  digitalWrite(SR_STCP, HIGH);

  // --- Настройка ADC (Arduino API, совместимо с ESP-IDF 5.x) ---
  analogReadResolution(12);       // 12 бит (0–4095)
  analogSetAttenuation(ADC_11db); // Полный диапазон 0–3.3 В

  // --- Конфигурация каналов ---
  for (int i = 0; i < 4; i++) {
    heaters[i].csPin        = csPins[i];
    heaters[i].relayPin     = relayPins[i];
    heaters[i].currentPin   = curPins[i];
    heaters[i].temperature  = NAN;
    heaters[i].rawCurrent   = 0;
    heaters[i].thermoOk     = false;
    heaters[i].relayOn      = false;
    heaters[i].state        = ST_IDLE;
    heaters[i].relayOnTime  = 0;
    heaters[i].thermoErrCount = 0;
    heaters[i].faultLatched = false;
  }

  // --- Первичное чтение термопар ---
  Serial.println("Проверка термопар...");
  for (int i = 0; i < 4; i++) {
    uint16_t rawSPI = 0;
    float t = readThermocouple(csPins[i], &rawSPI);
    if (isnan(t)) {
      Serial.printf("  ТЭН #%d: ОБРЫВ (raw=0x%04X)\n", i + 1, rawSPI);
    } else {
      heaters[i].temperature = t;
      Serial.printf("  ТЭН #%d: %.1f C (raw=0x%04X) — ОК\n", i + 1, t, rawSPI);
    }
  }

  Serial.println("\nЗапуск основного цикла управления...\n");
  Serial.println("Команды Serial: STATUS | RESET n | STOP n | CAL n | THRESH v\n");
}

// ======================== ГЛАВНЫЙ ЦИКЛ ===============================

void loop() {
  // 1. Обработка команд из Serial Monitor
  handleCommands();

  // 2. Чтение датчиков и логика управления для каждого канала
  for (int i = 0; i < 4; i++) {
    processHeater(heaters[i], i);
  }

  // 3. Обновление индикации
  updateLEDs();

  // 4. Периодическая диагностика в Serial
  printStatus();

  // 5. Пауза до следующего цикла
  delay(READ_INTERVAL);
}
