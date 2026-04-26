#include <max6675.h>

// Пины подключения
int thermoDO = 19;
int thermoCS = 5;
int thermoCLK = 18;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

float sumTemperatures = 0.0;
int countMeasurements = 0;

unsigned long lastMeasureTime = 0;
unsigned long lastOutputTime = 0;
const unsigned long measureInterval = 333;   // ~3 измерения в секунду
const unsigned long outputInterval = 1000;   // вывод раз в секунду

void setup() {
  Serial.begin(115200);
  // Небольшая задержка для стабилизации, без вывода в порт
  delay(1000);
  lastMeasureTime = millis();
  lastOutputTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Измерения с заданным интервалом
  if (currentMillis - lastMeasureTime >= measureInterval) {
    lastMeasureTime = currentMillis;
    float celsius = thermocouple.readCelsius();

    if (!isnan(celsius)) {
      sumTemperatures += celsius;
      countMeasurements++;
    }
    // Если ошибка — измерение просто пропускается (не влияет на среднее)
  }

  // 2. Вывод усреднённого значения раз в секунду
  if (currentMillis - lastOutputTime >= outputInterval) {
    lastOutputTime = currentMillis;

    if (countMeasurements > 0) {
      float averageTemp = sumTemperatures / countMeasurements;
      int roundedTemp = round(averageTemp);   // округление до целого
      
      // Единственное, что уходит в Serial — целое число
      Serial.println(roundedTemp);
      Serial.println(averageTemp);

      // Сброс накопителей
      sumTemperatures = 0.0;
      countMeasurements = 0;
    }
    // Если измерений не было (например, ошибка) — ничего не выводим,
    // на графике будет пропуск (точка не ставится)
  }
}