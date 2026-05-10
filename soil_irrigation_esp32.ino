/*
 * ============================================================
 *   AUTOMATED SOIL IRRIGATION SYSTEM
 *   Hardware: ESP32 + Soil Moisture Sensor + Water Level Sensor
 *            + DHT11 + 16x2 LCD (I2C) + Water Pump (Relay)
 * ============================================================
 *
 * PIN CONFIGURATION:
 *  - Soil Moisture Sensor (AO) --> GPIO34 (ADC1_CH6)
 *  - Water Level Sensor  (AO) --> GPIO35 (ADC1_CH7)
 *  - DHT11 Data Pin           --> GPIO4
 *  - Relay (Pump control)     --> GPIO26
 *  - LCD SDA                  --> GPIO21 (I2C SDA)
 *  - LCD SCL                  --> GPIO22 (I2C SCL)
 *  - LCD I2C Address          --> 0x27 (common default)
 *
 * LIBRARY DEPENDENCIES (Install via Arduino Library Manager):
 *  - LiquidCrystal_I2C  by Frank de Brabander
 *  - DHT sensor library  by Adafruit
 *  - Adafruit Unified Sensor by Adafruit
 *
 * AUTHOR  : Shaktimaan
 * VERSION : 1.0
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ─── PIN DEFINITIONS ────────────────────────────────────────
#define SOIL_MOISTURE_PIN   34      // Analog input (ADC)
#define WATER_LEVEL_PIN     35      // Analog input (ADC)
#define DHT_PIN             4       // DHT11 data pin
#define RELAY_PIN           26      // Relay IN pin (Active LOW)

// ─── SENSOR & HARDWARE CONFIG ───────────────────────────────
#define DHT_TYPE            DHT11

// ADC raw value thresholds (0–4095 on ESP32 12-bit ADC)
// Soil Moisture: dry soil → HIGH raw value, wet soil → LOW raw value
#define SOIL_DRY_THRESHOLD  2800    // Below this = wet, above = dry → pump ON
#define SOIL_WET_THRESHOLD  1500    // Below this = very wet → pump OFF

// Water Level: low water → LOW raw value
#define WATER_LOW_THRESHOLD 1000    // Below this = water tank too low → stop pump

// ─── LCD CONFIG (I2C) ───────────────────────────────────────
#define LCD_ADDRESS         0x27
#define LCD_COLS            16
#define LCD_ROWS            2

// ─── TIMING CONFIG ──────────────────────────────────────────
#define SENSOR_READ_INTERVAL    2000    // ms between sensor reads
#define LCD_TOGGLE_INTERVAL     3000    // ms between LCD screen toggles
#define PUMP_MIN_ON_TIME        5000    // ms minimum pump ON duration
#define PUMP_MIN_OFF_TIME       10000   // ms minimum pump OFF duration (prevent rapid cycling)

// ─── OBJECTS ────────────────────────────────────────────────
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);

// ─── GLOBAL STATE ───────────────────────────────────────────
struct SensorData {
  int   soilRaw;          // Raw ADC value from soil sensor
  int   waterLevelRaw;    // Raw ADC value from water level sensor
  float temperature;      // °C from DHT11
  float humidity;         // % RH from DHT11
  int   soilPercent;      // Soil moisture mapped 0–100%
  int   waterPercent;     // Water level mapped 0–100%
  bool  dhtError;         // DHT read failure flag
};

SensorData sensors;

bool  pumpRunning       = false;
bool  lowWaterAlert     = false;
unsigned long lastSensorRead   = 0;
unsigned long lastLCDToggle    = 0;
unsigned long pumpStartTime    = 0;
unsigned long pumpStopTime     = 0;
int   lcdScreen         = 0;    // 0 = soil+water, 1 = temp+humidity, 2 = pump status

// ─── CUSTOM LCD CHARACTERS ──────────────────────────────────
byte dropChar[8]  = { 0b00100, 0b00100, 0b01110, 0b11111,
                      0b11111, 0b11111, 0b01110, 0b00000 };
byte tempChar[8]  = { 0b00100, 0b01010, 0b01010, 0b01110,
                      0b11111, 0b11111, 0b01110, 0b00000 };
byte pumpChar[8]  = { 0b00000, 0b01110, 0b11111, 0b11111,
                      0b01110, 0b00100, 0b00100, 0b00000 };

// ─── FUNCTION DECLARATIONS ──────────────────────────────────
void readSensors();
void controlPump();
void updateLCD();
void displayScreen0();
void displayScreen1();
void displayScreen2();
void setPump(bool state);
int  rawToPercent(int raw, int minRaw, int maxRaw, bool invert);
void printBar(int percent, int barLength);
void serialDebug();

// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n=============================");
  Serial.println("  ESP32 IRRIGATION SYSTEM");
  Serial.println("=============================");

  // Configure relay pin - HIGH = pump OFF (active-low relay)
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);  // Ensure pump is OFF at startup

  // Init DHT
  dht.begin();

  // Init LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, dropChar);
  lcd.createChar(1, tempChar);
  lcd.createChar(2, pumpChar);

  // Startup splash screen
  lcd.setCursor(0, 0);
  lcd.print("  IRRIGATION SYS");
  lcd.setCursor(0, 1);
  lcd.print("   Initializing.");
  delay(500);
  lcd.setCursor(14, 1); lcd.print("..");
  delay(800);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   ESP32  v1.0  ");
  lcd.setCursor(0, 1);
  lcd.print("  System Ready! ");
  delay(1200);
  lcd.clear();

  // Force first read immediately
  lastSensorRead = millis() - SENSOR_READ_INTERVAL;

  Serial.println("System Ready. Monitoring started.\n");
}

// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // Read sensors at defined interval
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = now;
    readSensors();
    controlPump();
    serialDebug();
  }

  // Toggle LCD display screens
  if (now - lastLCDToggle >= LCD_TOGGLE_INTERVAL) {
    lastLCDToggle = now;
    lcdScreen = (lcdScreen + 1) % 3;
    lcd.clear();
  }

  updateLCD();
}

// ════════════════════════════════════════════════════════════
/**
 * Read all sensors and populate the sensors struct.
 * Soil moisture & water level: averaged over 5 samples to reduce noise.
 */
void readSensors() {
  // Oversample ADC for noise reduction
  long soilSum = 0, waterSum = 0;
  const int samples = 5;
  for (int i = 0; i < samples; i++) {
    soilSum  += analogRead(SOIL_MOISTURE_PIN);
    waterSum += analogRead(WATER_LEVEL_PIN);
    delay(10);
  }
  sensors.soilRaw      = soilSum  / samples;
  sensors.waterLevelRaw = waterSum / samples;

  // Map to percentages
  // Soil sensor: dry = ~3200 raw, fully submerged = ~500 raw → invert
  sensors.soilPercent  = rawToPercent(sensors.soilRaw,  500,  3200, true);
  // Water level: empty = ~0 raw, full = ~4095 raw → normal
  sensors.waterPercent = rawToPercent(sensors.waterLevelRaw, 200, 3800, false);

  // DHT11 read
  float h = dht.readHumidity();
  float t = dht.readTemperature();  // Celsius

  if (isnan(h) || isnan(t)) {
    sensors.dhtError = true;
    // Keep last known good values
  } else {
    sensors.dhtError   = false;
    sensors.temperature = t;
    sensors.humidity    = h;
  }

  lowWaterAlert = (sensors.waterLevelRaw < WATER_LOW_THRESHOLD);
}

// ════════════════════════════════════════════════════════════
/**
 * Auto irrigation logic:
 *  - Pump ON if: soil is DRY AND water tank has enough water
 *  - Pump OFF if: soil is WET OR water level too low
 *  - Hysteresis + minimum on/off times to prevent relay chatter
 */
void controlPump() {
  unsigned long now = millis();

  bool soilDry = (sensors.soilRaw > SOIL_DRY_THRESHOLD);
  bool soilWet = (sensors.soilRaw < SOIL_WET_THRESHOLD);
  bool waterOk = !lowWaterAlert;

  if (!pumpRunning) {
    // Decide to turn ON
    bool minOffElapsed = (pumpStopTime == 0) ||
                         ((now - pumpStopTime) >= PUMP_MIN_OFF_TIME);
    if (soilDry && waterOk && minOffElapsed) {
      setPump(true);
    }
  } else {
    // Decide to turn OFF
    bool minOnElapsed = ((now - pumpStartTime) >= PUMP_MIN_ON_TIME);
    if (minOnElapsed && (soilWet || !waterOk)) {
      setPump(false);
    }
    // Safety: force OFF if water too low regardless of min time
    if (!waterOk) {
      setPump(false);
    }
  }
}

// ════════════════════════════════════════════════════════════
void setPump(bool state) {
  if (state == pumpRunning) return;  // No change

  pumpRunning = state;
  // Active-low relay: LOW = pump ON, HIGH = pump OFF
  digitalWrite(RELAY_PIN, state ? LOW : HIGH);

  if (state) {
    pumpStartTime = millis();
    Serial.println(">>> PUMP ON  <<<");
  } else {
    pumpStopTime = millis();
    Serial.println(">>> PUMP OFF <<<");
  }
}

// ════════════════════════════════════════════════════════════
/**
 * Update the current LCD screen content.
 * Called every loop iteration so display stays fresh.
 */
void updateLCD() {
  switch (lcdScreen) {
    case 0: displayScreen0(); break;
    case 1: displayScreen1(); break;
    case 2: displayScreen2(); break;
  }
}

// ── Screen 0: Soil Moisture + Water Level ───────────────────
void displayScreen0() {
  lcd.setCursor(0, 0);
  lcd.write(byte(0));   // drop icon
  lcd.print("Soil:");
  // Print percentage with padding
  lcd.setCursor(6, 0);
  if (sensors.soilPercent < 10)  lcd.print("  ");
  else if (sensors.soilPercent < 100) lcd.print(" ");
  lcd.print(sensors.soilPercent);
  lcd.print("%");

  // Status label
  lcd.setCursor(11, 0);
  if      (sensors.soilPercent >= 60) lcd.print(" WET ");
  else if (sensors.soilPercent >= 30) lcd.print(" OK  ");
  else                                 lcd.print(" DRY ");

  // Row 1: Water level
  lcd.setCursor(0, 1);
  lcd.print("H2O:");
  lcd.setCursor(4, 1);
  if (sensors.waterPercent < 10)  lcd.print("  ");
  else if (sensors.waterPercent < 100) lcd.print(" ");
  lcd.print(sensors.waterPercent);
  lcd.print("%");

  lcd.setCursor(9, 1);
  if (lowWaterAlert) lcd.print(" LOW!!! ");
  else               lcd.print("  OK    ");
}

// ── Screen 1: Temperature + Humidity ────────────────────────
void displayScreen1() {
  lcd.setCursor(0, 0);
  lcd.write(byte(1));   // temp icon
  lcd.print("Temp: ");
  lcd.setCursor(6, 0);
  if (sensors.dhtError) {
    lcd.print("ERR     ");
  } else {
    lcd.print(sensors.temperature, 1);
    lcd.print((char)223);  // degree symbol
    lcd.print("C   ");
  }

  lcd.setCursor(0, 1);
  lcd.print("Humid:");
  lcd.setCursor(6, 1);
  if (sensors.dhtError) {
    lcd.print("ERR     ");
  } else {
    lcd.print((int)sensors.humidity);
    lcd.print("% RH    ");
  }
}

// ── Screen 2: Pump Status ────────────────────────────────────
void displayScreen2() {
  lcd.setCursor(0, 0);
  lcd.write(byte(2));   // pump icon
  lcd.print(" PUMP STATUS");

  lcd.setCursor(0, 1);
  if (pumpRunning) {
    lcd.print("  [  RUNNING  ] ");
  } else if (lowWaterAlert) {
    lcd.print("  [  NO WATER ] ");
  } else if (sensors.soilPercent >= 50) {
    lcd.print("  [  SOIL WET ] ");
  } else {
    lcd.print("  [  STANDBY  ] ");
  }
}

// ════════════════════════════════════════════════════════════
/**
 * Map raw ADC value to 0–100% with optional inversion.
 * invert=true  → high raw = low percent (soil moisture: dry sensor = high raw)
 * invert=false → high raw = high percent (water level: full tank = high raw)
 */
int rawToPercent(int raw, int minRaw, int maxRaw, bool invert) {
  raw = constrain(raw, minRaw, maxRaw);
  int pct = map(raw, minRaw, maxRaw, 0, 100);
  return invert ? (100 - pct) : pct;
}

// ════════════════════════════════════════════════════════════
/**
 * Print all sensor data to Serial Monitor for debugging.
 */
void serialDebug() {
  Serial.println("─────────────────────────────────");
  Serial.printf("Soil    : %d raw  → %d%%  [%s]\n",
    sensors.soilRaw, sensors.soilPercent,
    sensors.soilPercent >= 60 ? "WET" : (sensors.soilPercent >= 30 ? "OK" : "DRY"));

  Serial.printf("Water   : %d raw  → %d%%  [%s]\n",
    sensors.waterLevelRaw, sensors.waterPercent,
    lowWaterAlert ? "LOW!" : "OK");

  if (sensors.dhtError) {
    Serial.println("DHT11   : Read Error!");
  } else {
    Serial.printf("Temp    : %.1f °C\n", sensors.temperature);
    Serial.printf("Humidity: %.1f %%RH\n", sensors.humidity);
  }

  Serial.printf("Pump    : %s\n", pumpRunning ? "ON" : "OFF");
  Serial.println("─────────────────────────────────\n");
}
