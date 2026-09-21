#include <Arduino.h>

#include <Wire.h>
#include <VL53L1X.h>
#include <DFRobotDFPlayerMini.h>
#include "driver/rtc_io.h"
#include <sys/time.h>

#define LED_PIN             2
#define PIXEL_PIN           4
#define PLAYER_RX_PIN       16
#define PLAYER_TX_PIN       17
#define PLAYER_POWER_PIN    25
#define PLAYER_VOLUME       20
#define TRACK_DOOR_OPEN          1
#define TRACK_DOOR_CLOSED        2
#define TRACK_DOOR_OPEN_LOW_BATT 3
#define ACCEL_INT_PIN       GPIO_NUM_33
#define BATT_ADC_PIN        34
#define BATT_DIVIDER        3.128f
#define BATT_LOW_MV         4900
#define BATT_ABSENT_MV      3000
#define DOOR_OPEN_MM        2000
#define STATE_CONFIRM_COUNT 3
#define IDLE_TIMEOUT_MS     300000
#define SLEEP_BACKUP_S      60
#define BLINK_INTERVAL_MS   300
#define AWAKE_MA            45
#define SLEEP_MA            10

struct Color {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

const Color OFF    = {0, 0, 0};
const Color RED    = {60, 0, 0};
const Color ORANGE = {60, 25, 0};
const Color YELLOW = {60, 60, 0};
const Color GREEN  = {0, 60, 0};
const Color BLUE   = {0, 0, 60};

const Color GREEN_DIM = {0, 10, 0};

const Color DOOR_OPEN_COLOR   = ORANGE;
const Color DOOR_CLOSED_COLOR = GREEN_DIM;

VL53L1X sensor;
DFRobotDFPlayerMini player;
bool playerReady = false;

RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t doorEvents = 0;
RTC_DATA_ATTR uint64_t awakeUs = 0;
RTC_DATA_ATTR uint64_t sleepUs = 0;
RTC_DATA_ATTR uint64_t sleepStartUs = 0;
RTC_DATA_ATTR uint32_t dwellCount = 0;
RTC_DATA_ATTR uint32_t dwellSumMs = 0;
RTC_DATA_ATTR uint32_t dwellMinMs = 0xFFFFFFFF;
RTC_DATA_ATTR uint32_t dwellMaxMs = 0;

unsigned long openedAt = 0;

uint64_t bootUs = 0;

bool battLow = false;

bool doorOpen = false;
uint8_t pendingCount = 0;
unsigned long lastEvent = 0;
unsigned long lastBlink = 0;
bool blinkOn = false;
Color shownColor = {255, 255, 255};

void setLight(Color c) {
  if (c.r == shownColor.r && c.g == shownColor.g && c.b == shownColor.b) {
    return;
  }
  shownColor = c;
  digitalWrite(LED_PIN, (c.r | c.g | c.b) > 0);
  neopixelWrite(PIXEL_PIN, c.r, c.g, c.b);
}

void updateLight() {
  if (!doorOpen) {
    setLight(DOOR_CLOSED_COLOR);
    return;
  }
  if (millis() - lastBlink >= BLINK_INTERVAL_MS) {
    lastBlink = millis();
    blinkOn = !blinkOn;
  }
  setLight(blinkOn ? DOOR_OPEN_COLOR : OFF);
}

void readBattery() {
  uint32_t mv = analogReadMilliVolts(BATT_ADC_PIN) * BATT_DIVIDER;
  if (mv < BATT_ABSENT_MV) {
    battLow = false;
    Serial.printf("BATT %umV (no divider, warning off)\n", mv);
    return;
  }
  battLow = mv < BATT_LOW_MV;
  Serial.printf("BATT %umV%s\n", mv, battLow ? " LOW" : "");
}

void recordDwell() {
  if (openedAt == 0) {
    return;
  }
  uint32_t dwell = millis() - openedAt;
  openedAt = 0;

  dwellCount++;
  dwellSumMs += dwell;
  if (dwell < dwellMinMs) dwellMinMs = dwell;
  if (dwell > dwellMaxMs) dwellMaxMs = dwell;

  Serial.printf("DWELL %ums (n=%u avg=%ums min=%ums max=%ums)\n",
                dwell, dwellCount, dwellSumMs / dwellCount, dwellMinMs, dwellMaxMs);
}

void playTrack(uint8_t track) {
  if (playerReady) {
    player.playMp3Folder(track);
  }
}

uint64_t nowUs() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000000ULL + tv.tv_usec;
}

void printPowerStats() {
  uint64_t awakeSec = awakeUs / 1000000ULL;
  uint64_t sleepSec = sleepUs / 1000000ULL;
  uint64_t totalSec = awakeSec + sleepSec;
  if (totalSec == 0) {
    return;
  }
  float avgMa = (awakeSec * AWAKE_MA + sleepSec * SLEEP_MA) / (float)totalSec;

  Serial.printf("POWER boots=%u events=%u awake=%llus sleep=%llus total=%llus\n",
                bootCount, doorEvents, awakeSec, sleepSec, totalSec);
  Serial.printf("POWER duty=%.1f%% avg=%.1fmA est=%.0fmAh/day\n",
                100.0f * awakeSec / totalSec, avgMa, avgMa * 24.0f);
}

void goToSleep() {
  awakeUs += nowUs() - bootUs;
  sleepStartUs = nowUs();
  printPowerStats();
  Serial.println("sleeping");
  Serial.flush();
  setLight(OFF);
  digitalWrite(PLAYER_POWER_PIN, LOW);

  rtc_gpio_pullup_dis(ACCEL_INT_PIN);
  rtc_gpio_pulldown_en(ACCEL_INT_PIN);
  esp_sleep_enable_ext0_wakeup(ACCEL_INT_PIN, 1);
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_BACKUP_S * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  setCpuFrequencyMhz(80);

  bootUs = nowUs();
  bootCount++;
  if (sleepStartUs > 0) {
    sleepUs += bootUs - sleepStartUs;
  }

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(PLAYER_POWER_PIN, OUTPUT);
  digitalWrite(PLAYER_POWER_PIN, HIGH);

  pinMode(LED_PIN, OUTPUT);
  setLight(OFF);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long); // ปรับระยะเป็นโหมดไกล (สูงสุด 4 เมตร)
  sensor.setMeasurementTimingBudget(33000); // 33ms ต่อการวัด
  sensor.startContinuous(100); // วัดทุก 100ms

  Serial2.begin(9600, SERIAL_8N1, PLAYER_RX_PIN, PLAYER_TX_PIN);
  playerReady = player.begin(Serial2);
  if (playerReady) {
    player.volume(PLAYER_VOLUME);
    Serial.println("DFPlayer ready");
  } else {
    Serial.println("DFPlayer not found, light only");
  }

  readBattery();
  printPowerStats();
  lastEvent = millis();
}

void loop() {
  uint16_t distance = sensor.read();
  bool timeout = sensor.timeoutOccurred();
  bool nowOpen = timeout || distance > DOOR_OPEN_MM;

  Serial.print("Distance (mm): ");
  Serial.print(distance);
  Serial.print(nowOpen ? " OPEN" : " CLOSED");
  if (timeout) { Serial.print(" TIMEOUT"); }
  Serial.println();

  if (nowOpen != doorOpen) {
    pendingCount++;
    if (pendingCount >= STATE_CONFIRM_COUNT) {
      doorOpen = nowOpen;
      pendingCount = 0;
      lastEvent = millis();
      doorEvents++;
      if (doorOpen) {
        openedAt = millis();
        playTrack(battLow ? TRACK_DOOR_OPEN_LOW_BATT : TRACK_DOOR_OPEN);
      } else {
        recordDwell();
        playTrack(TRACK_DOOR_CLOSED);
      }
    }
  } else {
    pendingCount = 0;
  }

  updateLight();

  if (millis() - lastEvent >= IDLE_TIMEOUT_MS) {
    goToSleep();
  }
}