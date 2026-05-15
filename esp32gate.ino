// ═══════════════════════════════════════════════════════════════
// ESP32 Gate Controller
// Dual-leaf gate controller — ESP32 + FreeRTOS
// Features: relay sequencing, reed sensors, WiFi dashboard,
//           TFT display menu, NVS config/log, NTP, bilingual UI
// ═══════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TFT_eSPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <time.h>

// ── Build info ───────────────────────────────────────────────
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// ── Pin definitions ──────────────────────────────────────────
#define PIN_RELAY_L1_OPEN   25
#define PIN_RELAY_L1_CLOSE  26
#define PIN_RELAY_L2_OPEN   27
#define PIN_RELAY_L2_CLOSE  14
#define PIN_RELAY_AUX1      13  // Aux relay 1 — default: electromagnetic lock
#define PIN_RELAY_AUX2      12  // Aux relay 2 — default: light / siren

#define PIN_REED_OPEN_L1    34
#define PIN_REED_CLOSE_L1   35
#define PIN_REED_OPEN_L2    32
#define PIN_REED_CLOSE_L2   33

#define PIN_BTN_GATE        22
#define PIN_BTN_MINUS       16
#define PIN_BTN_PLUS        17
#define PIN_BTN_OK           0

#define LONG_PRESS_MS     3000
#define MENU_TIMEOUT_MS  30000

// ── Relay logic ──────────────────────────────────────────────
#define RELAY_ON   HIGH
#define RELAY_OFF  LOW

// ── NVS namespaces ───────────────────────────────────────────
#define NVS_NS_CONFIG  "gate_cfg"
#define NVS_NS_LOG     "gate_log"
#define NVS_NS_WIFI    "wifi"

// ── WiFi ─────────────────────────────────────────────────────
#define AP_SSID            "ESP32 Gate"
#define CONNECT_TIMEOUT_MS 10000

// ── NTP ──────────────────────────────────────────────────────
#define NTP_SERVER  "pool.ntp.org"
#define GMT_OFFSET  (-3 * 3600)
#define DST_OFFSET  0

// ── Event log ────────────────────────────────────────────────
#define LOG_MAX_ENTRIES 100

typedef enum {
  LOG_EVT_OPENED = 0, LOG_EVT_CLOSED, LOG_EVT_STOPPED,
  LOG_EVT_TIMEOUT, LOG_EVT_OPEN_TOO_LONG, LOG_EVT_ERROR,
  LOG_EVT_BREAKIN_FC1, LOG_EVT_BREAKIN_FC2, LOG_EVT_MOTOR_FC1, LOG_EVT_MOTOR_FC2
} LogEvent_t;

struct LogEntry { uint8_t event; char timestamp[15]; };

// ── Aux relay modes ──────────────────────────────────────────
typedef enum {
  AUX_MODE_DISABLED = 0,  // relay never activates
  AUX_MODE_LOCK,          // lock: release before open, engage after close
  AUX_MODE_LIGHT,         // light: on during movement + N min after close
  AUX_MODE_SIREN,         // siren: on during movement only
} AuxMode_t;

// ── Configuration ────────────────────────────────────────────
struct GateConfig {
  uint32_t leafDelayMs;
  uint32_t safetyTimeoutMs;
  uint32_t alertOpenMin;
  bool     sequenceInverted;
  uint8_t  language;
  // Stage 7 — Aux relays
  // aux1/2TimeParam meaning depends on mode:
  //   LOCK  -> ms to wait after releasing lock before opening
  //   LIGHT -> minutes to keep light on after gate closes
  //   SIREN/DISABLED -> unused
  uint8_t  aux1Mode;
  uint32_t aux1TimeParam;
  uint8_t  aux2Mode;
  uint32_t aux2TimeParam;
  // Stage 8 — Alarms
  // alertOpenMin==0 disables open-too-long alarm entirely
  bool     alarmBreakIn;    // sound siren on forced opening
  bool     alarmOpenSiren;  // sound siren when open too long (alertOpenMin>0)
  bool     alarmMotorFault; // detect motor fault (reed no response in 5s)
};

static GateConfig gConfig = {
  .leafDelayMs    = 3000,
  .safetyTimeoutMs= 20000,
  .alertOpenMin   = 10,
  .sequenceInverted=false,
  .language       = 0,
  .aux1Mode       = AUX_MODE_LOCK,
  .aux1TimeParam  = 2,
  .aux2Mode       = AUX_MODE_LIGHT,
  .aux2TimeParam  = 2,
  .alarmBreakIn   = true,
  .alarmOpenSiren = true,
  .alarmMotorFault= true,
};

// ── Leaf pins ────────────────────────────────────────────────
struct LeafPins {
  uint8_t firstOpen,firstClose,secondOpen,secondClose;
  uint8_t reedOpenFirst,reedCloseFirst,reedOpenSecond,reedCloseSecond;
  const char* firstName; const char* secondName;
};

// ── Gate states ──────────────────────────────────────────────
typedef enum {
  GATE_CLOSED=0, GATE_OPENING, GATE_OPEN,
  GATE_CLOSING, GATE_STOPPED, GATE_ERROR
} GateState_t;

typedef enum { CMD_TOGGLE=0 } GateCmd_t;

struct GateStatus {
  GateState_t state, lastMovement;
  bool reed_open_l1,reed_close_l1,reed_open_l2,reed_close_l2;
  unsigned long lastStateChange;
};

// ── Menu states ──────────────────────────────────────────────
typedef enum {
  MENU_IDLE=0, MENU_ROOT, MENU_GATE_CTRL,
  MENU_CONFIG, MENU_CONFIG_EDIT, MENU_LOG, MENU_WIFI
} MenuState_t;

// ── Display palette ──────────────────────────────────────────
// BSWAP: SPI byte order is inverted for this display
#define BSWAP(c) ((((c)>>8)&0xFF)|(((c)&0xFF)<<8))
#define D_BG     TFT_BLACK
#define D_ACCENT BSWAP(0x565F)  // azul claro
#define D_WHITE  TFT_WHITE
#define D_GREEN  0x07E0          // verde — sem swap
#define D_RED    BSWAP(0xF800)  // vermelho
#define D_YELLOW 0xFFE0          // amarelo — sem swap
#define D_DIM    BSWAP(0x4A49)  // cinza legível no fundo preto
#define D_W      160
#define D_H      128

// ── Globals ──────────────────────────────────────────────────
static GateStatus        gStatus   = {GATE_CLOSED,GATE_OPENING,false,true,false,true,0};
static SemaphoreHandle_t gMutex    = nullptr;
static QueueHandle_t     gCmdQueue = nullptr;
static QueueHandle_t     gBtnQueue = nullptr;
static Preferences       gPrefs;
static WebServer         server(80);
static TFT_eSPI          dTft;

static bool   wifiConnected=false, portalMode=false;
static bool   ntpSynced=false, displayPresent=false;
static String currentIP="";

static bool          gAlertOpen     = false;
static bool          gMotorActive   = false;  // set during any motor sequence, blocks break-in detection
// Per-channel light timers (index 0=AUX1, 1=AUX2)
static bool          gAuxLightOn[2]  = {false, false};
static unsigned long gAuxLightEnd[2] = {0, 0};

// Alarm state — persists until gate closes or is reset
typedef enum { ALARM_NONE=0, ALARM_BREAKIN_FC1, ALARM_BREAKIN_FC2, ALARM_OPEN_LONG, ALARM_MOTOR_FC1, ALARM_MOTOR_FC2 } AlarmType_t;
static AlarmType_t   gAlarm         = ALARM_NONE;

static uint16_t logHead=0, logTail=0, logCount=0;
static int      arrowFrame=0;

static MenuState_t   menuState     = MENU_IDLE;
static int           menuCursor    = 0;
static int           menuEditField = 0;
static unsigned long menuLastInput = 0;
static int           logScrollPos  = 0;

static TaskHandle_t hTaskGate=nullptr, hTaskInput=nullptr;
static TaskHandle_t hTaskMon=nullptr,  hTaskWeb=nullptr, hTaskDisp=nullptr;

typedef enum { BTN_GATE=0, BTN_PLUS, BTN_MINUS, BTN_OK, BTN_OK_LONG } BtnEvent_t;

// ═══════════════════════════════════════════════════════════════
// HELPERS
// ═══════════════════════════════════════════════════════════════

// ── State label table [language][state] ──────────────────────
// Order must match GateState_t enum: CLOSED,OPENING,OPEN,CLOSING,STOPPED,ERROR
static const char* STATE_LABELS[][6] = {
  {"CLOSED",  "OPENING", "OPEN",   "CLOSING",  "STOPPED", "ERROR"}, // EN
  {"FECHADO", "ABRINDO", "ABERTO", "FECHANDO", "PARADO",  "ERRO" }, // PT
};
#define STATE_LANG_COUNT 2

// EN only — for serial/JSON/NVS (protocol strings must not change with language)
const char* stateToStr(GateState_t s) {
  if ((int)s<0||(int)s>GATE_ERROR) return "UNKNOWN";
  return STATE_LABELS[0][(int)s];
}

// Localized — for display only
const char* stateToStrLocal(GateState_t s) {
  if ((int)s<0||(int)s>GATE_ERROR) return "?";
  uint8_t lang = constrain(gConfig.language, 0, STATE_LANG_COUNT-1);
  return STATE_LABELS[lang][(int)s];
}

void setState(GateState_t s) {
  xSemaphoreTake(gMutex,portMAX_DELAY);
  if (s==GATE_OPENING||s==GATE_CLOSING) gStatus.lastMovement=s;
  Serial.printf("[GATE] %s -> %s\n",stateToStr(gStatus.state),stateToStr(s));
  gStatus.state=s; gStatus.lastStateChange=millis();
  xSemaphoreGive(gMutex);
}

GateState_t getState() {
  xSemaphoreTake(gMutex,portMAX_DELAY);
  GateState_t s=gStatus.state; xSemaphoreGive(gMutex); return s;
}

GateState_t getLastMovement() {
  xSemaphoreTake(gMutex,portMAX_DELAY);
  GateState_t s=gStatus.lastMovement; xSemaphoreGive(gMutex); return s;
}

void getTimestamp(char* buf) {
  struct tm t;
  if (getLocalTime(&t))
    snprintf(buf,15,"%04d%02d%02d%02d%02d%02d",
      t.tm_year+1900,t.tm_mon+1,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec);
  else strncpy(buf,"00000000000000",15);
}

String formatTimestamp(const char* ts) {
  if (strncmp(ts,"00000000000000",14)==0) return "--";
  char buf[20];
  snprintf(buf,sizeof(buf),"%.4s-%.2s-%.2s %.2s:%.2s:%.2s",
    ts,ts+4,ts+6,ts+8,ts+10,ts+12);
  return String(buf);
}

// Inline translation helper — no accented chars (TFT_eSPI font limitation)
const char* tr(const char* en, const char* pt) {
  return gConfig.language==0 ? en : pt;
}

// ═══════════════════════════════════════════════════════════════
// NVS — CONFIG
// ═══════════════════════════════════════════════════════════════

void configLoad() {
  gPrefs.begin(NVS_NS_CONFIG,true);
  gConfig.leafDelayMs      = gPrefs.getUInt ("leafDelay",  gConfig.leafDelayMs);
  gConfig.safetyTimeoutMs  = gPrefs.getUInt ("safetyMs",   gConfig.safetyTimeoutMs);
  gConfig.alertOpenMin     = gPrefs.getUInt ("alertMin",   gConfig.alertOpenMin);
  gConfig.sequenceInverted = gPrefs.getBool ("seqInv",     gConfig.sequenceInverted);
  gConfig.language         = gPrefs.getUChar("lang",       gConfig.language);
  gConfig.aux1Mode       = gPrefs.getUChar("aux1Mode",  gConfig.aux1Mode);
  { uint32_t v = gPrefs.getUInt("aux1Time", gConfig.aux1TimeParam);
    gConfig.aux1TimeParam = (v > 120) ? 2 : v; }  // 2=default, discard legacy ms values
  gConfig.aux2Mode       = gPrefs.getUChar("aux2Mode",  gConfig.aux2Mode);
  { uint32_t v = gPrefs.getUInt("aux2Time", gConfig.aux2TimeParam);
    gConfig.aux2TimeParam = (v > 120) ? 2 : v; }
  gConfig.alarmBreakIn   = gPrefs.getBool ("almBreakIn",gConfig.alarmBreakIn);
  gConfig.alarmOpenSiren = gPrefs.getBool ("almOpenSrn",gConfig.alarmOpenSiren);
  gConfig.alarmMotorFault= gPrefs.getBool ("almMotor",  gConfig.alarmMotorFault);
  gPrefs.end();
  Serial.println("[CFG] Loaded");
}

void configSave() {
  gPrefs.begin(NVS_NS_CONFIG,false);
  gPrefs.putUInt ("leafDelay", gConfig.leafDelayMs);
  gPrefs.putUInt ("safetyMs",  gConfig.safetyTimeoutMs);
  gPrefs.putUInt ("alertMin",  gConfig.alertOpenMin);
  gPrefs.putBool ("seqInv",    gConfig.sequenceInverted);
  gPrefs.putUChar("lang",      gConfig.language);
  gPrefs.putUChar("aux1Mode", gConfig.aux1Mode);
  gPrefs.putUInt ("aux1Time", gConfig.aux1TimeParam);
  gPrefs.putUChar("aux2Mode", gConfig.aux2Mode);
  gPrefs.putUInt ("aux2Time", gConfig.aux2TimeParam);
  gPrefs.putBool ("almBreakIn",gConfig.alarmBreakIn);
  gPrefs.putBool ("almOpenSrn",gConfig.alarmOpenSiren);
  gPrefs.putBool ("almMotor",  gConfig.alarmMotorFault);
  gPrefs.end();
  Serial.println("[CFG] Saved");
}

// ═══════════════════════════════════════════════════════════════
// NVS — LOG
// ═══════════════════════════════════════════════════════════════

// ── Log event label table [language][event] ──────────────────
// Order must match LogEvent_t enum
static const char* LOG_EVENT_LABELS[][10] = {
  {"OPENED","CLOSED","STOPPED","TIMEOUT","OPEN>LIMIT","ERROR",
   "BREAKIN-FC1","BREAKIN-FC2","MOTOR-FC1","MOTOR-FC2"},        // EN
  {"ABERTO","FECHADO","PARADO","TIMEOUT","ABERTO>LIM","ERRO",
   "ARROMBA-FC1","ARROMBA-FC2","MOTOR-FC1","MOTOR-FC2"},        // PT
};

const char* logEventToStr(uint8_t evt) {
  if (evt>=10) return "UNKNOWN";
  return LOG_EVENT_LABELS[0][evt];
}

const char* logEventToStrLocal(uint8_t evt) {
  if (evt>=10) return "?";
  uint8_t lang=constrain(gConfig.language,0,STATE_LANG_COUNT-1);
  return LOG_EVENT_LABELS[lang][evt];
}

void logInit() {
  gPrefs.begin(NVS_NS_LOG,true);
  logHead=gPrefs.getUShort("head",0);
  logTail=gPrefs.getUShort("tail",0);
  logCount=gPrefs.getUShort("count",0);
  gPrefs.end();
  Serial.printf("[LOG] Init - %u entries\n",logCount);
}

void logWrite(LogEvent_t evt) {
  LogEntry entry; entry.event=(uint8_t)evt; getTimestamp(entry.timestamp);
  char key[4]; snprintf(key,sizeof(key),"e%02u",logTail);
  gPrefs.begin(NVS_NS_LOG,false);
  gPrefs.putBytes(key,&entry,sizeof(LogEntry));
  logTail=(logTail+1)%LOG_MAX_ENTRIES;
  if (logCount>=LOG_MAX_ENTRIES) logHead=(logHead+1)%LOG_MAX_ENTRIES;
  else logCount++;
  gPrefs.putUShort("head",logHead);
  gPrefs.putUShort("tail",logTail);
  gPrefs.putUShort("count",logCount);
  gPrefs.end();
  Serial.printf("[LOG] %s @ %s\n",logEventToStr(evt),entry.timestamp);
}

void logClear() {
  gPrefs.begin(NVS_NS_LOG,false); gPrefs.clear(); gPrefs.end();
  logHead=logTail=logCount=0;
  Serial.println("[LOG] Cleared");
}

LogEntry logGet(uint16_t idx) {
  LogEntry entry={0,"00000000000000"};
  char key[4]; snprintf(key,sizeof(key),"e%02u",(logHead+idx)%LOG_MAX_ENTRIES);
  gPrefs.begin(NVS_NS_LOG,true);
  gPrefs.getBytes(key,&entry,sizeof(LogEntry));
  gPrefs.end();
  return entry;
}

String logToJson() {
  String json="[";
  gPrefs.begin(NVS_NS_LOG,true);
  for (uint16_t i=0;i<logCount;i++) {
    uint16_t idx=(logHead+i)%LOG_MAX_ENTRIES;
    char key[4]; snprintf(key,sizeof(key),"e%02u",idx);
    LogEntry entry; gPrefs.getBytes(key,&entry,sizeof(LogEntry));
    if (i>0) json+=",";
    json+="{\"e\":\""; json+=logEventToStrLocal(entry.event);
    json+="\",\"t\":\""; json+=formatTimestamp(entry.timestamp); json+="\"}";
  }
  gPrefs.end(); json+="]";
  return json;
}

// ═══════════════════════════════════════════════════════════════
// RELAY SAFETY
// ═══════════════════════════════════════════════════════════════

void relayAllOff() {
  digitalWrite(PIN_RELAY_L1_OPEN, RELAY_OFF); digitalWrite(PIN_RELAY_L1_CLOSE,RELAY_OFF);
  digitalWrite(PIN_RELAY_L2_OPEN, RELAY_OFF); digitalWrite(PIN_RELAY_L2_CLOSE,RELAY_OFF);
  Serial.println("[RELAY] All OFF");
}

void relaySet(uint8_t pin, bool on, const char* label) {
  digitalWrite(pin, on?RELAY_ON:RELAY_OFF);
  Serial.printf("[RELAY] %s %s\n",label,on?"ON":"OFF");
}

// ═══════════════════════════════════════════════════════════════
// AUX RELAY CONTROL
// ═══════════════════════════════════════════════════════════════

void auxRelaySet(uint8_t pin, bool on, const char* label) {
  digitalWrite(pin, on ? RELAY_ON : RELAY_OFF);
  Serial.printf("[AUX] %s %s\n", label, on ? "ON" : "OFF");
}

// ── AUX logic by mode ────────────────────────────────────────
// LOCK:  release (ON) before opening + delay, engage (OFF) after fully closed.
// LIGHT: ON at any movement start, timer starts immediately.
//        Turns off after timeParam minutes regardless of gate state.
//        Timer 0 = stays on until manually cleared (treated as 60 min).
// SIREN: ON when alarm triggered, OFF when gate closes.
// DISABLED: never activates.

// Internal: activate light channel and start its timer immediately
// ch: 0=AUX1, 1=AUX2
static void auxLightOn(uint8_t ch, uint8_t pin, uint32_t timeParam, const char* label) {
  auxRelaySet(pin, true, label);
  uint32_t mins = timeParam > 0 ? timeParam : 60;
  gAuxLightOn[ch]  = true;
  gAuxLightEnd[ch] = millis() + mins * 60000UL;
  Serial.printf("[AUX] ch%d LIGHT timer: %u min (%lu ms)\n", ch, mins, mins*60000UL);
}

// Called at start of opening sequence
void auxOnOpen() {
  if (gConfig.aux1Mode == AUX_MODE_LOCK) {
    auxRelaySet(PIN_RELAY_AUX1, true, "AUX1-LOCK release");
    vTaskDelay((gConfig.aux1TimeParam * 1000) / portTICK_PERIOD_MS);
  } else if (gConfig.aux1Mode == AUX_MODE_LIGHT) {
    auxLightOn(0, PIN_RELAY_AUX1, gConfig.aux1TimeParam, "AUX1-LIGHT");
  }
  if (gConfig.aux2Mode == AUX_MODE_LOCK) {
    auxRelaySet(PIN_RELAY_AUX2, true, "AUX2-LOCK release");
    vTaskDelay((gConfig.aux2TimeParam * 1000) / portTICK_PERIOD_MS);
  } else if (gConfig.aux2Mode == AUX_MODE_LIGHT) {
    auxLightOn(1, PIN_RELAY_AUX2, gConfig.aux2TimeParam, "AUX2-LIGHT");
  }
}

// Called at start of closing sequence
void auxOnClose() {
  if (gConfig.aux1Mode == AUX_MODE_LIGHT)
    auxLightOn(0, PIN_RELAY_AUX1, gConfig.aux1TimeParam, "AUX1-LIGHT");
  if (gConfig.aux2Mode == AUX_MODE_LIGHT)
    auxLightOn(1, PIN_RELAY_AUX2, gConfig.aux2TimeParam, "AUX2-LIGHT");
}

// Called when gate finishes closing — lock engages, siren off
// Light is NOT touched here — timer handles it independently
void auxOnClosed() {
  if (gConfig.aux1Mode == AUX_MODE_LOCK)
    auxRelaySet(PIN_RELAY_AUX1, false, "AUX1-LOCK engage");
  if (gConfig.aux2Mode == AUX_MODE_LOCK)
    auxRelaySet(PIN_RELAY_AUX2, false, "AUX2-LOCK engage");
  if (gConfig.aux1Mode == AUX_MODE_SIREN) auxRelaySet(PIN_RELAY_AUX1, false, "AUX1-SIREN off");
  if (gConfig.aux2Mode == AUX_MODE_SIREN) auxRelaySet(PIN_RELAY_AUX2, false, "AUX2-SIREN off");
}

// Called by taskMonitor when alert fires
void auxOnAlert() {
  if (gConfig.aux1Mode == AUX_MODE_SIREN) auxRelaySet(PIN_RELAY_AUX1, true, "AUX1-SIREN on");
  if (gConfig.aux2Mode == AUX_MODE_SIREN) auxRelaySet(PIN_RELAY_AUX2, true, "AUX2-SIREN on");
}

// Called periodically by taskMonitor — handles light timer expiry
void auxTick() {
  unsigned long now = millis();
  if (gAuxLightOn[0] && now >= gAuxLightEnd[0]) {
    if (gConfig.aux1Mode == AUX_MODE_LIGHT) auxRelaySet(PIN_RELAY_AUX1, false, "AUX1-LIGHT off");
    gAuxLightOn[0] = false;
  }
  if (gAuxLightOn[1] && now >= gAuxLightEnd[1]) {
    if (gConfig.aux2Mode == AUX_MODE_LIGHT) auxRelaySet(PIN_RELAY_AUX2, false, "AUX2-LIGHT off");
    gAuxLightOn[1] = false;
  }
}

// ── Alarm helper ─────────────────────────────────────────────
void alarmTrigger(AlarmType_t type, LogEvent_t evt) {
  if (gAlarm==type) return;  // already active, don't repeat
  gAlarm=type;
  logWrite(evt);
  Serial.printf("[ALARM] %s\n", logEventToStr(evt));
  bool siren=false;
  if (type==ALARM_BREAKIN_FC1||type==ALARM_BREAKIN_FC2) siren=gConfig.alarmBreakIn;
  if (type==ALARM_OPEN_LONG)                             siren=gConfig.alarmOpenSiren;
  // ALARM_MOTOR: no siren — it's a fault, not a security event
  if (siren) auxOnAlert();
}

void alarmClear() {
  if (gAlarm==ALARM_NONE) return;
  gAlarm=ALARM_NONE;
  gAlertOpen=false;
  auxOnClosed();  // engages lock if configured + silences siren
}

LeafPins resolveLeafPins() {
  LeafPins p;
  if (!gConfig.sequenceInverted) {
    p.firstOpen=PIN_RELAY_L1_OPEN; p.firstClose=PIN_RELAY_L1_CLOSE;
    p.secondOpen=PIN_RELAY_L2_OPEN; p.secondClose=PIN_RELAY_L2_CLOSE;
    p.reedOpenFirst=PIN_REED_OPEN_L1; p.reedCloseFirst=PIN_REED_CLOSE_L1;
    p.reedOpenSecond=PIN_REED_OPEN_L2; p.reedCloseSecond=PIN_REED_CLOSE_L2;
    p.firstName="L1"; p.secondName="L2";
  } else {
    p.firstOpen=PIN_RELAY_L2_OPEN; p.firstClose=PIN_RELAY_L2_CLOSE;
    p.secondOpen=PIN_RELAY_L1_OPEN; p.secondClose=PIN_RELAY_L1_CLOSE;
    p.reedOpenFirst=PIN_REED_OPEN_L2; p.reedCloseFirst=PIN_REED_CLOSE_L2;
    p.reedOpenSecond=PIN_REED_OPEN_L1; p.reedCloseSecond=PIN_REED_CLOSE_L1;
    p.firstName="L2"; p.secondName="L1";
  }
  return p;
}

bool sequenceOpen() {
  // Opening: monitors close reeds going OFF (confirms motor moved the leaf).
  // If close reed doesn't go OFF within 5s of relay activation = motor fault.
  gMotorActive = true;
  if (gAlarm==ALARM_OPEN_LONG) alarmClear();  // movement started, alert served its purpose
  auxOnOpen();
  LeafPins lp=resolveLeafPins();
  unsigned long tStart=millis();
  bool secondStarted=false,firstDone=false,secondDone=false;
  // Record initial close reed states — fault if they don't go OFF
  bool cl1Init=(digitalRead(lp.reedCloseFirst)==LOW);
  bool cl2Init=(digitalRead(lp.reedCloseSecond)==LOW);
  bool firstMoved=!cl1Init;   // already off = already moved (edge case)
  bool secondMoved=!cl2Init;
  relaySet(lp.firstClose,false,"FIRST_CLOSE");
  relaySet(lp.secondClose,false,"SECOND_CLOSE");
  vTaskDelay(10/portTICK_PERIOD_MS);
  relaySet(lp.firstOpen,true,"FIRST_OPEN");
  unsigned long tFirst=millis();
  unsigned long tSecond=0;
  while (!(firstDone&&secondDone)) {
    GateCmd_t cmd;
    if (xQueuePeek(gCmdQueue,&cmd,0)==pdTRUE) {
      xQueueReceive(gCmdQueue,&cmd,0);
      relayAllOff(); gMotorActive=false; setState(GATE_STOPPED); logWrite(LOG_EVT_STOPPED); return false;
    }
    unsigned long el=millis()-tStart;
    if (el>=gConfig.safetyTimeoutMs) {
      relayAllOff(); gMotorActive=false; setState(GATE_ERROR); logWrite(LOG_EVT_TIMEOUT); return false;
    }
    // Motor fault: close reed of first leaf didn't go OFF in 5s
    if (!firstMoved && gConfig.alarmMotorFault && millis()-tFirst>=5000) {
      relayAllOff(); gMotorActive=false;
      AlarmType_t ft=gConfig.sequenceInverted?ALARM_MOTOR_FC2:ALARM_MOTOR_FC1;
      LogEvent_t  fe=gConfig.sequenceInverted?LOG_EVT_MOTOR_FC2:LOG_EVT_MOTOR_FC1;
      alarmTrigger(ft,fe); setState(GATE_ERROR); return false;
    }
    // Track close reed going OFF = motor is moving
    if (!firstMoved && digitalRead(lp.reedCloseFirst)!=LOW) firstMoved=true;

    if (!secondStarted&&el>=gConfig.leafDelayMs) {
      relaySet(lp.secondClose,false,"SECOND_CLOSE");
      vTaskDelay(10/portTICK_PERIOD_MS);
      relaySet(lp.secondOpen,true,"SECOND_OPEN");
      secondStarted=true;
      tSecond=millis();
    }
    // Motor fault: close reed of second leaf didn't go OFF in 5s
    if (secondStarted && !secondMoved && gConfig.alarmMotorFault && tSecond>0 && millis()-tSecond>=5000) {
      relayAllOff(); gMotorActive=false;
      AlarmType_t ft=gConfig.sequenceInverted?ALARM_MOTOR_FC1:ALARM_MOTOR_FC2;
      LogEvent_t  fe=gConfig.sequenceInverted?LOG_EVT_MOTOR_FC1:LOG_EVT_MOTOR_FC2;
      alarmTrigger(ft,fe); setState(GATE_ERROR); return false;
    }
    if (secondStarted && !secondMoved && digitalRead(lp.reedCloseSecond)!=LOW) secondMoved=true;

    // Completion: open reed goes ON
    if (!firstDone&&digitalRead(lp.reedOpenFirst)==LOW) {
      relaySet(lp.firstOpen,false,"FIRST_OPEN"); firstDone=true;
      Serial.printf("[REED] %s fully open\n",lp.firstName);
    }
    if (secondStarted&&!secondDone&&digitalRead(lp.reedOpenSecond)==LOW) {
      relaySet(lp.secondOpen,false,"SECOND_OPEN"); secondDone=true;
      Serial.printf("[REED] %s fully open\n",lp.secondName);
    }
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
  relayAllOff(); gMotorActive=false; return true;
}

bool sequenceClose() {
  // Closing: monitors open reeds going OFF (confirms motor moved the leaf).
  // If open reed doesn't go OFF within 5s of relay activation = motor fault.
  gMotorActive = true;
  if (gAlarm==ALARM_OPEN_LONG) alarmClear();  // movement started, alert served its purpose
  auxOnClose();
  LeafPins lp=resolveLeafPins();
  unsigned long tStart=millis();
  bool firstStarted=false,secondDone=false,firstDone=false;
  // Track open reed going OFF = motor is moving
  bool secondMoved=(digitalRead(lp.reedOpenSecond)!=LOW);
  bool firstMoved =(digitalRead(lp.reedOpenFirst) !=LOW);
  relaySet(lp.firstOpen,false,"FIRST_OPEN");
  relaySet(lp.secondOpen,false,"SECOND_OPEN");
  vTaskDelay(10/portTICK_PERIOD_MS);
  relaySet(lp.secondClose,true,"SECOND_CLOSE");
  unsigned long tSecond=millis();
  while (!(firstDone&&secondDone)) {
    GateCmd_t cmd;
    if (xQueuePeek(gCmdQueue,&cmd,0)==pdTRUE) {
      xQueueReceive(gCmdQueue,&cmd,0);
      relayAllOff(); gMotorActive=false; setState(GATE_STOPPED); logWrite(LOG_EVT_STOPPED); return false;
    }
    unsigned long el=millis()-tStart;
    if (el>=gConfig.safetyTimeoutMs) {
      relayAllOff(); gMotorActive=false; setState(GATE_ERROR); logWrite(LOG_EVT_TIMEOUT); return false;
    }
    // Motor fault: open reed of second leaf didn't go OFF in 5s
    if (!secondMoved && gConfig.alarmMotorFault && millis()-tSecond>=5000) {
      relayAllOff(); gMotorActive=false;
      AlarmType_t ft=gConfig.sequenceInverted?ALARM_MOTOR_FC1:ALARM_MOTOR_FC2;
      LogEvent_t  fe=gConfig.sequenceInverted?LOG_EVT_MOTOR_FC1:LOG_EVT_MOTOR_FC2;
      alarmTrigger(ft,fe); setState(GATE_ERROR); return false;
    }
    if (!secondMoved && digitalRead(lp.reedOpenSecond)!=LOW) secondMoved=true;

    if (!firstStarted&&el>=gConfig.leafDelayMs) {
      relaySet(lp.firstOpen,false,"FIRST_OPEN");
      vTaskDelay(10/portTICK_PERIOD_MS);
      relaySet(lp.firstClose,true,"FIRST_CLOSE");
      firstStarted=true;
    }
    // Motor fault: open reed of first leaf didn't go OFF in 5s
    if (firstStarted && !firstMoved && gConfig.alarmMotorFault) {
      // use firstStarted time via el - leafDelayMs as proxy
      if (el-gConfig.leafDelayMs>=5000) {
        relayAllOff(); gMotorActive=false;
        AlarmType_t ft=gConfig.sequenceInverted?ALARM_MOTOR_FC2:ALARM_MOTOR_FC1;
        LogEvent_t  fe=gConfig.sequenceInverted?LOG_EVT_MOTOR_FC2:LOG_EVT_MOTOR_FC1;
        alarmTrigger(ft,fe); setState(GATE_ERROR); return false;
      }
    }
    if (firstStarted && !firstMoved && digitalRead(lp.reedOpenFirst)!=LOW) firstMoved=true;

    if (!secondDone&&digitalRead(lp.reedCloseSecond)==LOW) {
      relaySet(lp.secondClose,false,"SECOND_CLOSE"); secondDone=true;
      Serial.printf("[REED] %s fully closed\n",lp.secondName);
    }
    if (firstStarted&&!firstDone&&digitalRead(lp.reedCloseFirst)==LOW) {
      relaySet(lp.firstClose,false,"FIRST_CLOSE"); firstDone=true;
      Serial.printf("[REED] %s fully closed\n",lp.firstName);
    }
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
  relayAllOff(); gMotorActive=false;
  auxOnClosed();
  return true;
}

// ═══════════════════════════════════════════════════════════════
// DISPLAY — primitives
// ═══════════════════════════════════════════════════════════════

uint16_t stateColor(GateState_t s) {
  switch(s) {
    case GATE_OPEN:    return D_GREEN;
    case GATE_OPENING: return D_GREEN;
    case GATE_CLOSING: return D_RED;
    case GATE_CLOSED:  return D_RED;
    case GATE_STOPPED: return D_YELLOW;
    case GATE_ERROR:   return D_YELLOW;
    default:           return D_WHITE;
  }
}

void dClear() { dTft.fillScreen(D_BG); }

// Thin accent border — 1px clean line on all sides
void dBorder() {
  dTft.drawRect(0, 0, D_W, D_H, D_ACCENT);
}

void dText(int x, int y, const char* txt, uint16_t color, uint8_t size=1) {
  dTft.setTextColor(color); dTft.setTextSize(size);
  dTft.setCursor(x,y); dTft.print(txt);
}

void dTextCentered(int y, const char* txt, uint16_t color, uint8_t size=1) {
  int x=max(2,(int)((D_W-strlen(txt)*6*size)/2));
  dText(x,y,txt,color,size);
}

// Header: accent title + accent underline — matches web tab style
void dHeader(const char* title) {
  dBorder();
  dTft.setTextColor(D_ACCENT); dTft.setTextSize(1);
  dTft.setCursor(6,5); dTft.print(title);
  dTft.drawFastHLine(1,15,D_W-2,D_ACCENT);  // solid accent line under header
  dTft.drawFastHLine(1,16,D_W-2,D_YELLOW);     // subtle shadow line
}

// ═══════════════════════════════════════════════════════════════
// DISPLAY — idle screen
// ═══════════════════════════════════════════════════════════════

void displayDrawIdleFrame() {
  dClear();
  // Idle frame: yellow border — warm tone, consistent with dim/error color
  dTft.drawRect(0, 0, D_W, D_H, D_YELLOW);
  // "ESP32 GATE" in accent — matches web header style
  dTft.setTextColor(D_ACCENT); dTft.setTextSize(1);
  dTft.setCursor(4,5); dTft.print("ESP32 GATE");
  // Accent header line + yellow shadow
  dTft.drawFastHLine(1,15,D_W-2,D_ACCENT);
  dTft.drawFastHLine(1,16,D_W-2,D_YELLOW);
  // Yellow separators
  dTft.drawFastHLine(4,95,D_W-8,D_YELLOW);
  dTft.drawFastHLine(4,115,D_W-8,D_YELLOW);
}

void displayDrawTime() {
  struct tm t;
  if (!getLocalTime(&t)) return;
  char buf[9];
  sprintf(buf,"%02d:%02d:%02d",t.tm_hour,t.tm_min,t.tm_sec);
  dTft.fillRect(88,3,68,10,D_BG);
  dTft.setTextColor(D_ACCENT); dTft.setTextSize(1);
  dTft.setCursor(90,5); dTft.print(buf);
}

void displayDrawIP() {
  // Only called when IP changes to avoid flicker
  dTft.fillRect(4,117,D_W-8,9,D_BG);
  dTft.setTextColor(D_YELLOW); dTft.setTextSize(1);
  String ipStr = String("http://") + currentIP;
  int x=max(2,(int)((D_W-(int)ipStr.length()*6)/2));
  dTft.setCursor(x,118); dTft.print(ipStr.c_str());
}

static GateState_t idleLastS   = GATE_ERROR;
static bool idleLastOL1=false,idleLastCL1=false;
static bool idleLastOL2=false,idleLastCL2=false;
static String idleLastIP = "";

void displayIdleReset() {
  // Reset dirty flags so the next displayDrawIdle() repaints everything.
  // Called when returning from menu to idle screen.
  idleLastS   = GATE_ERROR;
  idleLastOL1 = !idleLastOL1;
  idleLastIP  = "";
}

void displayDrawIdle(GateState_t s, bool ol1, bool cl1, bool ol2, bool cl2) {
  // ── State label ─────────────────────────────────────────────
  if (s!=idleLastS) {
    dTft.fillRect(1,17,D_W-2,78,D_BG);
    uint16_t color=stateColor(s);
    const char* label=stateToStrLocal(s);
    // State label centered, large
    int x=max(4,(int)((D_W-(int)strlen(label)*12)/2));
    dTft.setTextColor(color); dTft.setTextSize(2);
    dTft.setCursor(x,26); dTft.print(label);
    // Thin underline in state color below label
    dTft.drawFastHLine(20,44,D_W-40,color);
    idleLastS=s; arrowFrame=0;
    if (s!=GATE_OPENING&&s!=GATE_CLOSING)
      dTft.fillRect(4,47,D_W-8,30,D_BG);
  }

  // ── Alarm / alert display (arrow area y=47-78) ──────────────
  {
    static AlarmType_t lastAlarm=ALARM_NONE;
    static bool lastAlert=false;
    if (gAlarm!=lastAlarm || gAlertOpen!=lastAlert || s!=idleLastS) {
      dTft.fillRect(1,47,D_W-2,30,D_BG);
      if (gAlarm==ALARM_BREAKIN_FC1 || gAlarm==ALARM_BREAKIN_FC2) {
        const char* fc = (gAlarm==ALARM_BREAKIN_FC1) ? "FC1" : "FC2";
        char line2[20];
        snprintf(line2, sizeof(line2), "%s %s", tr("BREAKIN","ARROMBA"), fc);
        // "ALARME" large centered
        dTft.setTextSize(2); dTft.setTextColor(D_RED);
        int x=max(4,(int)((D_W-(int)strlen(tr("ALARME","ALARME"))*12)/2));
        dTft.setCursor(x,48); dTft.print(tr("ALARME","ALARME"));
        // detail line below
        dTft.setTextSize(1); dTft.setTextColor(D_RED);
        int x2=max(4,(int)((D_W-(int)strlen(line2)*6)/2));
        dTft.setCursor(x2,66); dTft.print(line2);
      } else if (gAlarm==ALARM_MOTOR_FC1 || gAlarm==ALARM_MOTOR_FC2) {
        const char* fc = (gAlarm==ALARM_MOTOR_FC1) ? "FC1" : "FC2";
        char line2[20];
        snprintf(line2, sizeof(line2), "%s %s", tr("MOTOR","MOTOR"), fc);
        dTft.setTextSize(2); dTft.setTextColor(D_YELLOW);
        int x=max(4,(int)((D_W-(int)strlen(tr("FAULT","FALHA"))*12)/2));
        dTft.setCursor(x,48); dTft.print(tr("FAULT","FALHA"));
        dTft.setTextSize(1); dTft.setTextColor(D_YELLOW);
        int x2=max(4,(int)((D_W-(int)strlen(line2)*6)/2));
        dTft.setCursor(x2,66); dTft.print(line2);
      } else if (gAlarm==ALARM_OPEN_LONG || gAlertOpen) {
        dTft.setTextSize(2); dTft.setTextColor(D_YELLOW);
        int x=max(4,(int)((D_W-(int)strlen(tr("ALERT","ALERTA"))*12)/2));
        dTft.setCursor(x,48); dTft.print(tr("ALERT","ALERTA"));
        dTft.setTextSize(1); dTft.setTextColor(D_YELLOW);
        const char* msg=tr("OPEN TOO LONG","ABERTO DEMAIS");
        int x2=max(4,(int)((D_W-(int)strlen(msg)*6)/2));
        dTft.setCursor(x2,66); dTft.print(msg);
      } else if (s==GATE_OPENING||s==GATE_CLOSING) {
        // arrows drawn separately below — just clear here
      }
      lastAlarm=gAlarm; lastAlert=gAlertOpen;
    }
  }

  // ── Arrow animation (only when no alarm active) ──────────────
  if ((s==GATE_OPENING||s==GATE_CLOSING) && gAlarm==ALARM_NONE) {
    bool opening=(s==GATE_OPENING);
    uint16_t color=opening?D_GREEN:D_RED;
    const char* arrow=opening?">":"<";
    int positions[]={16,40,64,88,112};
    dTft.setTextSize(3);
    for (int i=0;i<5;i++) {
      bool lit=((i-arrowFrame+5)%5)<2;
      dTft.setTextColor(lit?color:D_DIM);
      dTft.setCursor(positions[i],52); dTft.print(arrow);
    }
  }

  // ── Reed row — S1 left half, S2 right half, both centered ───
  if (ol1!=idleLastOL1||cl1!=idleLastCL1||ol2!=idleLastOL2||cl2!=idleLastCL2) {
    dTft.fillRect(1,96,D_W-2,18,D_BG);
    const char* lO=tr("O","A");
    const char* lC=tr("C","F");
    dTft.setTextSize(1);
    // Each half is 80px wide. Content: "Sx"(12) lO(6) ":"(6) circ(10) lC(6) ":"(6) circ(10) = 56px
    // Centered in 80px: start offset = (80-56)/2 = 12px from half edge
    // S1: half starts at x=0, content starts at x=12
    // S2: half starts at x=80, content starts at x=92
    int s1x=8,  s2x=88;  // left edge of each half's content
    int cy=102, r=4;     // circle center y and radius
    // S1
    dTft.setTextColor(D_ACCENT); dTft.setCursor(s1x,    99); dTft.print("FC1");
    dTft.setTextColor(D_YELLOW); dTft.setCursor(s1x+20, 99); dTft.print(lO); dTft.print(":");
    dTft.fillCircle(s1x+34, cy, r, ol1?D_GREEN:D_DIM);
    dTft.setTextColor(D_YELLOW); dTft.setCursor(s1x+42, 99); dTft.print(lC); dTft.print(":");
    dTft.fillCircle(s1x+56, cy, r, cl1?D_GREEN:D_DIM);
    // S2
    dTft.setTextColor(D_ACCENT); dTft.setCursor(s2x,    99); dTft.print("FC2");
    dTft.setTextColor(D_YELLOW); dTft.setCursor(s2x+20, 99); dTft.print(lO); dTft.print(":");
    dTft.fillCircle(s2x+34, cy, r, ol2?D_GREEN:D_DIM);
    dTft.setTextColor(D_YELLOW); dTft.setCursor(s2x+42, 99); dTft.print(lC); dTft.print(":");
    dTft.fillCircle(s2x+56, cy, r, cl2?D_GREEN:D_DIM);
    // vertical separator
    dTft.drawFastVLine(D_W/2, 96, 18, D_YELLOW);
    idleLastOL1=ol1; idleLastCL1=cl1; idleLastOL2=ol2; idleLastCL2=cl2;
  }

  // ── IP ───────────────────────────────────────────────────────
  if (currentIP!=idleLastIP) {
    displayDrawIP();
    idleLastIP=currentIP;
  }
}

// ═══════════════════════════════════════════════════════════════
// DISPLAY — menu screens
// ═══════════════════════════════════════════════════════════════

#define MENU_Y0   20
#define MENU_LINE 15

// Draw rounded selection background (accent color) + accent left bar at given y
void menuDrawSelected(int y) {
  dTft.fillRoundRect(4,y-1,D_W-8,MENU_LINE-2,3,D_ACCENT);
  dTft.fillRect(4,y-1,3,MENU_LINE-2,D_YELLOW);
}

// Selected item: accent rounded rect + left accent bar
// Non-selected: plain text with subtle indent
void menuDrawList(const char* title, const char** items, int count, int cursor) {
  dClear(); dHeader(title);
  for (int i=0;i<count;i++) {
    int y=MENU_Y0+i*MENU_LINE;
    if (i==cursor) {
      menuDrawSelected(y);
      dText(12,y,items[i],D_BG);
    } else {
      dText(10,y,items[i],D_YELLOW);
    }
  }
  // separator before last item (Back/Voltar equivalent)
  dTft.drawFastHLine(4,MENU_Y0+(count-1)*MENU_LINE-3,D_W-8,D_YELLOW);
  dTextCentered(D_H-10,tr("+/- nav  OK select","+/- nav  OK selec"),D_YELLOW);
}

void menuDrawRoot() {
  const char* items[]={
    tr("Gate Control","Controle"),
    tr("Configuration","Configuracao"),
    tr("Event Log","Registro"),
    "WiFi",
    tr("Exit","Sair")
  };
  menuDrawList(tr("MENU","MENU"),items,5,menuCursor);
}

void menuDrawGateCtrl() {
  const char* items[]={tr("Toggle Gate","Acionar"),tr("Back","Voltar")};
  menuDrawList(tr("GATE CTRL","PORTAO"),items,2,menuCursor);
}

void menuDrawConfig() {
  dClear(); dHeader(tr("CONFIGURATION","CONFIG"));
  // 12 fields (0-11) + Back (12) — same order as web
  // 0:LeafDelay 1:SafetyT/O 2:Sequence 3:Language
  // 4:AlertOpen 5:AlarmBreakIn 6:AlarmOpenSrn 7:AlarmMotor
  // 8:AUX1Mode 9:AUX1Param 10:AUX2Mode 11:AUX2Param
  struct { const char* label; char val[10]; } fields[12];
  static const char* auxModeStr[] = {"OFF","LOCK","LUZ","SIREN"};
  static const char* onOff[]      = {tr("OFF","OFF"), tr("ON","ON")};
  snprintf(fields[0].val,10,"%ums",    gConfig.leafDelayMs);
  snprintf(fields[1].val,10,"%ums",    gConfig.safetyTimeoutMs);
  snprintf(fields[2].val,10,"%s",      gConfig.sequenceInverted?"INV":"NRM");
  snprintf(fields[3].val,10,"%s",      gConfig.language==0?"EN":"PT");
  snprintf(fields[4].val,10,"%umin",   gConfig.alertOpenMin);
  snprintf(fields[5].val,10,"%s",      onOff[gConfig.alarmBreakIn]);
  snprintf(fields[6].val,10,"%s",      onOff[gConfig.alarmOpenSiren]);
  snprintf(fields[7].val,10,"%s",      onOff[gConfig.alarmMotorFault]);
  snprintf(fields[8].val,10,"%s",      auxModeStr[constrain(gConfig.aux1Mode,0,3)]);
  if      (gConfig.aux1Mode==AUX_MODE_LOCK)  snprintf(fields[9].val,10,"%us",  gConfig.aux1TimeParam);
  else if (gConfig.aux1Mode==AUX_MODE_LIGHT) snprintf(fields[9].val,10,"%umin",gConfig.aux1TimeParam);
  else                                        snprintf(fields[9].val,10,"--");
  snprintf(fields[10].val,10,"%s",     auxModeStr[constrain(gConfig.aux2Mode,0,3)]);
  if      (gConfig.aux2Mode==AUX_MODE_LOCK)  snprintf(fields[11].val,10,"%us",  gConfig.aux2TimeParam);
  else if (gConfig.aux2Mode==AUX_MODE_LIGHT) snprintf(fields[11].val,10,"%umin",gConfig.aux2TimeParam);
  else                                        snprintf(fields[11].val,10,"--");
  fields[0].label =tr("Leaf Delay",  "Delay Folha");
  fields[1].label =tr("Safety T/O",  "Timeout Seg");
  fields[2].label =tr("Sequence",    "Sequencia");
  fields[3].label =tr("Language",    "Idioma");
  fields[4].label =tr("Alert Open",  "Alerta Aberto");
  fields[5].label =tr("Brk-in Siren","Siren Arromba");
  fields[6].label =tr("Open Siren",  "Siren Aberto");
  fields[7].label =tr("Motor Fault", "Falha Motor");
  fields[8].label =tr("AUX1 Mode",   "AUX1 Modo");
  fields[9].label =(gConfig.aux1Mode==AUX_MODE_LOCK) ?tr("Lock Delay","Delay Trava"):
                   (gConfig.aux1Mode==AUX_MODE_LIGHT)?tr("Light Timer","Timer Luz"):
                    tr("AUX1 Param", "AUX1 Param");
  fields[10].label=tr("AUX2 Mode",   "AUX2 Modo");
  fields[11].label=(gConfig.aux2Mode==AUX_MODE_LOCK) ?tr("Lock Delay","Delay Trava"):
                   (gConfig.aux2Mode==AUX_MODE_LIGHT)?tr("Light Timer","Timer Luz"):
                    tr("AUX2 Param", "AUX2 Param");

  int visible=5;
  int scroll = menuCursor < visible   ? 0 :
               menuCursor >= 12       ? 7 : menuCursor - visible + 1;
  for (int i=0;i<visible;i++) {
    int fi=scroll+i;
    if (fi>=12) break;
    int y=MENU_Y0+i*MENU_LINE;
    bool sel=(fi==menuCursor);
    if (sel) menuDrawSelected(y);
    dText(10,y,fields[fi].label, sel?D_BG:D_YELLOW);
    dText(D_W-(int)strlen(fields[fi].val)*6-4,y,fields[fi].val, sel?D_BG:D_ACCENT);
  }
  int yBack=MENU_Y0+visible*MENU_LINE;
  bool selBack=(menuCursor==12);
  if (selBack) menuDrawSelected(yBack);
  dText(10,yBack,tr("Back","Voltar"),selBack?D_BG:D_YELLOW);
  dTft.drawFastHLine(4,yBack-3,D_W-8,D_YELLOW);
  char footer[22]; snprintf(footer,sizeof(footer),"%d/12 +/-nav OK edit",menuCursor+1);
  dTextCentered(D_H-10,footer,D_YELLOW);
  vTaskDelay(1/portTICK_PERIOD_MS);
}

void menuDrawConfigEdit() {
  dClear(); dHeader(tr("EDIT","EDITAR"));
  char title[32],val[20];
  const char* auxModeNames[]={tr("Disabled","Desab."),tr("Lock","Trava"),tr("Light","Luz"),tr("Siren","Sirene")};
  switch(menuEditField) {
    case 0:  strcpy(title,tr("Leaf Delay (ms)","Delay Folha (ms)"));
             snprintf(val,20,"%u",gConfig.leafDelayMs); break;
    case 1:  strcpy(title,tr("Safety T/O (ms)","Timeout Seg (ms)"));
             snprintf(val,20,"%u",gConfig.safetyTimeoutMs); break;
    case 2:  strcpy(title,tr("Sequence","Sequencia"));
             snprintf(val,20,"%s",gConfig.sequenceInverted?"INVERTED":"NORMAL"); break;
    case 3:  strcpy(title,tr("Language","Idioma"));
             snprintf(val,20,"%s",gConfig.language==0?"EN":"PT"); break;
    case 4:  strcpy(title,tr("Alert Open (min)","Alerta Aberto"));
             snprintf(val,20,"%u",gConfig.alertOpenMin); break;
    case 5:  strcpy(title,tr("Break-in Siren","Siren Arromba"));
             snprintf(val,20,"%s",gConfig.alarmBreakIn?tr("ON","ON"):tr("OFF","OFF")); break;
    case 6:  strcpy(title,tr("Open Long Siren","Siren Aberto"));
             snprintf(val,20,"%s",gConfig.alarmOpenSiren?tr("ON","ON"):tr("OFF","OFF")); break;
    case 7:  strcpy(title,tr("Motor Fault","Falha Motor"));
             snprintf(val,20,"%s",gConfig.alarmMotorFault?tr("ON","ON"):tr("OFF","OFF")); break;
    case 8:  strcpy(title,tr("AUX1 Mode","AUX1 Modo"));
             snprintf(val,20,"%s",auxModeNames[constrain(gConfig.aux1Mode,0,3)]); break;
    case 9:
      if (gConfig.aux1Mode==AUX_MODE_LOCK)       strcpy(title,tr("Lock Delay (s)","Delay Trava (s)"));
      else if (gConfig.aux1Mode==AUX_MODE_LIGHT)  strcpy(title,tr("Light Timer (min)","Timer Luz (min)"));
      else                                         strcpy(title,tr("AUX1 Param (n/a)","AUX1 Param (n/a)"));
      snprintf(val,20,"%u",gConfig.aux1TimeParam); break;
    case 10: strcpy(title,tr("AUX2 Mode","AUX2 Modo"));
             snprintf(val,20,"%s",auxModeNames[constrain(gConfig.aux2Mode,0,3)]); break;
    case 11:
      if (gConfig.aux2Mode==AUX_MODE_LOCK)       strcpy(title,tr("Lock Delay (s)","Delay Trava (s)"));
      else if (gConfig.aux2Mode==AUX_MODE_LIGHT)  strcpy(title,tr("Light Timer (min)","Timer Luz (min)"));
      else                                         strcpy(title,tr("AUX2 Param (n/a)","AUX2 Param (n/a)"));
      snprintf(val,20,"%u",gConfig.aux2TimeParam); break;
    default: strcpy(title,"?"); strcpy(val,"?"); break;
  }
  dTextCentered(30,title,D_YELLOW);
  dTft.fillRect(20,50,D_W-40,28,D_DIM);
  dTft.setTextColor(D_ACCENT); dTft.setTextSize(2);
  int x=max(22,(int)((D_W-(int)strlen(val)*12)/2));
  dTft.setCursor(x,55); dTft.print(val);
  dTextCentered(D_H-10,tr("+/- chg  OK save","+/- mud  OK salva"),D_YELLOW);
}

// Log menu: items 0..logCount-1 = entries (newest first), logCount = Clear, logCount+1 = Back
void menuDrawLog() {
  dClear(); dHeader(tr("EVENT LOG","REGISTRO"));

  int visible=4;
  int totalItems=(int)logCount+2; // entries + Clear + Back

  if (logCount==0) {
    dTextCentered(50,tr("No entries","Sem registros"),D_YELLOW);
  } else {
    for (int i=0;i<visible&&(logScrollPos+i)<(int)logCount;i++) {
      int entryIdx=logCount-1-(logScrollPos+i);
      LogEntry e=logGet(entryIdx);
      int y=MENU_Y0+i*MENU_LINE;
      int itemIdx=logScrollPos+i;
      bool sel=(menuCursor==itemIdx);
      if (sel) menuDrawSelected(y);
      char evtBuf[12]; snprintf(evtBuf,sizeof(evtBuf),"%-9s",logEventToStrLocal(e.event));
      dText(10,y,evtBuf,sel?D_BG:D_YELLOW);
      if (strncmp(e.timestamp,"00000000000000",14)!=0) {
        char tsBuf[12];
        snprintf(tsBuf,sizeof(tsBuf),"%.2s/%.2s %.2s:%.2s",
          e.timestamp+6, e.timestamp+4,
          e.timestamp+8, e.timestamp+10);
        dText(D_W-66,y,tsBuf,sel?D_BG:D_DIM);
      }
    }
  }

  // Action items
  int y0=MENU_Y0+visible*MENU_LINE+2;
  dTft.drawFastHLine(4,y0-2,D_W-8,D_YELLOW);

  bool selClear=(menuCursor==(int)logCount);
  bool selBack =(menuCursor==(int)logCount+1);

  if (selClear) menuDrawSelected(y0);
  dText(10,y0, tr("Clear Log","Limpar Log"), selClear?D_BG:D_RED);

  if (selBack) menuDrawSelected(y0+MENU_LINE);
  dText(10,y0+MENU_LINE,tr("Back","Voltar"), selBack?D_BG:D_YELLOW);

  char footer[24];
  snprintf(footer,sizeof(footer),"%u/%u +/-nav OK sel",(unsigned)(menuCursor+1),(unsigned)totalItems);
  dTextCentered(D_H-10,footer,D_YELLOW);
}

void menuDrawWifi() {
  const char* items[]={tr("Reset WiFi","Resetar WiFi"),tr("Back","Voltar")};
  menuDrawList("WIFI",items,2,menuCursor);
}

// ═══════════════════════════════════════════════════════════════
// MENU LOGIC
// ═══════════════════════════════════════════════════════════════

// Return to idle screen from any menu state.
void menuReturnToIdle() {
  menuState=MENU_IDLE;
  displayIdleReset();       // Force full idle repaint after menu exit
  displayDrawIdleFrame();
}

void menuHandleBtn(BtnEvent_t btn) {
  menuLastInput=millis();

  if (menuState==MENU_IDLE) {
    if (btn==BTN_OK) { menuState=MENU_ROOT; menuCursor=0; menuDrawRoot(); }
    return;
  }

  switch(menuState) {

    case MENU_ROOT:
      if (btn==BTN_PLUS)  { menuCursor=(menuCursor+1)%5; menuDrawRoot(); }
      if (btn==BTN_MINUS) { menuCursor=(menuCursor+4)%5; menuDrawRoot(); }
      if (btn==BTN_OK) {
        switch(menuCursor) {
          case 0: menuState=MENU_GATE_CTRL; menuCursor=0; menuDrawGateCtrl(); break;
          case 1: menuState=MENU_CONFIG;    menuCursor=0; menuDrawConfig();   break;
          case 2: menuState=MENU_LOG; logScrollPos=0; menuCursor=0; menuDrawLog(); break;
          case 3: menuState=MENU_WIFI;      menuCursor=0; menuDrawWifi();     break;
          case 4: menuReturnToIdle(); break;
        }
      }
      break;

    case MENU_GATE_CTRL:
      if (btn==BTN_PLUS||btn==BTN_MINUS) { menuCursor^=1; menuDrawGateCtrl(); }
      if (btn==BTN_OK) {
        if (menuCursor==0) { GateCmd_t cmd=CMD_TOGGLE; xQueueSend(gCmdQueue,&cmd,0); }
        menuState=MENU_ROOT; menuCursor=0; menuDrawRoot();
      }
      break;

    case MENU_CONFIG:
      if (btn==BTN_PLUS)  { menuCursor=(menuCursor+1)%13; menuDrawConfig(); }
      if (btn==BTN_MINUS) { menuCursor=(menuCursor+12)%13; menuDrawConfig(); }
      if (btn==BTN_OK) {
        if (menuCursor==12) { menuState=MENU_ROOT; menuCursor=0; menuDrawRoot(); }
        else { menuEditField=menuCursor; menuState=MENU_CONFIG_EDIT; menuDrawConfigEdit(); }
      }
      break;

    case MENU_CONFIG_EDIT:
      if (btn==BTN_PLUS||btn==BTN_MINUS) {
        int dir=(btn==BTN_PLUS)?1:-1;
        switch(menuEditField) {
          case 0:  gConfig.leafDelayMs     =constrain((int)gConfig.leafDelayMs    +dir*100,0,    30000); break;
          case 1:  gConfig.safetyTimeoutMs =constrain((int)gConfig.safetyTimeoutMs+dir*500,1000, 60000); break;
          case 2:  gConfig.sequenceInverted=!gConfig.sequenceInverted; break;
          case 3:  gConfig.language        =gConfig.language==0?1:0; break;
          case 4:  gConfig.alertOpenMin    =constrain((int)gConfig.alertOpenMin   +dir,0,120); break;
          case 5:  gConfig.alarmBreakIn    =!gConfig.alarmBreakIn; break;
          case 6:  gConfig.alarmOpenSiren  =!gConfig.alarmOpenSiren; break;
          case 7:  gConfig.alarmMotorFault =!gConfig.alarmMotorFault; break;
          case 8:  gConfig.aux1Mode        =(gConfig.aux1Mode+4+dir)%4; break;
          case 9:  if (gConfig.aux1Mode==AUX_MODE_LOCK||gConfig.aux1Mode==AUX_MODE_LIGHT)
                     gConfig.aux1TimeParam=constrain((int)gConfig.aux1TimeParam+dir,0,30); break;
          case 10: gConfig.aux2Mode        =(gConfig.aux2Mode+4+dir)%4; break;
          case 11: if (gConfig.aux2Mode==AUX_MODE_LOCK||gConfig.aux2Mode==AUX_MODE_LIGHT)
                     gConfig.aux2TimeParam=constrain((int)gConfig.aux2TimeParam+dir,0,30); break;
        }
        menuDrawConfigEdit();
      }
      if (btn==BTN_OK) { configSave(); menuState=MENU_CONFIG; menuDrawConfig(); }
      break;

    // Log menu: cursor navigates entries + Clear + Back
    // scroll window follows cursor for entries
    case MENU_LOG: {
      int totalItems=(int)logCount+2;
      if (btn==BTN_PLUS) {
        menuCursor=(menuCursor+1)%totalItems;
        // scroll window: keep cursor in view for entry items
        if (menuCursor<(int)logCount) {
          if (menuCursor>=logScrollPos+4) logScrollPos=menuCursor-3;
          if (menuCursor<logScrollPos)    logScrollPos=menuCursor;
        }
        menuDrawLog();
      }
      if (btn==BTN_MINUS) {
        menuCursor=(menuCursor+totalItems-1)%totalItems;
        if (menuCursor<(int)logCount) {
          if (menuCursor>=logScrollPos+4) logScrollPos=menuCursor-3;
          if (menuCursor<logScrollPos)    logScrollPos=menuCursor;
        }
        menuDrawLog();
      }
      if (btn==BTN_OK) {
        if (menuCursor==(int)logCount) {
          logClear(); logScrollPos=0; menuCursor=0; menuDrawLog();
        } else if (menuCursor==(int)logCount+1) {
          menuState=MENU_ROOT; menuCursor=0; menuDrawRoot();
        }
      }
      break;
    }

    case MENU_WIFI:
      if (btn==BTN_PLUS||btn==BTN_MINUS) { menuCursor^=1; menuDrawWifi(); }
      if (btn==BTN_OK) {
        if (menuCursor==0) {
          gPrefs.begin(NVS_NS_WIFI,false); gPrefs.clear(); gPrefs.end();
          dClear();
          dTextCentered(50,tr("WiFi cleared","WiFi apagado"),D_YELLOW);
          dTextCentered(65,tr("Restarting...","Reiniciando..."),D_WHITE);
          delay(2000); ESP.restart();
        }
        menuState=MENU_ROOT; menuCursor=0; menuDrawRoot();
      }
      break;

    default: break;
  }
}

// ═══════════════════════════════════════════════════════════════
// TASKS
// Core 0 (prio 1): taskMonitor, taskWeb
// Core 1 (prio 2): taskGateController, taskInputHandler
// Core 1 (prio 1): taskDisplay
// WiFi stack runs on Core 0. Gate control and display on Core 1
// to prevent network latency from affecting relay timing or UI.
// ═══════════════════════════════════════════════════════════════

void taskGateController(void* param) {
  GateCmd_t cmd;
  for (;;) {
    if (xQueueReceive(gCmdQueue,&cmd,portMAX_DELAY)!=pdTRUE) continue;
    GateState_t state=getState();
    switch(state) {
      case GATE_CLOSED:
        setState(GATE_OPENING);
        if (sequenceOpen()) { setState(GATE_OPEN); logWrite(LOG_EVT_OPENED); } break;
      case GATE_OPEN:
        setState(GATE_CLOSING);
        if (sequenceClose()) { setState(GATE_CLOSED); logWrite(LOG_EVT_CLOSED); } break;
      case GATE_STOPPED:
        if (getLastMovement()==GATE_OPENING) {
          setState(GATE_CLOSING);
          if (sequenceClose()) { setState(GATE_CLOSED); logWrite(LOG_EVT_CLOSED); }
        } else {
          setState(GATE_OPENING);
          if (sequenceOpen()) { setState(GATE_OPEN); logWrite(LOG_EVT_OPENED); }
        } break;
      case GATE_ERROR:
        relayAllOff(); setState(GATE_CLOSED); logWrite(LOG_EVT_ERROR); break;
      default: break;
    }
  }
}

void taskInputHandler(void* param) {
  bool lastGate=HIGH,lastPlus=HIGH,lastMinus=HIGH,lastOk=HIGH;
  for (;;) {
    bool gate=digitalRead(PIN_BTN_GATE);
    bool plus=digitalRead(PIN_BTN_PLUS);
    bool minus=digitalRead(PIN_BTN_MINUS);
    bool ok=digitalRead(PIN_BTN_OK);

    if (gate==LOW&&lastGate==HIGH) {
      vTaskDelay(50/portTICK_PERIOD_MS);
      if (digitalRead(PIN_BTN_GATE)==LOW) {
        if (menuState==MENU_IDLE) {
          GateCmd_t cmd=CMD_TOGGLE; xQueueSend(gCmdQueue,&cmd,0);
        } else {
          BtnEvent_t b=BTN_GATE; xQueueSend(gBtnQueue,&b,0);
        }
        Serial.println("[INPUT] GATE pressed");
      }
    }
    lastGate=gate;

#define BTN_REPEAT_DELAY_MS  600   // time held before repeat starts
#define BTN_REPEAT_RATE_MS   120   // repeat interval

    if (plus==LOW&&lastPlus==HIGH) {
      vTaskDelay(50/portTICK_PERIOD_MS);
      if (digitalRead(PIN_BTN_PLUS)==LOW) {
        BtnEvent_t b=BTN_PLUS; xQueueSend(gBtnQueue,&b,0);
        unsigned long held=millis();
        while (digitalRead(PIN_BTN_PLUS)==LOW) {
          if (millis()-held>=BTN_REPEAT_DELAY_MS) {
            xQueueSend(gBtnQueue,&b,0);
            vTaskDelay(BTN_REPEAT_RATE_MS/portTICK_PERIOD_MS);
          } else vTaskDelay(10/portTICK_PERIOD_MS);
        }
      }
    }
    lastPlus=plus;

    if (minus==LOW&&lastMinus==HIGH) {
      vTaskDelay(50/portTICK_PERIOD_MS);
      if (digitalRead(PIN_BTN_MINUS)==LOW) {
        BtnEvent_t b=BTN_MINUS; xQueueSend(gBtnQueue,&b,0);
        unsigned long held=millis();
        while (digitalRead(PIN_BTN_MINUS)==LOW) {
          if (millis()-held>=BTN_REPEAT_DELAY_MS) {
            xQueueSend(gBtnQueue,&b,0);
            vTaskDelay(BTN_REPEAT_RATE_MS/portTICK_PERIOD_MS);
          } else vTaskDelay(10/portTICK_PERIOD_MS);
        }
      }
    }
    lastMinus=minus;

    if (ok==LOW&&lastOk==HIGH) {
      unsigned long t0=millis();
      while (digitalRead(PIN_BTN_OK)==LOW) vTaskDelay(10/portTICK_PERIOD_MS);
      unsigned long held=millis()-t0;
      BtnEvent_t b=(held>=LONG_PRESS_MS)?BTN_OK_LONG:BTN_OK;
      xQueueSend(gBtnQueue,&b,0);
      Serial.printf("[INPUT] OK %s\n",held>=LONG_PRESS_MS?"LONG":"SHORT");
    }
    lastOk=ok;

    xSemaphoreTake(gMutex,portMAX_DELAY);
    gStatus.reed_open_l1 =(digitalRead(PIN_REED_OPEN_L1) ==LOW);
    gStatus.reed_close_l1=(digitalRead(PIN_REED_CLOSE_L1)==LOW);
    gStatus.reed_open_l2 =(digitalRead(PIN_REED_OPEN_L2) ==LOW);
    gStatus.reed_close_l2=(digitalRead(PIN_REED_CLOSE_L2)==LOW);
    xSemaphoreGive(gMutex);

    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void taskMonitor(void* param) {
  unsigned long lastPrint=0, lastAlert=0;
  for (;;) {
    unsigned long now=millis();
    xSemaphoreTake(gMutex,portMAX_DELAY);
    GateState_t state=gStatus.state;
    unsigned long elapsed=now-gStatus.lastStateChange;
    bool cl1=gStatus.reed_close_l1, cl2=gStatus.reed_close_l2;
    bool ol1=gStatus.reed_open_l1,  ol2=gStatus.reed_open_l2;
    xSemaphoreGive(gMutex);

    // ── Break-in detection ───────────────────────────────────
    // Only active when GATE_CLOSED and no motor sequence running.
    // gMotorActive is set by sequenceOpen/Close to prevent false positives
    // during the window between setState(GATE_OPENING) and FC reeds moving.
    static bool armedFC1=false, armedFC2=false;
    if (state==GATE_CLOSED && !gMotorActive) {
      // Arm when reed is confirmed closed
      if (cl1) armedFC1=true;
      if (cl2) armedFC2=true;
      // Trigger when armed reed goes OFF (portao forced open)
      if (armedFC1 && !cl1) {
        alarmTrigger(ALARM_BREAKIN_FC1, LOG_EVT_BREAKIN_FC1);
        armedFC1=false;  // disarm to avoid repeat
      }
      if (armedFC2 && !cl2) {
        alarmTrigger(ALARM_BREAKIN_FC2, LOG_EVT_BREAKIN_FC2);
        armedFC2=false;
      }
      // Clear break-in alarm only when BOTH close reeds are active again
      if ((gAlarm==ALARM_BREAKIN_FC1 || gAlarm==ALARM_BREAKIN_FC2) && cl1 && cl2) {
        alarmClear(); armedFC1=true; armedFC2=true;
      }
    } else {
      // Reset arming when gate moves — normal operation
      armedFC1=false; armedFC2=false;
    }

    // ── Open too long ────────────────────────────────────────
    if (state==GATE_OPEN && gConfig.alertOpenMin>0 &&
        elapsed>=gConfig.alertOpenMin*60000UL && now-lastAlert>=60000UL) {
      lastAlert=now;
      gAlertOpen=true;
      alarmTrigger(ALARM_OPEN_LONG, LOG_EVT_OPEN_TOO_LONG);
      Serial.printf("[ALERT] Open for %lu min\n",elapsed/60000UL);
    }

    // ── Clear alarms when gate closes ────────────────────────
    // Only clear on transition to CLOSED, not every cycle while CLOSED
    {
      static GateState_t lastState = GATE_CLOSED;
      if (state==GATE_CLOSED && lastState!=GATE_CLOSED) {
        gAlertOpen=false;
        lastAlert=0;
        alarmClear();
      }
      lastState=state;
    }

    auxTick();

    if (now-lastPrint>=5000) {
      lastPrint=now;
      xSemaphoreTake(gMutex,portMAX_DELAY);
      Serial.printf("[STATUS] %-8s | OL1:%d CL1:%d OL2:%d CL2:%d | %lus\n",
        stateToStr(gStatus.state),
        gStatus.reed_open_l1,gStatus.reed_close_l1,
        gStatus.reed_open_l2,gStatus.reed_close_l2,
        elapsed/1000);
      xSemaphoreGive(gMutex);
    }
    vTaskDelay(200/portTICK_PERIOD_MS);  // faster poll for break-in detection
  }
}

void taskWeb(void* param) {
  for (;;) { server.handleClient(); vTaskDelay(5/portTICK_PERIOD_MS); }
}

void taskDisplay(void* param) {
  if (!displayPresent) vTaskDelete(nullptr);

  displayDrawIdleFrame();
  displayIdleReset();

  unsigned long lastTime=0,lastArrow=0;

  for (;;) {
    unsigned long now=millis();

    // Menu timeout
    if (menuState!=MENU_IDLE&&now-menuLastInput>=MENU_TIMEOUT_MS) {
      menuReturnToIdle();
    }

    // Process buttons
    BtnEvent_t btn;
    while (xQueueReceive(gBtnQueue,&btn,0)==pdTRUE) {
      menuHandleBtn(btn);
    }

    if (menuState==MENU_IDLE) {
      xSemaphoreTake(gMutex,portMAX_DELAY);
      GateState_t s=gStatus.state;
      bool ol1=gStatus.reed_open_l1,cl1=gStatus.reed_close_l1;
      bool ol2=gStatus.reed_open_l2,cl2=gStatus.reed_close_l2;
      xSemaphoreGive(gMutex);

      displayDrawIdle(s,ol1,cl1,ol2,cl2);

      if ((s==GATE_OPENING||s==GATE_CLOSING)&&now-lastArrow>=200) {
        lastArrow=now;
        arrowFrame = (s==GATE_OPENING) ? (arrowFrame+1)%5 : (arrowFrame+4)%5;
      }
      if (ntpSynced&&now-lastTime>=1000) { lastTime=now; displayDrawTime(); }
    }

    vTaskDelay(50/portTICK_PERIOD_MS);
  }
}

// ═══════════════════════════════════════════════════════════════
// HTML
// ═══════════════════════════════════════════════════════════════

const char HTML_PORTAL[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Gate - WiFi</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#0a0a0a;color:#e0e0e0;display:flex;justify-content:center;align-items:center;min-height:100vh;padding:1rem}
.card{background:#111;border:1px solid #222;border-radius:10px;padding:2rem;width:100%;max-width:360px}
h1{font-size:.8rem;color:#4fc3f7;letter-spacing:.15em;text-transform:uppercase;margin-bottom:.25rem}
p.sub{font-size:.7rem;color:#666;margin-bottom:1.5rem;padding-bottom:1rem;border-bottom:1px solid #1e1e1e}
label{display:block;font-size:.65rem;color:#888;text-transform:uppercase;letter-spacing:.08em;margin-bottom:.3rem}
input{width:100%;background:#0a0a0a;border:1px solid #222;border-radius:4px;color:#e0e0e0;font-family:monospace;font-size:.9rem;padding:.6rem .75rem;margin-bottom:1rem;outline:none;transition:border-color .2s}
input:focus{border-color:#4fc3f7}
button{width:100%;background:#4fc3f7;color:#0a0a0a;border:none;border-radius:4px;font-family:monospace;font-size:.85rem;font-weight:bold;padding:.7rem;cursor:pointer}
.msg{margin-top:1rem;font-size:.7rem;color:#4fc3f7;text-align:center}
</style></head><body>
<div class="card">
  <h1>ESP32 Gate</h1><p class="sub">WiFi Configuration</p>
  <label>Network (SSID)</label>
  <input type="text" id="ssid" placeholder="Network name" autocomplete="off">
  <label>Password</label>
  <input type="password" id="pass" placeholder="Password">
  <button onclick="save()">Save &amp; Restart</button>
  <div class="msg" id="msg"></div>
</div>
<script>
function save(){
  const s=document.getElementById('ssid').value.trim();
  const p=document.getElementById('pass').value;
  if(!s){document.getElementById('msg').textContent='Enter network name.';return;}
  document.getElementById('msg').textContent='Saving...';
  fetch('/save?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p))
    .then(r=>r.text()).then(t=>{document.getElementById('msg').textContent=t;})
    .catch(()=>{document.getElementById('msg').textContent='Error.';});
}
</script></body></html>
)rawliteral";

// ── Web UI i18n table ─────────────────────────────────────────
// Order must match lang index: 0=EN, 1=PT
// Keys used as %KEY% placeholders in HTML_DASH_TMPL
struct WebStr { const char* key; const char* val[2]; };
static const WebStr WEB_STRINGS[] = {
  {"TAB_STATUS",   {"Status",          "Estado"          }},
  {"TAB_CONFIG",   {"Config",          "Config"          }},
  {"TAB_LOG",      {"Log",             "Log"             }},
  {"LBL_STATE",    {"Gate State",      "Estado do Portao"}},
  {"LBL_S1O",      {"FC1 Open",        "FC1 Aberto"      }},
  {"LBL_S1C",      {"FC1 Close",       "FC1 Fechado"     }},
  {"LBL_S2O",      {"FC2 Open",        "FC2 Aberto"      }},
  {"LBL_S2C",      {"FC2 Close",       "FC2 Fechado"     }},
  {"BTN_TOGGLE",   {"Toggle Gate",     "Acionar Portao"  }},
  {"BTN_WIFIRST",  {"Reset WiFi",      "Resetar WiFi"    }},
  {"LBL_DELAY",    {"Leaf Delay (ms)", "Delay Folha (ms)"}},
  {"DSC_DELAY",    {"Delay before activating second leaf","Delay antes de acionar segunda folha"}},
  {"LBL_SAFETY",   {"Safety Timeout (ms)","Timeout Seguranca (ms)"}},
  {"DSC_SAFETY",   {"Max movement time before force-stop","Tempo max de movimento antes de parar"}},
  {"LBL_ALERT",    {"Alert if Open (min)","Alerta se Aberto (min)"}},
  {"DSC_ALERT",    {"Alert if gate open longer than this","Alerta se portao ficar aberto demais"}},
  {"LBL_SEQINV",   {"Sequence Inverted","Sequencia Invertida"}},
  {"DSC_SEQINV",   {"L2 leads on open, L1 leads on close","L2 abre primeiro, L1 fecha primeiro"}},
  {"LBL_LANG",     {"Language",        "Idioma"          }},
  {"LBL_NORMAL",   {"Normal",          "Normal"          }},
  {"LBL_INVERTED", {"Inverted",        "Invertida"       }},
  {"BTN_SAVE",     {"Save Configuration","Salvar Configuracao"}},
  {"BTN_WIFICFG",  {"Reset WiFi Credentials","Resetar Credenciais WiFi"}},
  {"LBL_ENTRIES",  {"entries",         "registros"       }},
  {"BTN_CLEAR",    {"Clear Log",       "Limpar Log"      }},
  {"TH_EVENT",     {"Event",           "Evento"          }},
  {"TH_TIME",      {"Timestamp",       "Data/Hora"       }},
  {"TXT_LOADING",  {"Loading...",      "Carregando..."   }},
  {"TXT_NOENTRIES",{"No entries",      "Sem registros"   }},
  {"CFM_WIFIRST",  {"Reset WiFi?",     "Resetar WiFi?"   }},
  {"CFM_CLEAR",    {"Clear all log entries?","Limpar todos os registros?"}},
  {"TXT_SAVED",    {"Configuration saved.","Configuracao salva."}},
  // Stage 7 — Aux outputs
  {"LBL_AUX1MODE",  {"AUX1 Mode",         "AUX1 Modo"           }},
  {"DSC_AUX1MODE",  {"AUX1 relay function","Funcao do rele AUX1" }},
  {"LBL_AUX1DLY",  {"Lock Release Delay (ms)","Delay Liberacao Trava (ms)"}},
  {"DSC_AUX1DLY",  {"Wait before opening after lock releases","Aguarda antes de abrir apos liberar trava"}},
  {"LBL_AUX2MODE",  {"AUX2 Mode",         "AUX2 Modo"           }},
  {"DSC_AUX2MODE",  {"AUX2 relay function","Funcao do rele AUX2" }},
  {"LBL_AUX2TMR",  {"Light Timer (min)",  "Timer Luz (min)"     }},
  {"DSC_AUX2TMR",  {"Minutes light stays on after gate closes (0=off)","Minutos que a luz fica acesa apos fechar (0=deslig.)"}},
  {"OPT_DISABLED",  {"Disabled",          "Desabilitado"        }},
  {"OPT_LOCK",      {"Lock",              "Trava"               }},
  {"OPT_LIGHT",     {"Light",             "Luz"                 }},
  {"OPT_SIREN",     {"Siren",             "Sirene"              }},
  {"BTN_TEST",      {"Test",              "Testar"              }},
  {"LBL_AUX_TEST",  {"Aux Test",          "Teste Aux"           }},
  // Stage 8 — Alarms
  {"SEC_ALARMS",    {"Alarms",            "Alarmes"             }},
  {"LBL_ALM_OPEN",  {"Open Too Long (min)","Aberto Demais (min)"}},
  {"DSC_ALM_OPEN",  {"0 = disabled. Alert if gate stays open longer than this","0 = desabilitado. Alerta se portao ficar aberto demais"}},
  {"LBL_ALM_BREAKIN",{"Break-in Siren",   "Sirene Arrombamento" }},
  {"DSC_ALM_BREAKIN",{"Sound siren if gate is forced open","Aciona sirene se portao for aberto a forca"}},
  {"LBL_ALM_OPENSRN",{"Open Too Long Siren","Sirene Aberto Demais"}},
  {"DSC_ALM_OPENSRN",{"Sound siren when gate stays open too long","Aciona sirene quando portao fica aberto demais"}},
  {"LBL_ALM_MOTOR", {"Motor Fault Detect","Detectar Falha Motor"}},
  {"DSC_ALM_MOTOR", {"Stop and alert if motor relay fires but reed doesn't respond in 5s","Para e alerta se rele acionar mas FC nao responder em 5s"}},
};
#define WEB_STRINGS_COUNT (sizeof(WEB_STRINGS)/sizeof(WEB_STRINGS[0]))

const char HTML_DASH_TMPL[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Gate</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#0a0a0a;color:#e0e0e0;min-height:100vh}
.header{background:#111;border-bottom:1px solid #1e1e1e;padding:.75rem 1rem;display:flex;align-items:center;justify-content:space-between}
.header h1{font-size:.75rem;color:#4fc3f7;letter-spacing:.15em;text-transform:uppercase}
.dot{display:inline-block;width:7px;height:7px;border-radius:50%;background:#4fc3f7;margin-right:6px;animation:pulse 2s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.ip{font-size:.65rem;color:#444}
.tabs{display:flex;background:#0f0f0f;border-bottom:1px solid #1e1e1e}
.tab{flex:1;padding:.6rem;text-align:center;font-size:.65rem;color:#444;cursor:pointer;letter-spacing:.08em;text-transform:uppercase;border-bottom:2px solid transparent;transition:all .2s}
.tab.active{color:#4fc3f7;border-bottom-color:#4fc3f7}
.panel{display:none;padding:1rem}.panel.active{display:block}
.status-box{background:#111;border:1px solid #1e1e1e;border-radius:8px;padding:1.5rem;text-align:center;margin-bottom:1rem}
.state-label{font-size:.6rem;color:#888;text-transform:uppercase;letter-spacing:.1em;margin-bottom:.5rem}
.state-value{font-size:2rem;font-weight:bold;color:#4fc3f7}
.reeds{display:grid;grid-template-columns:1fr 1fr;gap:.5rem;margin-bottom:1rem}
.reed-box{background:#111;border:1px solid #1e1e1e;border-radius:6px;padding:.75rem;text-align:center}
.reed-label{font-size:.6rem;color:#888;text-transform:uppercase;margin-bottom:.5rem}
.reed-led{display:inline-block;width:14px;height:14px;border-radius:50%;background:#1e1e1e;transition:background .3s}
.reed-led.on{background:#00c853;box-shadow:0 0 6px #00c85340}
.btn-toggle{width:100%;background:#4fc3f7;color:#0a0a0a;border:none;border-radius:6px;font-family:monospace;font-size:.9rem;font-weight:bold;padding:.9rem;cursor:pointer;margin-bottom:.5rem}
.btn-reset{width:100%;background:transparent;color:#555;border:1px solid #222;border-radius:6px;font-family:monospace;font-size:.75rem;padding:.5rem;cursor:pointer}
.form-group{margin-bottom:1rem}
.form-label{font-size:.65rem;color:#888;text-transform:uppercase;letter-spacing:.08em;display:block;margin-bottom:.3rem}
.form-desc{font-size:.6rem;color:#444;margin-bottom:.3rem}
input[type=number],select{width:100%;background:#0a0a0a;border:1px solid #222;border-radius:4px;color:#e0e0e0;font-family:monospace;font-size:.85rem;padding:.5rem .75rem;outline:none}
input:focus,select:focus{border-color:#4fc3f7}
.toggle-wrap{display:flex;align-items:center;gap:.75rem}
.toggle{position:relative;width:36px;height:20px;cursor:pointer}
.toggle input{opacity:0;width:0;height:0}
.slider{position:absolute;top:0;left:0;right:0;bottom:0;background:#222;border-radius:20px;transition:.3s}
.slider:before{content:"";position:absolute;width:14px;height:14px;left:3px;bottom:3px;background:#666;border-radius:50%;transition:.3s}
input:checked+.slider{background:#4fc3f7}
input:checked+.slider:before{transform:translateX(16px);background:#0a0a0a}
.btn-save{width:100%;background:#4fc3f7;color:#0a0a0a;border:none;border-radius:4px;font-family:monospace;font-size:.85rem;font-weight:bold;padding:.7rem;cursor:pointer;margin-top:.5rem}
.btn-aux-test{flex:1;background:transparent;color:#4fc3f7;border:1px solid #4fc3f7;border-radius:4px;font-family:monospace;font-size:.75rem;padding:.5rem;cursor:pointer}
.btn-aux-test:active{background:rgba(79,195,247,.15)}
.btn-wifi{width:100%;background:transparent;color:#e74c3c;border:1px solid #e74c3c;border-radius:4px;font-family:monospace;font-size:.75rem;padding:.5rem;cursor:pointer;margin-top:.5rem}
.msg{font-size:.7rem;color:#4fc3f7;margin-top:.5rem;text-align:center;min-height:1rem}
.log-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:.75rem}
.log-count{font-size:.65rem;color:#444}
.btn-clear{background:transparent;color:#e74c3c;border:1px solid #e74c3c;border-radius:4px;font-family:monospace;font-size:.65rem;padding:.25rem .6rem;cursor:pointer}
.log-table{width:100%;border-collapse:collapse;font-size:.7rem}
.log-table th{text-align:left;color:#888;font-size:.6rem;text-transform:uppercase;padding:.4rem .5rem;border-bottom:1px solid #1e1e1e}
.log-table td{padding:.4rem .5rem;border-bottom:1px solid #0f0f0f;color:#888}
.log-table td:first-child{color:#e0e0e0;font-weight:bold}
.log-empty{text-align:center;color:#333;padding:2rem;font-size:.75rem}
</style></head><body>
<div class="header"><h1><span class="dot"></span>ESP32 Gate</h1><span class="ip" id="hdr-ip">...</span></div>
<div class="tabs">
  <div class="tab active" onclick="showTab('dash')">%TAB_STATUS%</div>
  <div class="tab" onclick="showTab('cfg')">%TAB_CONFIG%</div>
  <div class="tab" onclick="showTab('log')">%TAB_LOG%</div>
</div>
<div class="panel active" id="tab-dash">
  <div class="status-box"><div class="state-label">%LBL_STATE%</div><div class="state-value" id="val-state">-</div></div>
  <div class="reeds">
    <div class="reed-box"><div class="reed-label">%LBL_S1O%</div><div class="reed-led" id="r-ol1"></div></div>
    <div class="reed-box"><div class="reed-label">%LBL_S1C%</div><div class="reed-led" id="r-cl1"></div></div>
    <div class="reed-box"><div class="reed-label">%LBL_S2O%</div><div class="reed-led" id="r-ol2"></div></div>
    <div class="reed-box"><div class="reed-label">%LBL_S2C%</div><div class="reed-led" id="r-cl2"></div></div>
  </div>
  <button class="btn-toggle" onclick="toggleGate()">%BTN_TOGGLE%</button>
  <button class="btn-reset" onclick="if(confirm('%CFM_WIFIRST%'))location='/wifi-reset'">%BTN_WIFIRST%</button>
</div>
<div class="panel" id="tab-cfg">
  <div class="form-group"><label class="form-label">%LBL_DELAY%</label><div class="form-desc">%DSC_DELAY%</div><input type="number" id="c-delay" min="0" max="30000" step="100"></div>
  <div class="form-group"><label class="form-label">%LBL_SAFETY%</label><div class="form-desc">%DSC_SAFETY%</div><input type="number" id="c-safety" min="1000" max="60000" step="500"></div>
  <div class="form-group"><label class="form-label">%LBL_SEQINV%</label><div class="form-desc">%DSC_SEQINV%</div><div class="toggle-wrap"><label class="toggle"><input type="checkbox" id="c-inv"><span class="slider"></span></label><span style="font-size:.75rem;color:#888" id="inv-lbl">%LBL_NORMAL%</span></div></div>
  <div class="form-group"><label class="form-label">%LBL_LANG%</label><select id="c-lang"><option value="0">English</option><option value="1">Portugues</option></select></div>
  <div style="margin:12px 0 6px;font-size:.6rem;color:#4fc3f7;text-transform:uppercase;letter-spacing:.1em;border-bottom:1px solid #1e1e1e;padding-bottom:4px">%SEC_ALARMS%</div>
  <div class="form-group"><label class="form-label">%LBL_ALM_OPEN%</label><div class="form-desc">%DSC_ALM_OPEN%</div><input type="number" id="c-alertmin" min="0" max="120" step="1"></div>
  <div class="form-group"><label class="form-label">%LBL_ALM_BREAKIN%</label><div class="form-desc">%DSC_ALM_BREAKIN%</div><div class="toggle-wrap"><label class="toggle"><input type="checkbox" id="c-alm-breakin"><span class="slider"></span></label></div></div>
  <div class="form-group"><label class="form-label">%LBL_ALM_OPENSRN%</label><div class="form-desc">%DSC_ALM_OPENSRN%</div><div class="toggle-wrap"><label class="toggle"><input type="checkbox" id="c-alm-opensrn"><span class="slider"></span></label></div></div>
  <div class="form-group"><label class="form-label">%LBL_ALM_MOTOR%</label><div class="form-desc">%DSC_ALM_MOTOR%</div><div class="toggle-wrap"><label class="toggle"><input type="checkbox" id="c-alm-motor"><span class="slider"></span></label></div></div>
  <div style="margin:12px 0 6px;font-size:.6rem;color:#4fc3f7;text-transform:uppercase;letter-spacing:.1em;border-bottom:1px solid #1e1e1e;padding-bottom:4px">Aux Outputs</div>
  <div class="form-group"><label class="form-label">%LBL_AUX1MODE%</label><div class="form-desc">%DSC_AUX1MODE%</div><select id="c-aux1mode" onchange="updateAuxParam(1,true)"><option value="0">%OPT_DISABLED%</option><option value="1">%OPT_LOCK%</option><option value="2">%OPT_LIGHT%</option><option value="3">%OPT_SIREN%</option></select></div>
  <div class="form-group" id="aux1-param-group"><label class="form-label" id="aux1-param-lbl">--</label><input type="number" id="c-aux1time" min="0" step="1"></div>
  <div class="form-group"><label class="form-label">%LBL_AUX2MODE%</label><div class="form-desc">%DSC_AUX2MODE%</div><select id="c-aux2mode" onchange="updateAuxParam(2,true)"><option value="0">%OPT_DISABLED%</option><option value="1">%OPT_LOCK%</option><option value="2">%OPT_LIGHT%</option><option value="3">%OPT_SIREN%</option></select></div>
  <div class="form-group" id="aux2-param-group"><label class="form-label" id="aux2-param-lbl">--</label><input type="number" id="c-aux2time" min="0" step="1"></div>
  <div id="aux-test-section" style="display:none;margin:12px 0 6px">    <div style="font-size:.6rem;color:#4fc3f7;text-transform:uppercase;letter-spacing:.1em;border-bottom:1px solid #1e1e1e;padding-bottom:4px;margin-bottom:8px">%LBL_AUX_TEST%</div>
    <div style="display:flex;gap:8px">
      <button id="btn-aux1-test" class="btn-aux-test" onclick="testAux(1)" style="display:none">AUX1 — %BTN_TEST%</button>
      <button id="btn-aux2-test" class="btn-aux-test" onclick="testAux(2)" style="display:none">AUX2 — %BTN_TEST%</button>
    </div>
    <div class="msg" id="aux-test-msg"></div>
  </div>
  <button class="btn-save" onclick="saveConfig()">%BTN_SAVE%</button>
  <button class="btn-wifi" onclick="if(confirm('%CFM_WIFIRST%'))location='/wifi-reset'">%BTN_WIFICFG%</button>
  <div class="msg" id="cfg-msg"></div>
</div>
<div class="panel" id="tab-log">
  <div class="log-header"><span class="log-count" id="log-count">0 %LBL_ENTRIES%</span><button class="btn-clear" onclick="clearLog()">%BTN_CLEAR%</button></div>
  <table class="log-table"><thead><tr><th>%TH_EVENT%</th><th>%TH_TIME%</th></tr></thead><tbody id="log-body"><tr><td colspan="2" class="log-empty">%TXT_LOADING%</td></tr></tbody></table>
</div>
<script>
var T={noEntries:'%TXT_NOENTRIES%',entries:' %LBL_ENTRIES%',cfmClear:'%CFM_CLEAR%',normal:'%LBL_NORMAL%',inverted:'%LBL_INVERTED%'};
function showTab(t){['dash','cfg','log'].forEach((n,i)=>{document.querySelectorAll('.tab')[i].classList.toggle('active',n===t);document.getElementById('tab-'+n).classList.toggle('active',n===t);});if(t==='log')loadLog();if(t==='cfg')loadConfig();}
function fetchStatus(){fetch('/data').then(r=>r.json()).then(d=>{document.getElementById('val-state').textContent=d.state;document.getElementById('hdr-ip').textContent=d.ip;setReed('r-ol1',d.ol1);setReed('r-cl1',d.cl1);setReed('r-ol2',d.ol2);setReed('r-cl2',d.cl2);}).catch(()=>{});}
function setReed(id,v){const el=document.getElementById(id);el.className='reed-led '+(v?'on':'');}
function toggleGate(){fetch('/toggle').then(()=>setTimeout(fetchStatus,500));}
var LOCK_LBL='%OPT_LOCK%',LIGHT_LBL='%OPT_LIGHT%';
function updateAuxParam(n,reset){var mode=parseInt(document.getElementById('c-aux'+n+'mode').value);var grp=document.getElementById('aux'+n+'-param-group');var lbl=document.getElementById('aux'+n+'-param-lbl');var inp=document.getElementById('c-aux'+n+'time');if(mode===1){grp.style.display='';lbl.textContent=LOCK_LBL+' Delay (s)';inp.step=1;inp.max=30;if(reset)inp.value=2;}else if(mode===2){grp.style.display='';lbl.textContent=LIGHT_LBL+' Timer (min)';inp.step=1;inp.max=30;if(reset)inp.value=2;}else{grp.style.display='none';}var btn=document.getElementById('btn-aux'+n+'-test');if(btn)btn.style.display=mode>0?'':'none';var sec=document.getElementById('aux-test-section');if(sec){var a1=parseInt(document.getElementById('c-aux1mode').value);var a2=parseInt(document.getElementById('c-aux2mode').value);sec.style.display=(a1>0||a2>0)?'':'none';}}
function testAux(n){fetch('/aux-test?ch='+n).then(r=>r.text()).then(t=>{var msg=document.getElementById('aux-test-msg');msg.textContent='AUX'+n+' test started';setTimeout(()=>msg.textContent='',3000);}).catch(()=>{});}
function loadConfig(){fetch('/config').then(r=>r.json()).then(d=>{document.getElementById('c-delay').value=d.leafDelay;document.getElementById('c-safety').value=d.safetyMs;document.getElementById('c-alertmin').value=d.alertMin;document.getElementById('c-inv').checked=d.seqInv;document.getElementById('c-lang').value=d.lang;document.getElementById('inv-lbl').textContent=d.seqInv?T.inverted:T.normal;document.getElementById('c-aux1mode').value=d.aux1Mode;document.getElementById('c-aux1time').value=d.aux1Time;document.getElementById('c-aux2mode').value=d.aux2Mode;document.getElementById('c-aux2time').value=d.aux2Time;updateAuxParam(1);updateAuxParam(2);document.getElementById('c-alm-breakin').checked=d.almBreakIn;document.getElementById('c-alm-opensrn').checked=d.almOpenSrn;document.getElementById('c-alm-motor').checked=d.almMotor;});}
document.getElementById('c-inv').addEventListener('change',function(){document.getElementById('inv-lbl').textContent=this.checked?T.inverted:T.normal;});
function saveConfig(){var prevLang=document.getElementById('c-lang').value;const p=new URLSearchParams({leafDelay:document.getElementById('c-delay').value,safetyMs:document.getElementById('c-safety').value,alertMin:document.getElementById('c-alertmin').value,seqInv:document.getElementById('c-inv').checked?'1':'0',lang:prevLang,aux1Mode:document.getElementById('c-aux1mode').value,aux1Time:document.getElementById('c-aux1time').value,aux2Mode:document.getElementById('c-aux2mode').value,aux2Time:document.getElementById('c-aux2time').value,almBreakIn:document.getElementById('c-alm-breakin').checked?'1':'0',almOpenSrn:document.getElementById('c-alm-opensrn').checked?'1':'0',almMotor:document.getElementById('c-alm-motor').checked?'1':'0'});fetch('/config-save?'+p.toString()).then(r=>r.text()).then(t=>{document.getElementById('cfg-msg').textContent=t;setTimeout(()=>location.reload(),800);});}
function loadLog(){fetch('/log').then(r=>r.json()).then(entries=>{document.getElementById('log-count').textContent=entries.length+T.entries;const tbody=document.getElementById('log-body');if(!entries.length){tbody.innerHTML='<tr><td colspan="2" class="log-empty">'+T.noEntries+'</td></tr>';return;}tbody.innerHTML=entries.slice().reverse().map(e=>`<tr><td>${e.e}</td><td>${e.t}</td></tr>`).join('');});}
function clearLog(){if(confirm(T.cfmClear))fetch('/log-clear').then(()=>loadLog());}
fetchStatus();setInterval(fetchStatus,5000);
</script></body></html>
)rawliteral";
String buildDash() {
  // Template stored in PROGMEM, copy to RAM then replace placeholders
  String html = String(HTML_DASH_TMPL);
  uint8_t lang = constrain(gConfig.language, 0, 1);
  for (int i=0; i<(int)WEB_STRINGS_COUNT; i++) {
    String key = String("%") + WEB_STRINGS[i].key + "%";
    html.replace(key, WEB_STRINGS[i].val[lang]);
  }
  return html;
}

// ═══════════════════════════════════════════════════════════════
// WEB ROUTES
// ═══════════════════════════════════════════════════════════════

void setupPortalRoutes() {
  server.on("/",[](){server.send_P(200,"text/html",HTML_PORTAL);});
  server.on("/save",[](){
    String ssid=server.arg("ssid"),pass=server.arg("pass");
    if (!ssid.length()){server.send(400,"text/plain","SSID required");return;}
    gPrefs.begin(NVS_NS_WIFI,false);
    gPrefs.putString("ssid",ssid);gPrefs.putString("pass",pass);gPrefs.end();
    server.send(200,"text/plain","Saved! Restarting...");
    delay(1500);ESP.restart();
  });
}

void setupDashRoutes() {
  server.on("/",[](){String html=buildDash();server.send(200,"text/html",html);});
  server.on("/data",[](){
    xSemaphoreTake(gMutex,portMAX_DELAY);
    GateState_t s=gStatus.state;
    bool ol1=gStatus.reed_open_l1,cl1=gStatus.reed_close_l1;
    bool ol2=gStatus.reed_open_l2,cl2=gStatus.reed_close_l2;
    xSemaphoreGive(gMutex);
    char json[128];
    snprintf(json,sizeof(json),
      "{\"state\":\"%s\",\"ip\":\"%s\",\"ol1\":%d,\"cl1\":%d,\"ol2\":%d,\"cl2\":%d}",
      stateToStrLocal(s),currentIP.c_str(),ol1,cl1,ol2,cl2);
    server.send(200,"application/json",json);
  });
  server.on("/toggle",[](){
    GateCmd_t cmd=CMD_TOGGLE;xQueueSend(gCmdQueue,&cmd,0);
    server.send(200,"text/plain","OK");
  });
  server.on("/config",[](){
    char json[256];
    snprintf(json,sizeof(json),
      "{\"leafDelay\":%u,\"safetyMs\":%u,\"alertMin\":%u,\"seqInv\":%s,\"lang\":%u"
      ",\"aux1Mode\":%u,\"aux1Time\":%u,\"aux2Mode\":%u,\"aux2Time\":%u"
      ",\"almBreakIn\":%s,\"almOpenSrn\":%s,\"almMotor\":%s}",
      gConfig.leafDelayMs,gConfig.safetyTimeoutMs,gConfig.alertOpenMin,
      gConfig.sequenceInverted?"true":"false",gConfig.language,
      gConfig.aux1Mode,gConfig.aux1TimeParam,gConfig.aux2Mode,gConfig.aux2TimeParam,
      gConfig.alarmBreakIn?"true":"false",
      gConfig.alarmOpenSiren?"true":"false",
      gConfig.alarmMotorFault?"true":"false");
    server.send(200,"application/json",json);
  });
  server.on("/config-save",[](){
    if (server.hasArg("leafDelay")) gConfig.leafDelayMs     =server.arg("leafDelay").toInt();
    if (server.hasArg("safetyMs"))  gConfig.safetyTimeoutMs =server.arg("safetyMs").toInt();
    if (server.hasArg("alertMin"))  gConfig.alertOpenMin    =server.arg("alertMin").toInt();
    if (server.hasArg("seqInv"))    gConfig.sequenceInverted=server.arg("seqInv")=="1";
    if (server.hasArg("lang"))      gConfig.language        =server.arg("lang").toInt();
    if (server.hasArg("aux1Mode")) gConfig.aux1Mode      =constrain(server.arg("aux1Mode").toInt(),0,3);
    if (server.hasArg("aux1Time")) gConfig.aux1TimeParam  =server.arg("aux1Time").toInt();
    if (server.hasArg("aux2Mode")) gConfig.aux2Mode      =constrain(server.arg("aux2Mode").toInt(),0,3);
    if (server.hasArg("aux2Time")) gConfig.aux2TimeParam  =server.arg("aux2Time").toInt();
    if (server.hasArg("almBreakIn"))  gConfig.alarmBreakIn   =server.arg("almBreakIn")=="1";
    if (server.hasArg("almOpenSrn"))  gConfig.alarmOpenSiren =server.arg("almOpenSrn")=="1";
    if (server.hasArg("almMotor"))    gConfig.alarmMotorFault=server.arg("almMotor")=="1";
    configSave();
    server.send(200,"text/plain", gConfig.language==0?"Configuration saved.":"Configuracao salva.");
  });
  server.on("/log",      [](){server.send(200,"application/json",logToJson());});
  server.on("/log-clear",[](){logClear();server.send(200,"text/plain","OK");});
  server.on("/aux-test",[](){
    if (!server.hasArg("ch")) { server.send(400,"text/plain","missing ch"); return; }
    int ch = server.arg("ch").toInt();
    uint8_t  mode  = (ch==1) ? gConfig.aux1Mode      : gConfig.aux2Mode;
    uint32_t param = (ch==1) ? gConfig.aux1TimeParam  : gConfig.aux2TimeParam;
    uint8_t  pin   = (ch==1) ? PIN_RELAY_AUX1         : PIN_RELAY_AUX2;
    if (mode==AUX_MODE_DISABLED) { server.send(200,"text/plain","disabled"); return; }
    // Pack args into a heap struct for the task
    struct AuxTestArgs { uint8_t pin; uint8_t mode; uint32_t param; };
    AuxTestArgs* args = new AuxTestArgs{pin, mode, param};
    xTaskCreate([](void* p){
      AuxTestArgs* a = (AuxTestArgs*)p;
      uint8_t ch = (a->pin == PIN_RELAY_AUX1) ? 0 : 1;
      if (a->mode==AUX_MODE_LOCK) {
        auxRelaySet(a->pin, true,  "TEST-LOCK release");
        vTaskDelay((a->param>0 ? a->param : 2) * 1000 / portTICK_PERIOD_MS);
        auxRelaySet(a->pin, false, "TEST-LOCK engage");
      } else if (a->mode==AUX_MODE_LIGHT) {
        // Use same auxLightOn path as movement — timer handled by auxTick
        // param=0 uses 5s for test convenience instead of 60min default
        uint32_t testParam = a->param > 0 ? a->param : 0;
        uint32_t mins = testParam > 0 ? testParam : 0;
        auxRelaySet(a->pin, true, "TEST-LIGHT on");
        uint32_t ms = mins > 0 ? mins * 60000UL : 5000UL;
        gAuxLightOn[ch]  = true;
        gAuxLightEnd[ch] = millis() + ms;
      } else if (a->mode==AUX_MODE_SIREN) {
        bool cur = digitalRead(a->pin)==RELAY_ON;
        auxRelaySet(a->pin, !cur, "TEST-SIREN toggle");
      }
      delete a;
      vTaskDelete(nullptr);
    }, "auxTest", 2048, args, 1, nullptr);
    server.send(200,"text/plain","OK");
  });
  server.on("/wifi-reset",[](){
    server.send(200,"text/plain","WiFi cleared. Restarting...");
    delay(1000);
    gPrefs.begin(NVS_NS_WIFI,false);gPrefs.clear();gPrefs.end();
    ESP.restart();
  });
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("===============================");
  Serial.println("  ESP32 Gate Controller");
  Serial.printf ("  Build: %s %s\n",BUILD_DATE,BUILD_TIME);
  Serial.println("===============================");

  pinMode(PIN_RELAY_L1_OPEN, OUTPUT); pinMode(PIN_RELAY_L1_CLOSE,OUTPUT);
  pinMode(PIN_RELAY_L2_OPEN, OUTPUT); pinMode(PIN_RELAY_L2_CLOSE,OUTPUT);
  pinMode(PIN_RELAY_AUX1,    OUTPUT); pinMode(PIN_RELAY_AUX2,    OUTPUT);
  relayAllOff();
  // AUX relays safe state on boot (gate assumed closed):
  // LOCK: engaged = relay OFF (de-energized)
  // LIGHT/SIREN/DISABLED: relay OFF
  digitalWrite(PIN_RELAY_AUX1, RELAY_OFF);
  digitalWrite(PIN_RELAY_AUX2, RELAY_OFF);

  // All reed GPIOs use external pull-up resistors — INPUT only
  // GPIO 34/35: input-only, no internal pull-up available
  // GPIO 32/33: internal pull-up available but using external for consistency
  pinMode(PIN_REED_OPEN_L1, INPUT); pinMode(PIN_REED_CLOSE_L1,INPUT);
  pinMode(PIN_REED_OPEN_L2, INPUT); pinMode(PIN_REED_CLOSE_L2,INPUT);

  pinMode(PIN_BTN_GATE, INPUT_PULLUP);
  pinMode(PIN_BTN_MINUS,INPUT_PULLUP);
  pinMode(PIN_BTN_PLUS, INPUT_PULLUP);
  pinMode(PIN_BTN_OK,   INPUT_PULLUP);

  // Long press OK on boot = WiFi reset
  delay(100);
  if (digitalRead(PIN_BTN_OK)==LOW) {
    unsigned long t0=millis();
    while (digitalRead(PIN_BTN_OK)==LOW) delay(10);
    if (millis()-t0>=LONG_PRESS_MS) {
      Serial.println("[INIT] Long press - clearing WiFi");
      gPrefs.begin(NVS_NS_WIFI,false);gPrefs.clear();gPrefs.end();
      delay(500);ESP.restart();
    }
  }

  configLoad();
  logInit();

  // Display
  dTft.init();
  dTft.setRotation(1);
  dTft.fillScreen(D_BG);
  displayPresent=true;
  Serial.println("[DISPLAY] Initialized");

  // Boot screen
  dClear(); dBorder();
  dTextCentered(30,"ESP32 GATE",D_ACCENT,2);
  char buildStr[32];
  snprintf(buildStr,sizeof(buildStr),"%s",BUILD_DATE);
  dTextCentered(62,buildStr,D_YELLOW);
  dTextCentered(76,BUILD_TIME,D_YELLOW);

  gMutex   =xSemaphoreCreateMutex();
  gCmdQueue=xQueueCreate(5, sizeof(GateCmd_t));
  gBtnQueue=xQueueCreate(10,sizeof(BtnEvent_t));

  // WiFi
  gPrefs.begin(NVS_NS_WIFI,true);
  String ssid=gPrefs.getString("ssid","");
  String pass=gPrefs.getString("pass","");
  gPrefs.end();

  bool hasCredentials=ssid.length()>0;
  bool forcePortal   =digitalRead(PIN_BTN_GATE)==LOW;

  if (!forcePortal&&hasCredentials) {
    dTft.fillRect(0,88,D_W,20,D_BG);
    dTextCentered(95,tr("Connecting...","Conectando..."),D_WHITE);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(),pass.c_str());
    unsigned long start=millis();
    while (WiFi.status()!=WL_CONNECTED&&millis()-start<CONNECT_TIMEOUT_MS) delay(250);
    if (WiFi.status()==WL_CONNECTED) {
      wifiConnected=true;
      currentIP=WiFi.localIP().toString();
      Serial.printf("[WIFI] Connected - %s\n",currentIP.c_str());
      configTime(GMT_OFFSET,DST_OFFSET,NTP_SERVER);
      struct tm t;
      if (getLocalTime(&t)){ntpSynced=true;Serial.println("[NTP] Synced");}
      setupDashRoutes(); server.begin();
    } else {
      Serial.println("[WIFI] Failed - portal");
      portalMode=true;
      WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID);
      currentIP=WiFi.softAPIP().toString();
      setupPortalRoutes();server.begin();
    }
  } else {
    portalMode=true;
    WiFi.mode(WIFI_AP);WiFi.softAP(AP_SSID);
    currentIP=WiFi.softAPIP().toString();
    Serial.printf("[WIFI] AP - %s\n",currentIP.c_str());
    setupPortalRoutes();server.begin();
  }

  delay(1500);

  // ── Boot state: determine initial gate state from reed switches ──
  {
    bool cl1 = (digitalRead(PIN_REED_CLOSE_L1)==LOW);
    bool cl2 = (digitalRead(PIN_REED_CLOSE_L2)==LOW);
    bool ol1 = (digitalRead(PIN_REED_OPEN_L1) ==LOW);
    bool ol2 = (digitalRead(PIN_REED_OPEN_L2) ==LOW);
    GateState_t bootState;
    if      (cl1 && cl2)  bootState = GATE_CLOSED;
    else if (ol1 && ol2)  bootState = GATE_OPEN;
    else                  bootState = GATE_STOPPED;
    gStatus.state          = bootState;
    gStatus.reed_close_l1  = cl1;
    gStatus.reed_close_l2  = cl2;
    gStatus.reed_open_l1   = ol1;
    gStatus.reed_open_l2   = ol2;
    gStatus.lastStateChange= millis();
    Serial.printf("[INIT] Boot state: %s (CL1:%d CL2:%d OL1:%d OL2:%d)\n",
      stateToStr(bootState), cl1, cl2, ol1, ol2);
  }

  xTaskCreatePinnedToCore(taskGateController,"gate",   4096,nullptr,2,&hTaskGate, 1);
  xTaskCreatePinnedToCore(taskInputHandler,  "input",  2048,nullptr,2,&hTaskInput,1);
  xTaskCreatePinnedToCore(taskMonitor,       "monitor",2048,nullptr,1,&hTaskMon,  0);
  xTaskCreatePinnedToCore(taskWeb,           "web",    4096,nullptr,1,&hTaskWeb,  0);
  xTaskCreatePinnedToCore(taskDisplay,       "display",4096,nullptr,1,&hTaskDisp, 1);

  Serial.printf("[INIT] Ready - http://%s\n",currentIP.c_str());
}

void loop() { vTaskDelay(portMAX_DELAY); }
