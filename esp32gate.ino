// ═══════════════════════════════════════════════════════════════
// ESP32 Gate Controller
// Stages 1-3+5+menu — bugfix rev1
// Fixes: menu exit, log back/clear, idle redraw, arrows,
//        IP flicker, PT accents removed
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
#define LOG_MAX_ENTRIES 50

typedef enum {
  LOG_EVT_OPENED = 0, LOG_EVT_CLOSED, LOG_EVT_STOPPED,
  LOG_EVT_TIMEOUT, LOG_EVT_OPEN_TOO_LONG, LOG_EVT_ERROR
} LogEvent_t;

struct LogEntry { uint8_t event; char timestamp[15]; };

// ── Configuration ────────────────────────────────────────────
struct GateConfig {
  uint32_t leafDelayMs;
  uint32_t safetyTimeoutMs;
  uint32_t alertOpenMin;
  bool     sequenceInverted;
  uint8_t  language;
};

static GateConfig gConfig = {
  .leafDelayMs      = 3000,
  .safetyTimeoutMs  = 20000,
  .alertOpenMin     = 10,
  .sequenceInverted = false,
  .language         = 0
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
#define D_BG     TFT_BLACK
#define D_ACCENT 0x07FF
#define D_WHITE  TFT_WHITE
#define D_GREEN  0x07E0
#define D_RED    0xF800
#define D_YELLOW 0xFFE0
#define D_DIM    0x2945
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

static uint16_t logHead=0, logTail=0, logCount=0;
static int      arrowFrame=0;

static MenuState_t   menuState     = MENU_IDLE;
static int           menuCursor    = 0;
static int           menuEditField = 0;
static unsigned long menuLastInput = 0;
static int           logScrollPos  = 0;
static bool          idleNeedsRedraw = false; // FIX: flag to force idle redraw

static TaskHandle_t hTaskGate=nullptr, hTaskInput=nullptr;
static TaskHandle_t hTaskMon=nullptr,  hTaskWeb=nullptr, hTaskDisp=nullptr;

typedef enum { BTN_GATE=0, BTN_PLUS, BTN_MINUS, BTN_OK, BTN_OK_LONG } BtnEvent_t;

// ═══════════════════════════════════════════════════════════════
// HELPERS
// ═══════════════════════════════════════════════════════════════

const char* stateToStr(GateState_t s) {
  switch(s) {
    case GATE_CLOSED:  return "CLOSED";
    case GATE_OPENING: return "OPENING";
    case GATE_OPEN:    return "OPEN";
    case GATE_CLOSING: return "CLOSING";
    case GATE_STOPPED: return "STOPPED";
    case GATE_ERROR:   return "ERROR";
    default:           return "UNKNOWN";
  }
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

// FIX: no accented characters in PT strings
const char* s(const char* en, const char* pt) {
  return gConfig.language==0 ? en : pt;
}

// ═══════════════════════════════════════════════════════════════
// NVS — CONFIG
// ═══════════════════════════════════════════════════════════════

void configLoad() {
  gPrefs.begin(NVS_NS_CONFIG,true);
  gConfig.leafDelayMs      = gPrefs.getUInt ("leafDelay",gConfig.leafDelayMs);
  gConfig.safetyTimeoutMs  = gPrefs.getUInt ("safetyMs", gConfig.safetyTimeoutMs);
  gConfig.alertOpenMin     = gPrefs.getUInt ("alertMin", gConfig.alertOpenMin);
  gConfig.sequenceInverted = gPrefs.getBool ("seqInv",   gConfig.sequenceInverted);
  gConfig.language         = gPrefs.getUChar("lang",     gConfig.language);
  gPrefs.end();
  Serial.println("[CFG] Loaded");
}

void configSave() {
  gPrefs.begin(NVS_NS_CONFIG,false);
  gPrefs.putUInt ("leafDelay",gConfig.leafDelayMs);
  gPrefs.putUInt ("safetyMs", gConfig.safetyTimeoutMs);
  gPrefs.putUInt ("alertMin", gConfig.alertOpenMin);
  gPrefs.putBool ("seqInv",   gConfig.sequenceInverted);
  gPrefs.putUChar("lang",     gConfig.language);
  gPrefs.end();
  Serial.println("[CFG] Saved");
}

// ═══════════════════════════════════════════════════════════════
// NVS — LOG
// ═══════════════════════════════════════════════════════════════

const char* logEventToStr(uint8_t evt) {
  switch(evt) {
    case LOG_EVT_OPENED:        return "OPENED";
    case LOG_EVT_CLOSED:        return "CLOSED";
    case LOG_EVT_STOPPED:       return "STOPPED";
    case LOG_EVT_TIMEOUT:       return "TIMEOUT";
    case LOG_EVT_OPEN_TOO_LONG: return "OPEN>LIMIT";
    case LOG_EVT_ERROR:         return "ERROR";
    default:                    return "UNKNOWN";
  }
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
    json+="{\"e\":\""; json+=logEventToStr(entry.event);
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
// MOVEMENT SEQUENCES
// ═══════════════════════════════════════════════════════════════

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
  LeafPins lp=resolveLeafPins();
  unsigned long tStart=millis();
  bool secondStarted=false,firstDone=false,secondDone=false;
  relaySet(lp.firstClose,false,"FIRST_CLOSE");
  relaySet(lp.secondClose,false,"SECOND_CLOSE");
  vTaskDelay(10/portTICK_PERIOD_MS);
  relaySet(lp.firstOpen,true,"FIRST_OPEN");
  while (!(firstDone&&secondDone)) {
    GateCmd_t cmd;
    if (xQueuePeek(gCmdQueue,&cmd,0)==pdTRUE) {
      xQueueReceive(gCmdQueue,&cmd,0);
      relayAllOff(); setState(GATE_STOPPED); logWrite(LOG_EVT_STOPPED); return false;
    }
    unsigned long el=millis()-tStart;
    if (el>=gConfig.safetyTimeoutMs) {
      relayAllOff(); setState(GATE_ERROR); logWrite(LOG_EVT_TIMEOUT); return false;
    }
    if (!secondStarted&&el>=gConfig.leafDelayMs) {
      relaySet(lp.secondClose,false,"SECOND_CLOSE");
      vTaskDelay(10/portTICK_PERIOD_MS);
      relaySet(lp.secondOpen,true,"SECOND_OPEN");
      secondStarted=true;
    }
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
  relayAllOff(); return true;
}

bool sequenceClose() {
  LeafPins lp=resolveLeafPins();
  unsigned long tStart=millis();
  bool firstStarted=false,secondDone=false,firstDone=false;
  relaySet(lp.firstOpen,false,"FIRST_OPEN");
  relaySet(lp.secondOpen,false,"SECOND_OPEN");
  vTaskDelay(10/portTICK_PERIOD_MS);
  relaySet(lp.secondClose,true,"SECOND_CLOSE");
  while (!(firstDone&&secondDone)) {
    GateCmd_t cmd;
    if (xQueuePeek(gCmdQueue,&cmd,0)==pdTRUE) {
      xQueueReceive(gCmdQueue,&cmd,0);
      relayAllOff(); setState(GATE_STOPPED); logWrite(LOG_EVT_STOPPED); return false;
    }
    unsigned long el=millis()-tStart;
    if (el>=gConfig.safetyTimeoutMs) {
      relayAllOff(); setState(GATE_ERROR); logWrite(LOG_EVT_TIMEOUT); return false;
    }
    if (!firstStarted&&el>=gConfig.leafDelayMs) {
      relaySet(lp.firstOpen,false,"FIRST_OPEN");
      vTaskDelay(10/portTICK_PERIOD_MS);
      relaySet(lp.firstClose,true,"FIRST_CLOSE");
      firstStarted=true;
    }
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
  relayAllOff(); return true;
}

// ═══════════════════════════════════════════════════════════════
// DISPLAY — primitives
// ═══════════════════════════════════════════════════════════════

uint16_t stateColor(GateState_t s) {
  switch(s) {
    case GATE_OPEN:    return D_ACCENT;
    case GATE_OPENING: return D_GREEN;
    case GATE_CLOSING: return D_RED;
    case GATE_CLOSED:  return D_WHITE;
    case GATE_STOPPED: return D_YELLOW;
    case GATE_ERROR:   return D_YELLOW;
    default:           return D_WHITE;
  }
}

void dClear() { dTft.fillScreen(D_BG); }

void dBorder() {
  dTft.drawFastHLine(0,0,    D_W,D_ACCENT); dTft.drawFastHLine(0,1,    D_W,D_DIM);
  dTft.drawFastHLine(0,D_H-2,D_W,D_DIM);   dTft.drawFastHLine(0,D_H-1,D_W,D_ACCENT);
  dTft.drawFastVLine(0,    0,D_H,D_ACCENT); dTft.drawFastVLine(1,    0,D_H,D_DIM);
  dTft.drawFastVLine(D_W-2,0,D_H,D_DIM);   dTft.drawFastVLine(D_W-1,0,D_H,D_ACCENT);
}

void dText(int x, int y, const char* txt, uint16_t color, uint8_t size=1) {
  dTft.setTextColor(color); dTft.setTextSize(size);
  dTft.setCursor(x,y); dTft.print(txt);
}

void dTextCentered(int y, const char* txt, uint16_t color, uint8_t size=1) {
  int x=max(2,(int)((D_W-strlen(txt)*6*size)/2));
  dText(x,y,txt,color,size);
}

void dHeader(const char* title) {
  dBorder();
  dText(6,5,title,D_WHITE);
  dTft.drawFastHLine(4,16,D_W-8,D_DIM);
}

// ═══════════════════════════════════════════════════════════════
// DISPLAY — idle screen
// ═══════════════════════════════════════════════════════════════

void displayDrawIdleFrame() {
  dClear(); dBorder();
  dText(6,5,"ESP32 GATE",D_WHITE);
  dTft.drawFastHLine(4,16, D_W-8,D_DIM);
  dTft.drawFastHLine(4,100,D_W-8,D_DIM);
  dTft.drawFastHLine(4,116,D_W-8,D_DIM);
}

void displayDrawTime() {
  struct tm t;
  if (!getLocalTime(&t)) return;
  char buf[9];
  sprintf(buf,"%02d:%02d:%02d",t.tm_hour,t.tm_min,t.tm_sec);
  dTft.fillRect(90,3,66,10,D_BG);
  dTft.setTextColor(D_ACCENT); dTft.setTextSize(1);
  dTft.setCursor(92,5); dTft.print(buf);
}

void displayDrawIP() {
  // FIX: only called when IP changes, no flicker
  dTft.fillRect(4,118,D_W-8,8,D_BG);
  dTft.setTextColor(D_WHITE); dTft.setTextSize(1);
  int x=max(2,(int)((D_W-(int)currentIP.length()*6)/2));
  dTft.setCursor(x,119); dTft.print(currentIP.c_str());
}

// FIX: full idle redraw — resets lastS to force state repaint
static GateState_t idleLastS   = GATE_ERROR;
static bool idleLastOL1=false,idleLastCL1=false;
static bool idleLastOL2=false,idleLastCL2=false;
static String idleLastIP = "";

void displayIdleReset() {
  // Call after returning from menu to force full repaint
  idleLastS   = GATE_ERROR;
  idleLastOL1 = !idleLastOL1; // force reed redraw
  idleLastIP  = "";            // force IP redraw
}

void displayDrawIdle(GateState_t s, bool ol1, bool cl1, bool ol2, bool cl2) {
  // State
  if (s!=idleLastS) {
    dTft.fillRect(4,18,D_W-8,78,D_BG);
    uint16_t color=stateColor(s);
    const char* label=stateToStr(s);
    int x=max(4,(int)((D_W-(int)strlen(label)*12)/2));
    dTft.setTextColor(color); dTft.setTextSize(2);
    dTft.setCursor(x,28); dTft.print(label);
    idleLastS=s; arrowFrame=0;
    // clear arrow area when not moving
    if (s!=GATE_OPENING&&s!=GATE_CLOSING)
      dTft.fillRect(4,48,D_W-8,30,D_BG);
  }

  // FIX: arrows — closing goes LEFT (<), opening goes RIGHT (>)
  if (s==GATE_OPENING||s==GATE_CLOSING) {
    bool opening=(s==GATE_OPENING);
    uint16_t color=opening?D_GREEN:D_RED;
    // FIX: closing uses '<'
    const char* arrow=opening?">":"<";
    int positions[]={20,44,68,92,116};
    dTft.setTextSize(3);
    for (int i=0;i<5;i++) {
      // FIX: closing animation travels right-to-left
      // opening: lit arrows move right (frame advances left index)
      // closing: lit arrows move left (frame advances right index)
      bool lit;
      if (opening) lit=((i-arrowFrame+5)%5)<2;
      else         lit=((arrowFrame-i+5)%5)<2;
      dTft.setTextColor(lit?color:D_DIM);
      dTft.setCursor(positions[i],52); dTft.print(arrow);
    }
  }

  // Reed row
  if (ol1!=idleLastOL1||cl1!=idleLastCL1||ol2!=idleLastOL2||cl2!=idleLastCL2) {
    dTft.fillRect(4,103,D_W-8,12,D_BG);
    dTft.setTextSize(1);
    dTft.setTextColor(D_WHITE); dTft.setCursor(6,105);  dTft.print("L1 O:");
    dTft.setTextColor(ol1?D_GREEN:D_DIM); dTft.setCursor(38,105); dTft.print(ol1?"Y":"N");
    dTft.setTextColor(D_WHITE); dTft.setCursor(48,105); dTft.print("C:");
    dTft.setTextColor(cl1?D_GREEN:D_DIM); dTft.setCursor(62,105); dTft.print(cl1?"Y":"N");
    dTft.setTextColor(D_WHITE); dTft.setCursor(80,105); dTft.print("L2 O:");
    dTft.setTextColor(ol2?D_GREEN:D_DIM); dTft.setCursor(112,105); dTft.print(ol2?"Y":"N");
    dTft.setTextColor(D_WHITE); dTft.setCursor(122,105); dTft.print("C:");
    dTft.setTextColor(cl2?D_GREEN:D_DIM); dTft.setCursor(136,105); dTft.print(cl2?"Y":"N");
    idleLastOL1=ol1; idleLastCL1=cl1; idleLastOL2=ol2; idleLastCL2=cl2;
  }

  // FIX: IP — only redraw when changed
  if (currentIP!=idleLastIP) {
    displayDrawIP();
    idleLastIP=currentIP;
  }
}

// ═══════════════════════════════════════════════════════════════
// DISPLAY — menu screens
// ═══════════════════════════════════════════════════════════════

#define MENU_Y0   20
#define MENU_LINE 16

void menuDrawList(const char* title, const char** items, int count, int cursor) {
  dClear(); dHeader(title);
  for (int i=0;i<count;i++) {
    int y=MENU_Y0+i*MENU_LINE;
    if (i==cursor) {
      dTft.fillRect(4,y-1,D_W-8,MENU_LINE-1,D_ACCENT);
      dText(10,y,items[i],D_BG);
    } else {
      dText(10,y,items[i],D_WHITE);
    }
  }
  dText(4,D_H-10,"+/- nav  OK select",D_DIM);
}

// FIX: added "Exit"/"Sair" as 5th item
void menuDrawRoot() {
  const char* items[]={
    s("Gate Control","Controle"),
    s("Configuration","Configuracao"),
    s("Event Log","Registro"),
    "WiFi",
    s("Exit","Sair")
  };
  menuDrawList("MENU",items,5,menuCursor);
}

void menuDrawGateCtrl() {
  const char* items[]={s("Toggle Gate","Acionar"),s("Back","Voltar")};
  menuDrawList(s("GATE CTRL","PORTAO"),items,2,menuCursor);
}

void menuDrawConfig() {
  dClear(); dHeader(s("CONFIGURATION","CONFIG"));
  struct { const char* label; char val[20]; } fields[5];
  snprintf(fields[0].val,20,"%ums",  gConfig.leafDelayMs);
  snprintf(fields[1].val,20,"%ums",  gConfig.safetyTimeoutMs);
  snprintf(fields[2].val,20,"%umin", gConfig.alertOpenMin);
  snprintf(fields[3].val,20,"%s",    gConfig.sequenceInverted?"INV":"NRM");
  snprintf(fields[4].val,20,"%s",    gConfig.language==0?"EN":"PT");
  fields[0].label=s("Leaf Delay", "Delay Folha");
  fields[1].label=s("Safety T/O", "Timeout Seg");
  fields[2].label=s("Alert Open", "Alerta Abert");
  fields[3].label=s("Sequence",   "Sequencia");
  fields[4].label=s("Language",   "Idioma");
  for (int i=0;i<5;i++) {
    int y=MENU_Y0+i*MENU_LINE;
    bool sel=(i==menuCursor);
    if (sel) dTft.fillRect(4,y-1,D_W-8,MENU_LINE-1,D_ACCENT);
    dText(8,y,fields[i].label,sel?D_BG:D_WHITE);
    dText(D_W-(int)strlen(fields[i].val)*6-8,y,fields[i].val,sel?D_BG:D_ACCENT);
  }
  int y=MENU_Y0+5*MENU_LINE;
  bool sel=(menuCursor==5);
  if (sel) dTft.fillRect(4,y-1,D_W-8,MENU_LINE-1,D_ACCENT);
  dText(8,y,s("Back","Voltar"),sel?D_BG:D_WHITE);
  dText(4,D_H-10,"+/- select  OK edit",D_DIM);
}

void menuDrawConfigEdit() {
  dClear(); dHeader(s("EDIT","EDITAR"));
  char title[32],val[20];
  switch(menuEditField) {
    case 0: strcpy(title,s("Leaf Delay (ms)","Delay Folha (ms)"));
            snprintf(val,20,"%u",gConfig.leafDelayMs); break;
    case 1: strcpy(title,s("Safety T/O (ms)","Timeout Seg (ms)"));
            snprintf(val,20,"%u",gConfig.safetyTimeoutMs); break;
    case 2: strcpy(title,s("Alert Open (min)","Alerta Aberto"));
            snprintf(val,20,"%u",gConfig.alertOpenMin); break;
    case 3: strcpy(title,s("Sequence","Sequencia"));
            snprintf(val,20,"%s",gConfig.sequenceInverted?"INVERTED":"NORMAL"); break;
    case 4: strcpy(title,s("Language","Idioma"));
            snprintf(val,20,"%s",gConfig.language==0?"EN":"PT"); break;
  }
  dTextCentered(30,title,D_WHITE);
  dTft.fillRect(20,50,D_W-40,28,D_DIM);
  dTft.setTextColor(D_ACCENT); dTft.setTextSize(2);
  int x=max(22,(int)((D_W-(int)strlen(val)*12)/2));
  dTft.setCursor(x,55); dTft.print(val);
  dText(4,D_H-10,s("+/- change  OK save","+/- muda  OK salva"),D_DIM);
}

// FIX: log menu now has navigable items: entries list + "Clear" + "Back"
void menuDrawLog() {
  dClear(); dHeader(s("EVENT LOG","REGISTRO"));

  int visible=4; // rows for log entries
  if (logCount==0) {
    dTextCentered(40,s("No entries","Sem registros"),D_DIM);
  } else {
    for (int i=0;i<visible&&(logScrollPos+i)<(int)logCount;i++) {
      int idx=logCount-1-(logScrollPos+i);
      LogEntry e=logGet(idx);
      int y=MENU_Y0+i*MENU_LINE;
      char line[16]; snprintf(line,sizeof(line),"%-10s",logEventToStr(e.event));
      dText(6,y,line,D_WHITE);
      if (strncmp(e.timestamp,"00000000000000",14)!=0) {
        char hhmm[6]; snprintf(hhmm,6,"%.2s:%.2s",e.timestamp+8,e.timestamp+10);
        dText(D_W-40,y,hhmm,D_ACCENT);
      }
    }
  }

  // FIX: two action items below entries
  int y0=MENU_Y0+visible*MENU_LINE+2;
  dTft.drawFastHLine(4,y0-2,D_W-8,D_DIM);

  bool selClear=(menuCursor==0);
  bool selBack =(menuCursor==1);

  if (selClear) dTft.fillRect(4,y0-1,  D_W-8,MENU_LINE-1,D_RED);
  dText(10,y0,  s("Clear Log","Limpar Log"), selClear?D_BG:D_RED);

  if (selBack)  dTft.fillRect(4,y0+MENU_LINE-1,D_W-8,MENU_LINE-1,D_ACCENT);
  dText(10,y0+MENU_LINE,s("Back","Voltar"), selBack?D_BG:D_WHITE);

  char footer[20];
  snprintf(footer,sizeof(footer),"%u entries  +/- scroll",(unsigned)logCount);
  dText(4,D_H-10,footer,D_DIM);
}

void menuDrawWifi() {
  const char* items[]={s("Reset WiFi","Resetar WiFi"),s("Back","Voltar")};
  menuDrawList("WIFI",items,2,menuCursor);
}

// ═══════════════════════════════════════════════════════════════
// MENU LOGIC
// ═══════════════════════════════════════════════════════════════

void menuReturnToIdle() {
  menuState=MENU_IDLE;
  displayIdleReset();       // FIX: force full idle repaint
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
          case 2: menuState=MENU_LOG; logScrollPos=0; menuCursor=1; menuDrawLog(); break;
          case 3: menuState=MENU_WIFI;      menuCursor=0; menuDrawWifi();     break;
          case 4: menuReturnToIdle(); break; // FIX: Exit item
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
      if (btn==BTN_PLUS)  { menuCursor=(menuCursor+1)%6; menuDrawConfig(); }
      if (btn==BTN_MINUS) { menuCursor=(menuCursor+5)%6; menuDrawConfig(); }
      if (btn==BTN_OK) {
        if (menuCursor==5) { menuState=MENU_ROOT; menuCursor=0; menuDrawRoot(); }
        else { menuEditField=menuCursor; menuState=MENU_CONFIG_EDIT; menuDrawConfigEdit(); }
      }
      break;

    case MENU_CONFIG_EDIT:
      if (btn==BTN_PLUS||btn==BTN_MINUS) {
        int dir=(btn==BTN_PLUS)?1:-1;
        switch(menuEditField) {
          case 0: gConfig.leafDelayMs     =constrain((int)gConfig.leafDelayMs    +dir*100,0,    30000); break;
          case 1: gConfig.safetyTimeoutMs =constrain((int)gConfig.safetyTimeoutMs+dir*500,1000, 60000); break;
          case 2: gConfig.alertOpenMin    =constrain((int)gConfig.alertOpenMin   +dir,    1,    120);   break;
          case 3: gConfig.sequenceInverted=!gConfig.sequenceInverted; break;
          case 4: gConfig.language        =gConfig.language==0?1:0; break;
        }
        menuDrawConfigEdit();
      }
      if (btn==BTN_OK) { configSave(); menuState=MENU_CONFIG; menuDrawConfig(); }
      break;

    // FIX: log menu — cursor 0=Clear, 1=Back; +/- scrolls entries
    case MENU_LOG:
      if (btn==BTN_PLUS) {
        if (logScrollPos+4<(int)logCount) { logScrollPos++; menuDrawLog(); }
      }
      if (btn==BTN_MINUS) {
        if (logScrollPos>0) { logScrollPos--; menuDrawLog(); }
      }
      if (btn==BTN_OK) {
        if (menuCursor==0) {
          // Clear selected
          logClear(); logScrollPos=0; menuDrawLog();
        } else {
          // Back selected
          menuState=MENU_ROOT; menuCursor=0; menuDrawRoot();
        }
      }
      // Toggle between Clear and Back with gate button
      if (btn==BTN_GATE) { menuCursor^=1; menuDrawLog(); }
      break;

    case MENU_WIFI:
      if (btn==BTN_PLUS||btn==BTN_MINUS) { menuCursor^=1; menuDrawWifi(); }
      if (btn==BTN_OK) {
        if (menuCursor==0) {
          gPrefs.begin(NVS_NS_WIFI,false); gPrefs.clear(); gPrefs.end();
          dClear();
          dTextCentered(50,s("WiFi cleared","WiFi apagado"),D_YELLOW);
          dTextCentered(65,s("Restarting...","Reiniciando..."),D_WHITE);
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

    if (plus==LOW&&lastPlus==HIGH) {
      vTaskDelay(50/portTICK_PERIOD_MS);
      if (digitalRead(PIN_BTN_PLUS)==LOW) {
        BtnEvent_t b=BTN_PLUS; xQueueSend(gBtnQueue,&b,0);
        Serial.println("[INPUT] + pressed");
      }
    }
    lastPlus=plus;

    if (minus==LOW&&lastMinus==HIGH) {
      vTaskDelay(50/portTICK_PERIOD_MS);
      if (digitalRead(PIN_BTN_MINUS)==LOW) {
        BtnEvent_t b=BTN_MINUS; xQueueSend(gBtnQueue,&b,0);
        Serial.println("[INPUT] - pressed");
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
  unsigned long lastPrint=0,lastAlert=0;
  for (;;) {
    unsigned long now=millis();
    xSemaphoreTake(gMutex,portMAX_DELAY);
    GateState_t state=gStatus.state;
    unsigned long elapsed=now-gStatus.lastStateChange;
    xSemaphoreGive(gMutex);
    if (state==GATE_OPEN&&elapsed>=gConfig.alertOpenMin*60000UL&&now-lastAlert>=60000UL) {
      lastAlert=now;
      Serial.printf("[ALERT] Open for %lu min\n",elapsed/60000UL);
      logWrite(LOG_EVT_OPEN_TOO_LONG);
    }
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
    vTaskDelay(500/portTICK_PERIOD_MS);
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
        lastArrow=now; arrowFrame=(arrowFrame+1)%5;
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

const char HTML_DASH[] PROGMEM = R"rawliteral(
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
.reed-label{font-size:.6rem;color:#888;text-transform:uppercase;margin-bottom:.25rem}
.reed-val{font-size:.85rem;font-weight:bold}
.reed-val.on{color:#4fc3f7}.reed-val.off{color:#333}
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
  <div class="tab active" onclick="showTab('dash')">Status</div>
  <div class="tab" onclick="showTab('cfg')">Config</div>
  <div class="tab" onclick="showTab('log')">Log</div>
</div>
<div class="panel active" id="tab-dash">
  <div class="status-box"><div class="state-label">Gate State</div><div class="state-value" id="val-state">-</div></div>
  <div class="reeds">
    <div class="reed-box"><div class="reed-label">L1 Open</div><div class="reed-val" id="r-ol1">-</div></div>
    <div class="reed-box"><div class="reed-label">L1 Close</div><div class="reed-val" id="r-cl1">-</div></div>
    <div class="reed-box"><div class="reed-label">L2 Open</div><div class="reed-val" id="r-ol2">-</div></div>
    <div class="reed-box"><div class="reed-label">L2 Close</div><div class="reed-val" id="r-cl2">-</div></div>
  </div>
  <button class="btn-toggle" onclick="toggleGate()">Toggle Gate</button>
  <button class="btn-reset" onclick="if(confirm('Reset WiFi?'))location='/wifi-reset'">Reset WiFi</button>
</div>
<div class="panel" id="tab-cfg">
  <div class="form-group"><label class="form-label">Leaf Delay (ms)</label><div class="form-desc">Delay before activating second leaf</div><input type="number" id="c-delay" min="0" max="30000" step="100"></div>
  <div class="form-group"><label class="form-label">Safety Timeout (ms)</label><div class="form-desc">Max movement time before force-stop</div><input type="number" id="c-safety" min="1000" max="60000" step="500"></div>
  <div class="form-group"><label class="form-label">Alert if Open (min)</label><div class="form-desc">Alert if gate open longer than this</div><input type="number" id="c-alert" min="1" max="120"></div>
  <div class="form-group"><label class="form-label">Sequence Inverted</label><div class="form-desc">L2 leads on open, L1 leads on close</div><div class="toggle-wrap"><label class="toggle"><input type="checkbox" id="c-inv"><span class="slider"></span></label><span style="font-size:.75rem;color:#888" id="inv-lbl">Normal</span></div></div>
  <div class="form-group"><label class="form-label">Language</label><select id="c-lang"><option value="0">English</option><option value="1">Portugues</option></select></div>
  <button class="btn-save" onclick="saveConfig()">Save Configuration</button>
  <button class="btn-wifi" onclick="if(confirm('Reset WiFi?'))location='/wifi-reset'">Reset WiFi Credentials</button>
  <div class="msg" id="cfg-msg"></div>
</div>
<div class="panel" id="tab-log">
  <div class="log-header"><span class="log-count" id="log-count">0 entries</span><button class="btn-clear" onclick="clearLog()">Clear Log</button></div>
  <table class="log-table"><thead><tr><th>Event</th><th>Timestamp</th></tr></thead><tbody id="log-body"><tr><td colspan="2" class="log-empty">Loading...</td></tr></tbody></table>
</div>
<script>
function showTab(t){['dash','cfg','log'].forEach((n,i)=>{document.querySelectorAll('.tab')[i].classList.toggle('active',n===t);document.getElementById('tab-'+n).classList.toggle('active',n===t);});if(t==='log')loadLog();if(t==='cfg')loadConfig();}
function fetchStatus(){fetch('/data').then(r=>r.json()).then(d=>{document.getElementById('val-state').textContent=d.state;document.getElementById('hdr-ip').textContent=d.ip;setReed('r-ol1',d.ol1);setReed('r-cl1',d.cl1);setReed('r-ol2',d.ol2);setReed('r-cl2',d.cl2);}).catch(()=>{});}
function setReed(id,v){const el=document.getElementById(id);el.textContent=v?'ON':'--';el.className='reed-val '+(v?'on':'off');}
function toggleGate(){fetch('/toggle').then(()=>setTimeout(fetchStatus,500));}
function loadConfig(){fetch('/config').then(r=>r.json()).then(d=>{document.getElementById('c-delay').value=d.leafDelay;document.getElementById('c-safety').value=d.safetyMs;document.getElementById('c-alert').value=d.alertMin;document.getElementById('c-inv').checked=d.seqInv;document.getElementById('c-lang').value=d.lang;document.getElementById('inv-lbl').textContent=d.seqInv?'Inverted':'Normal';});}
document.getElementById('c-inv').addEventListener('change',function(){document.getElementById('inv-lbl').textContent=this.checked?'Inverted':'Normal';});
function saveConfig(){const p=new URLSearchParams({leafDelay:document.getElementById('c-delay').value,safetyMs:document.getElementById('c-safety').value,alertMin:document.getElementById('c-alert').value,seqInv:document.getElementById('c-inv').checked?'1':'0',lang:document.getElementById('c-lang').value});fetch('/config-save?'+p.toString()).then(r=>r.text()).then(t=>{document.getElementById('cfg-msg').textContent=t;setTimeout(()=>document.getElementById('cfg-msg').textContent='',3000);});}
function loadLog(){fetch('/log').then(r=>r.json()).then(entries=>{document.getElementById('log-count').textContent=entries.length+' entries';const tbody=document.getElementById('log-body');if(!entries.length){tbody.innerHTML='<tr><td colspan="2" class="log-empty">No entries</td></tr>';return;}tbody.innerHTML=entries.slice().reverse().map(e=>`<tr><td>${e.e}</td><td>${e.t}</td></tr>`).join('');});}
function clearLog(){if(confirm('Clear all log entries?'))fetch('/log-clear').then(()=>loadLog());}
fetchStatus();setInterval(fetchStatus,5000);
</script></body></html>
)rawliteral";

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
  server.on("/",[](){server.send_P(200,"text/html",HTML_DASH);});
  server.on("/data",[](){
    xSemaphoreTake(gMutex,portMAX_DELAY);
    GateState_t s=gStatus.state;
    bool ol1=gStatus.reed_open_l1,cl1=gStatus.reed_close_l1;
    bool ol2=gStatus.reed_open_l2,cl2=gStatus.reed_close_l2;
    xSemaphoreGive(gMutex);
    char json[128];
    snprintf(json,sizeof(json),
      "{\"state\":\"%s\",\"ip\":\"%s\",\"ol1\":%d,\"cl1\":%d,\"ol2\":%d,\"cl2\":%d}",
      stateToStr(s),currentIP.c_str(),ol1,cl1,ol2,cl2);
    server.send(200,"application/json",json);
  });
  server.on("/toggle",[](){
    GateCmd_t cmd=CMD_TOGGLE;xQueueSend(gCmdQueue,&cmd,0);
    server.send(200,"text/plain","OK");
  });
  server.on("/config",[](){
    char json[128];
    snprintf(json,sizeof(json),
      "{\"leafDelay\":%u,\"safetyMs\":%u,\"alertMin\":%u,\"seqInv\":%s,\"lang\":%u}",
      gConfig.leafDelayMs,gConfig.safetyTimeoutMs,gConfig.alertOpenMin,
      gConfig.sequenceInverted?"true":"false",gConfig.language);
    server.send(200,"application/json",json);
  });
  server.on("/config-save",[](){
    if (server.hasArg("leafDelay")) gConfig.leafDelayMs     =server.arg("leafDelay").toInt();
    if (server.hasArg("safetyMs"))  gConfig.safetyTimeoutMs =server.arg("safetyMs").toInt();
    if (server.hasArg("alertMin"))  gConfig.alertOpenMin    =server.arg("alertMin").toInt();
    if (server.hasArg("seqInv"))    gConfig.sequenceInverted=server.arg("seqInv")=="1";
    if (server.hasArg("lang"))      gConfig.language        =server.arg("lang").toInt();
    configSave();
    server.send(200,"text/plain","Configuration saved.");
  });
  server.on("/log",      [](){server.send(200,"application/json",logToJson());});
  server.on("/log-clear",[](){logClear();server.send(200,"text/plain","OK");});
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
  relayAllOff();

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
  dTextCentered(95,"Starting...",D_WHITE);

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
    dTextCentered(95,"Connecting...",D_WHITE);
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

  xTaskCreatePinnedToCore(taskGateController,"gate",   4096,nullptr,2,&hTaskGate, 1);
  xTaskCreatePinnedToCore(taskInputHandler,  "input",  2048,nullptr,2,&hTaskInput,1);
  xTaskCreatePinnedToCore(taskMonitor,       "monitor",2048,nullptr,1,&hTaskMon,  0);
  xTaskCreatePinnedToCore(taskWeb,           "web",    4096,nullptr,1,&hTaskWeb,  0);
  xTaskCreatePinnedToCore(taskDisplay,       "display",4096,nullptr,1,&hTaskDisp, 1);

  Serial.printf("[INIT] Ready - http://%s\n",currentIP.c_str());
}

void loop() { vTaskDelay(portMAX_DELAY); }
