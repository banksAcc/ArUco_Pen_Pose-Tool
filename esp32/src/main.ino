#include <Arduino.h>
#include <NimBLEDevice.h>

// -------------------- PINOUT --------------------
const int RED_PIN   = 15;
const int GREEN_PIN = 2;   // se dà noie al boot, valuta 23/21/22/25/26/27/32/33
const int BLUE_PIN  = 4;

const int BTN_EVENT_PIN = 16; // SHORT/LONG -> TX
const int BTN_BLE_PIN   = 17; // gestione advertising

// -------------------- LEDC (PWM v3.x pin-based) --------------------
const int PWM_FREQ = 5000; // 5 kHz
const int PWM_RES  = 8;    // 8 bit
float globalBrightness = 0.10; // 0..1

// ---- Tipi PRIMA delle funzioni ----
enum class LedState { OFF, ARMING, ADVERTISING, CONNECTED, ERROR_ };

struct Button {
  int pin;
  bool lastStable = true;   // pullup: HIGH=rilasciato, LOW=premuto
  bool lastRead   = true;
  unsigned long lastChangeMs = 0;
  unsigned long pressedStartMs = 0;
  bool longHandled = false;
};

// ---- Prototipi ----
void setLedState(LedState s);
void setRGB(uint8_t r, uint8_t g, uint8_t b);

// -------------------- LED helpers --------------------
static inline uint8_t inv(uint8_t v) { return 255 - v; } // anodo comune
uint8_t applyB(uint8_t v) {
  float s = v * globalBrightness;
  if (s < 0) s = 0; if (s > 255) s = 255;
  return (uint8_t)s;
}
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  r = applyB(r); g = applyB(g); b = applyB(b);
  ledcWrite(RED_PIN,   inv(r));
  ledcWrite(GREEN_PIN, inv(g));
  ledcWrite(BLUE_PIN,  inv(b));
}
void setColorOff() { setRGB(0,0,0); }

// -------------------- BUTTONS --------------------
Button btnEvent{BTN_EVENT_PIN};
Button btnBle{BTN_BLE_PIN};
const unsigned long DEBOUNCE_MS    = 40;
const unsigned long LONG_MS_BLE    = 4000; // 4 s per BLE
const unsigned long LONG_MS_EVENT  = 2000; // 2 s per SHORT/LONG

void initButton(Button &b) {
  pinMode(b.pin, INPUT_PULLUP);
  b.lastStable = digitalRead(b.pin);
  b.lastRead   = b.lastStable;
  b.lastChangeMs = millis();
  b.pressedStartMs = 0;
  b.longHandled = false;
}

bool updateButton(Button &b, bool &fellEdge, bool &roseEdge, bool &longPress, unsigned long longMs) {
  bool now = digitalRead(b.pin);
  unsigned long t = millis();
  fellEdge = roseEdge = longPress = false;

  if (now != b.lastRead) { b.lastRead = now; b.lastChangeMs = t; }
  if ((t - b.lastChangeMs) >= DEBOUNCE_MS && now != b.lastStable) {
    b.lastStable = now;
    if (now == LOW) { fellEdge = true; b.pressedStartMs = t; b.longHandled = false; }
    else { roseEdge = true; b.pressedStartMs = 0; b.longHandled = false; }
  }
  if (!b.longHandled && b.lastStable == LOW && (t - b.pressedStartMs) >= longMs) {
    b.longHandled = true; longPress = true;
  }
  return b.lastStable == LOW;
}

// -------------------- BLE (NimBLE) --------------------
static NimBLEServer*         pServer = nullptr;
static NimBLECharacteristic* pTxChar = nullptr; // Notify to PC
static NimBLECharacteristic* pRxChar = nullptr; // Write from PC

// Nordic UART Service UUIDs
static NimBLEUUID UUID_SERVICE("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID UUID_RX     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // write
static NimBLEUUID UUID_TX     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // notify

volatile bool bleEnabled   = false;
volatile bool bleConnected = false;
static uint16_t lastConnHandle = 0xFFFF;

// stato “arming” BLE
static bool blePressArmed = false;
// stato iniziale al momento della pressione
static bool pressInitialConnected = false;
static bool pressInitialEnabled   = false;

// evento SHORT/LONG
static bool evtIsLong = false;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    bleConnected = true;
    lastConnHandle = info.getConnHandle();
    Serial.println("[BLE] Connected");
    setLedState(LedState::CONNECTED);
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    bleConnected = false;
    lastConnHandle = 0xFFFF;
    Serial.printf("[BLE] Disconnected (reason %d)\n", reason);
    if (bleEnabled) {
      NimBLEDevice::startAdvertising();
      Serial.println("[BLE] Advertising restarted");
      setLedState(LedState::ADVERTISING);
    } else {
      setLedState(LedState::OFF);
    }
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    std::string v = c->getValue();
    if (!v.empty()) {
      Serial.print("[BLE] RX: ");
      Serial.write((const uint8_t*)v.data(), v.size());
      Serial.println();
    }
  }
};

void bleInitOnce() {
  static bool inited = false;
  if (inited) return; inited = true;

  NimBLEDevice::init("ESP32-RGB-BLE");

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(UUID_SERVICE);
  pTxChar = pService->createCharacteristic(UUID_TX, NIMBLE_PROPERTY::NOTIFY);
  pRxChar = pService->createCharacteristic(UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pRxChar->setCallbacks(new RxCallbacks());
  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData ad, sd;
  ad.setName("ESP32-RGB-BLE");
  ad.setCompleteServices(NimBLEUUID(UUID_SERVICE));
  sd.setName("ESP32-RGB-BLE");
  pAdv->setAdvertisementData(ad);
  pAdv->setScanResponseData(sd);

  Serial.println("[BLE] Stack initialized");
}

void bleStartAdvertising() {
  bleEnabled = true;
  NimBLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising ON");
}

void bleStopAdvertising() {
  bleEnabled = false;
  NimBLEDevice::stopAdvertising();
  Serial.println("[BLE] Advertising OFF");
}

// -------------------- LED status --------------------
LedState ledState = LedState::OFF;

void setLedState(LedState s) {
  ledState = s;
  switch (s) {
    case LedState::CONNECTED:
      setRGB(0, 200, 0);          // verde fisso
      break;
    case LedState::OFF:
      setColorOff();
      break;
    default: break; // ARMING/ADVERTISING/ERROR_ gestiti da updateLedEffects
  }
}

void updateLedEffects() {
  unsigned long t = millis();
  switch (ledState) {
    case LedState::ARMING: {
      // giallo lampeggiante ~4 Hz
      if ((t / 125) % 2) setRGB(255, 255, 0);
      else setColorOff();
    } break;
    case LedState::ADVERTISING: {
      // blu "breathing" ~2 s
      float phase = ((t % 2000UL) / 2000.0f) * 6.2831853f;
      uint8_t lvl = (uint8_t)(30 + 225 * (0.5f * (sinf(phase) + 1.0f)));
      setRGB(0, 0, lvl);
    } break;
    case LedState::ERROR_: {
      if ((t / 250) % 2) setRGB(255, 0, 0); else setColorOff();
    } break;
    default: break;
  }
}

// -------------------- SETUP / LOOP --------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // BOOT DARK: spento prima del PWM (anodo comune: HIGH = spento)
  pinMode(RED_PIN,   OUTPUT); digitalWrite(RED_PIN,   HIGH);
  pinMode(GREEN_PIN, OUTPUT); digitalWrite(GREEN_PIN, HIGH);
  pinMode(BLUE_PIN,  OUTPUT); digitalWrite(BLUE_PIN,  HIGH);

  // PWM + duty spento subito
  ledcAttach(RED_PIN,   PWM_FREQ, PWM_RES);   ledcWrite(RED_PIN,   255);
  ledcAttach(GREEN_PIN, PWM_FREQ, PWM_RES);   ledcWrite(GREEN_PIN, 255);
  ledcAttach(BLUE_PIN,  PWM_FREQ, PWM_RES);   ledcWrite(BLUE_PIN,  255);
  setColorOff();

  initButton(btnEvent);
  initButton(btnBle);

  bleInitOnce();

  // Boot: entra subito in advertising
  bleStartAdvertising();
  setLedState(LedState::ADVERTISING);

  Serial.println("== READY ==");
  Serial.println("BTN16: SHORT/LONG (2s) -> TX");
  Serial.println("BTN17: hold 4s con arming giallo; logica: disconn/advertising/off secondo stato iniziale");
}

void loop() {
  bool fell, rose, lpress;

  // --- BTN17: pairing/advertising ---
  bool pressedBle = updateButton(btnBle, fell, rose, lpress, LONG_MS_BLE);

  if (fell) {
    // entra subito in "arming" (giallo lampeggiante)
    pressInitialConnected = bleConnected;
    pressInitialEnabled   = bleEnabled;
    blePressArmed = true;
    setLedState(LedState::ARMING);
    Serial.printf("[BLE] Pair press: arming (start state: %s/%s)\n",
                  pressInitialConnected ? "CONNECTED" : "NOT_CONN",
                  pressInitialEnabled ? "ADV_ON" : "ADV_OFF");
  }

  if (lpress && blePressArmed) {
    // LONG PRESS (>=4s): azione su stato iniziale
    if (pressInitialConnected) {
      // disconnect -> ADV ON
      if (bleConnected && pServer && lastConnHandle != 0xFFFF) {
        Serial.println("[BLE] Long: disconnect then ADV");
        bleStartAdvertising(); // abilita prima, così onDisconnect riavvia subito
        pServer->disconnect(lastConnHandle);
      } else {
        bleStartAdvertising();
        setLedState(LedState::ADVERTISING);
      }
    } else if (pressInitialEnabled) {
      // era già in advertising -> spegni
      Serial.println("[BLE] Long: stop advertising");
      bleStopAdvertising();
      setLedState(LedState::OFF);
    } else {
      // era OFF -> avvia advertising
      Serial.println("[BLE] Long: start advertising");
      bleStartAdvertising();
      setLedState(LedState::ADVERTISING);
    }
    blePressArmed = false;
  }

  if (rose && blePressArmed) {
    // SHORT PRESS (<4s): azione su stato iniziale
    if (pressInitialConnected) {
      // disconnect -> LED OFF (niente advertising)
      Serial.println("[BLE] Short: disconnect -> OFF");
      bleStopAdvertising(); // assicurati che non riparta da callback
      if (bleConnected && pServer && lastConnHandle != 0xFFFF) {
        pServer->disconnect(lastConnHandle);
      }
      setLedState(LedState::OFF);
    } else {
      // se era in advertising: annulla e resta in advertising
      // se era OFF: resta OFF
      Serial.println("[BLE] Short: canceled -> restore");
      if (pressInitialEnabled) setLedState(LedState::ADVERTISING);
      else setLedState(LedState::OFF);
    }
    blePressArmed = false;
  }

  // Allineamento automatico LED se non siamo in ARming
  if (!blePressArmed) {
    if (bleConnected && ledState != LedState::CONNECTED) setLedState(LedState::CONNECTED);
    if (!bleConnected && bleEnabled && ledState != LedState::ADVERTISING) setLedState(LedState::ADVERTISING);
    if (!bleConnected && !bleEnabled && ledState != LedState::OFF) setLedState(LedState::OFF);
  }

  // --- BTN16: evento SHORT/LONG (2s) ---
  bool fellE, roseE, lpressE;
  bool pressedEvt = updateButton(btnEvent, fellE, roseE, lpressE, LONG_MS_EVENT);

  if (fellE) evtIsLong = false;
  if (lpressE) evtIsLong = true;
  if (roseE) {
    if (bleConnected && pTxChar) {
      if (evtIsLong) {
        const char* msg = "LONG\n";
        pTxChar->setValue((uint8_t*)msg, strlen(msg));
        pTxChar->notify();
        Serial.println("[BLE] TX: LONG");
      } else {
        const char* msg = "SHORT\n";
        pTxChar->setValue((uint8_t*)msg, strlen(msg));
        pTxChar->notify();
        Serial.println("[BLE] TX: SHORT");
      }
    } else {
      Serial.println("[BLE] Not connected -> ignoring event press");
      setRGB(255, 0, 0); delay(80); setColorOff();
    }
  }

  updateLedEffects();
}
