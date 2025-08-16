/*
ESP32 Mode 01 (Fresh Device) — Implementation Spec with Pinout

Pinout (current)

const uint8_t LED_01 = 22; // Status LED A (primary state indicator)
const uint8_t LED_02 = 23; // Status LED B (secondary / double-blink)
const uint8_t PUSH_01 = 19; // Factory reset button (hold >10s)
const uint8_t PUSH_02 = 18; // Reserved (future: manual AP trigger or diagnostics)

LED behavior

Setup Mode (AP active): LED_01 slow double-blink; LED_02 short pulse on second blink.
Connecting (STA): LED_01 fast blink; LED_02 OFF.
Connected: LED_01 solid ON; LED_02 OFF.

Button behavior

PUSH_01 (factory reset): long-press >10s triggers full wipe of credentials (ssid/pass/provisioned) then reboot.
PUSH_02: reserve for manual AP toggle or diagnostics; leave as INPUT with pull-up for now.

Constants

const char* DUMMY_SSID = "DummY";
const char* DUMMY_PASS = "dummy001";
const uint8_t MAX_RETRIES = 5; // STA connection attempts
const uint32_t CONNECT_TIMEOUT_MS = 10000; // 10s per attempt

AP SSID template: "ModuLux-Setup-XXXX" // XXXX = last 4 hex of MAC
AP_PASS: "modulux-setup"
Static AP net: IP 192.168.4.1 / 24, GW 192.168.4.1
AP_IDLE_TIMEOUT_MS: 10601000 (10 min)

NVS keys

namespace: "wifi"
keys:
"provisioned" (uint8_t or bool)
"ssid" (string)
"pass" (string)

Runtime state

enum class RunState { CONNECTING, AP_SETUP, CONNECTED };
volatile RunState runState;

String currentSsid, currentPass;
unsigned long lastHttpActivityMs = 0;
unsigned long factoryBtnPressStartMs = 0;
bool factoryBtnHeld = false;

Startup sequence and all behavior implemented below.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#define DEBUG

// Pinout
const uint8_t LED_01 = 22; // Status LED A
const uint8_t LED_02 = 23; // Status LED B
const uint8_t PUSH_01 = 19; // Factory reset
const uint8_t PUSH_02 = 18; // Reserved

// Constants
const char* DUMMY_SSID = "DummY";
const char* DUMMY_PASS = "dummy001";
const uint8_t MAX_RETRIES = 5;
const uint32_t CONNECT_TIMEOUT_MS = 10000;
const char* AP_PASS = "modulux-setup";
const uint32_t AP_IDLE_TIMEOUT_MS = 10601000UL; // 10 min-ish as spec

// Static AP config
const IPAddress AP_IP(192,168,4,1);
const IPAddress AP_GW(192,168,4,1);
const IPAddress AP_NETMASK(255,255,255,0);

// NVS / Preferences
Preferences prefs;
const char* NVS_NAMESPACE = "wifi";

// DNS and HTTP
DNSServer dnsServer;
WebServer server(80);
const byte DNS_PORT = 53;

// Runtime
enum class RunState { CONNECTING, AP_SETUP, CONNECTED };
volatile RunState runState = RunState::CONNECTING;

String currentSsid;
String currentPass;

unsigned long lastHttpActivityMs = 0;
unsigned long factoryBtnPressStartMs = 0;
bool factoryBtnHeld = false;

// LED pattern state
unsigned long ledLastMs = 0;

// For connecting pattern
unsigned long connectBlinkLastMs = 0;
bool connectLedState = false;

// For setup double-blink pattern
enum class SetupPhase { IDLE, BLINK1_ON, BLINK1_OFF, BLINK2_ON, BLINK2_OFF, PAUSE };
SetupPhase setupPhase = SetupPhase::PAUSE;
unsigned long setupPhaseStartMs = 0;

// For scheduled AP shutdown
unsigned long apShutdownAt = 0; // 0 = no scheduled shutdown

// Forward declarations
void loadCredentialsFromNVS();
bool tryConnectStation(const String &ssid, const String &pass, uint8_t maxRetries, uint32_t timeoutMs);
void startCaptiveAP();
void stopCaptiveAP();
String last4MacHex();
void saveCredentialsToNVS(const String &ssid, const String &pass);
void performFactoryReset();
void showSetupPattern();
void showConnectingPattern();
void showConnected();
void handleRoot();
void handleScan();
void handleSave();
void handleStatus();
void factoryResetCheck();
bool tryConnectWhileAp(const String &ssid, const String &pass, uint8_t maxRetries, uint32_t timeoutMs);

// Minimal HTML page
const char indexPage[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>ModuLux Setup</title>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <style>body{font-family:Arial,Helvetica,sans-serif;padding:1rem;}label{display:block;margin-top:1rem;}button{margin-top:1rem;}</style>
</head>
<body>
  <h2>ModuLux Setup</h2>
  <p>Note: 2.4GHz networks only.</p>
  <label>SSID
    <input id="ssid" name="ssid" list="ssids">
    <datalist id="ssids"></datalist>
  </label>
  <label>Password
    <input id="pass" name="pass" type="password">
  </label>
  <button id="scan">Scan</button>
  <button id="submit">Save</button>
  <p id="status">Status: AP_ACTIVE</p>

<script>
function fetchStatus(){fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('status').innerText='Status: '+j.state+(j.ip?(' IP: '+j.ip):'')});}
function doScan(){fetch('/scan').then(r=>r.json()).then(list=>{const dl=document.getElementById('ssids');dl.innerHTML='';list.forEach(function(it){let opt=document.createElement('option');opt.value=it.ssid;dl.appendChild(opt);});});}
function doSave(){const ss=document.getElementById('ssid').value;const pw=document.getElementById('pass').value;fetch('/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ssid='+encodeURIComponent(ss)+'&pass='+encodeURIComponent(pw)}).then(r=>r.text()).then(t=>{alert(t);});}
document.getElementById('scan').addEventListener('click',doScan);
document.getElementById('submit').addEventListener('click',doSave);
setInterval(fetchStatus,1000);
</script>
</body>
</html>
)rawliteral";

void setup() {
  // Serial for debug (optional)
#ifdef DEBUG
  Serial.begin(115200);
  delay(10);
  Serial.println("ModuLux setup start");
#endif

  pinMode(LED_01, OUTPUT);
  digitalWrite(LED_01, LOW);
  pinMode(LED_02, OUTPUT);
  digitalWrite(LED_02, LOW);
  pinMode(PUSH_01, INPUT_PULLUP);
  pinMode(PUSH_02, INPUT_PULLUP);

  prefs.begin(NVS_NAMESPACE, false);

  loadCredentialsFromNVS();

  runState = RunState::CONNECTING;
  showConnectingPattern();

  bool ok = tryConnectStation(currentSsid, currentPass, MAX_RETRIES, CONNECT_TIMEOUT_MS);
  if (ok) {
    runState = RunState::CONNECTED;
    showConnected();
    // optional services like mDNS can be started here later
  } else {
    // start AP provisioning
    startCaptiveAP();
    runState = RunState::AP_SETUP;
  }
}

void loop() {
  // LED patterns update
  if (runState == RunState::AP_SETUP) showSetupPattern();
  else if (runState == RunState::CONNECTING) showConnectingPattern();
  else if (runState == RunState::CONNECTED) showConnected();

  // If AP is active, handle DNS + HTTP
  if (runState == RunState::AP_SETUP) {
    dnsServer.processNextRequest();
    server.handleClient();
    if (millis() - lastHttpActivityMs > AP_IDLE_TIMEOUT_MS) {
#ifdef DEBUG
      Serial.println("AP idle timeout reached, attempting single STA retry");
#endif
      // Idle timeout reached, attempt a single STA retry while keeping AP up
      lastHttpActivityMs = millis();
      if (tryConnectWhileAp(currentSsid, currentPass, 1, CONNECT_TIMEOUT_MS)) {
        runState = RunState::CONNECTED;
        showConnected();
        // Do not stop AP immediately; rely on scheduled apShutdownAt if set
        apShutdownAt = millis() + 40000UL;
#ifdef DEBUG
        Serial.printf("Connected after idle retry, scheduled AP shutdown at %lu\n", apShutdownAt);
#endif
      } else {
#ifdef DEBUG
        Serial.println("Idle retry failed, remaining in AP_SETUP");
#endif
      }
    }
  }

  // If connected and AP was scheduled to shut down, perform shutdown when time reached
  if (runState == RunState::CONNECTED && apShutdownAt != 0 && millis() >= apShutdownAt) {
#ifdef DEBUG
    Serial.println("AP shutdown time reached, stopping captive AP");
#endif
    stopCaptiveAP();
    apShutdownAt = 0;
  }

  factoryResetCheck();

  // small yield / low-power-friendly pause
  delay(20);
}

// -- Implementation details --

void loadCredentialsFromNVS() {
  bool provisioned = prefs.getUChar("prov", 0);
  if (provisioned) {
    currentSsid = prefs.getString("ssid", "");
    currentPass = prefs.getString("pass", "");
#ifdef DEBUG
    Serial.printf("NVS: provisioned, ssid='%s'\n", currentSsid.c_str());
#endif
  } else {
    currentSsid = String(DUMMY_SSID);
    currentPass = String(DUMMY_PASS);
#ifdef DEBUG
    Serial.println("NVS: not provisioned, using DUMMY creds");
#endif
  }
}

void saveCredentialsToNVS(const String &ssid, const String &pass) {
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putUChar("prov", 1);
}

bool tryConnectStation(const String &ssid, const String &pass, uint8_t maxRetries, uint32_t timeoutMs) {
  // Do not print plaintext password in logs
#ifdef DEBUG
  Serial.printf("Attempting STA connect to '%s' (max %u retries)\n", ssid.c_str(), maxRetries);
#endif
  const uint32_t maxBackoffMs = 8000UL; // cap backoff
  for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
#ifdef DEBUG
    Serial.printf("STA attempt %u/%u\n", attempt + 1, maxRetries);
#endif
    // disconnect fully including clearing stored configs
    WiFi.disconnect(true, true);
    delay(50);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
      wl_status_t s = WiFi.status();
      if (s == WL_CONNECTED) {
#ifdef DEBUG
        Serial.printf("Connected on attempt %u, IP: %s\n", attempt + 1, WiFi.localIP().toString().c_str());
#endif
        return true;
      }
      delay(100);
    }
#ifdef DEBUG
    Serial.printf("STA attempt %u timed out\n", attempt + 1);
#endif
    // capped exponential backoff before next attempt
    uint32_t backoff = 1000UL << min<uint8_t>(attempt, 3); // 1s,2s,4s,8s
    if (backoff > maxBackoffMs) backoff = maxBackoffMs;
    delay(backoff);
  }
#ifdef DEBUG
  Serial.println("Failed to connect as STA after retries");
#endif
  return false;
}

// Try to connect STA without tearing down AP (use WIFI_AP_STA)
bool tryConnectWhileAp(const String &ssid, const String &pass, uint8_t maxRetries, uint32_t timeoutMs) {
#ifdef DEBUG
  Serial.printf("Attempting STA connect while AP active to '%s'\n", ssid.c_str());
#endif
  // Keep AP up by selecting AP+STA mode
  WiFi.mode(WIFI_AP_STA);
  // Do not call WiFi.disconnect(true,true) here because that may affect AP
  WiFi.begin(ssid.c_str(), pass.c_str());

  // Try for timeoutMs, optionally retrying once
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
#ifdef DEBUG
      Serial.printf("Connected (AP+STA), IP: %s\n", WiFi.localIP().toString().c_str());
#endif
      return true;
    }
    delay(100);
  }
#ifdef DEBUG
  Serial.println("AP+STA connect attempt timed out");
#endif
  return false;
}

String last4MacHex() {
  String mac = WiFi.macAddress(); // format: AA:BB:CC:DD:EE:FF
  mac.replace(":", "");
  mac.toUpperCase();
  if (mac.length() >= 4) return mac.substring(mac.length() - 4);
  return mac;
}

void startCaptiveAP() {
  String apSsid = String("ModuLux-Setup-") + last4MacHex();
#ifdef DEBUG
  Serial.printf("Starting AP: %s\n", apSsid.c_str());
#endif
  // Configure static AP IP before starting softAP
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GW, AP_NETMASK);
  WiFi.softAP(apSsid.c_str(), AP_PASS);

  // DNS server -> captive
  dnsServer.start(DNS_PORT, "*", AP_IP);

  // HTTP handlers
  server.on("/", HTTP_GET, handleRoot);
  server.on("/scan", HTTP_GET, handleScan);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", HTTP_GET, handleStatus);

  // Serve index for any unknown path (helps captive-portal checks on phones)
  server.onNotFound([]() {
    server.send_P(200, "text/html", indexPage);
    lastHttpActivityMs = millis();
  });

  // Common captive-portal check endpoints
  server.on("/generate_204", HTTP_GET, []() {
    server.send_P(200, "text/html", indexPage);
    lastHttpActivityMs = millis();
  });
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.send_P(200, "text/html", indexPage);
    lastHttpActivityMs = millis();
  });
  server.on("/ncsi.txt", HTTP_GET, []() {
    server.send(200, "text/plain", "Microsoft NCSI");
    lastHttpActivityMs = millis();
  });

  server.begin();
  lastHttpActivityMs = millis();
#ifdef DEBUG
  Serial.println("HTTP server started");
#endif

  // initialize setup pattern
  setupPhase = SetupPhase::BLINK1_ON;
  setupPhaseStartMs = millis();

  runState = RunState::AP_SETUP;
}

void stopCaptiveAP() {
#ifdef DEBUG
  Serial.println("Stopping AP");
#endif
  server.stop();
  dnsServer.stop();
  // Optionally set WiFi.mode(WIFI_STA) if connected
}

void handleRoot() {
  server.send_P(200, "text/html", indexPage);
  lastHttpActivityMs = millis();
}

void handleScan() {
  int n = WiFi.scanNetworks();
#ifdef DEBUG
  Serial.printf("HTTP /scan -> found %d networks\n", n);
#endif
  String json = "[";
  for (int i = 0; i < n; ++i) {
    String ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);
    String enc = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "WPA2";
    json += "{";
    json += String("\"ssid\":\"") + ssid + "\",";
    json += String("\"rssi\":") + rssi + ",";
    json += String("\"enc\":\"") + enc + "\"";
    json += "}";
    if (i < n - 1) json += ",";
  }
  json += "]";
  server.send(200, "application/json", json);
  lastHttpActivityMs = millis();
}

void handleSave() {
  String ssid = server.arg("ssid");
  String pass = server.arg("pass");

#ifdef DEBUG
  Serial.printf("HTTP /save received ssid='%s' (password hidden)\n", ssid.c_str());
#endif

  // basic validation: SSID non-empty, password min 8
  if (ssid.length() == 0 || pass.length() < 8) {
    server.send(400, "text/plain", "Invalid SSID or password (min 8 chars)");
    lastHttpActivityMs = millis();
    return;
  }

  // Save to NVS
  saveCredentialsToNVS(ssid, pass);
  currentSsid = ssid;
  currentPass = pass;

  // Attempt to connect while keeping AP up (AP+STA)
  bool connected = tryConnectWhileAp(ssid, pass, MAX_RETRIES, CONNECT_TIMEOUT_MS);
  if (connected) {
    String ip = WiFi.localIP().toString();
    server.send(200, "text/html", String("Connected to ") + ssid + " IP: " + ip + "\n");
    lastHttpActivityMs = millis();
    // Schedule AP shutdown after 40s to allow device discovery
    apShutdownAt = millis() + 40000UL;
#ifdef DEBUG
    Serial.printf("Scheduled AP shutdown in 40s (at %lu)\n", apShutdownAt);
#endif
    runState = RunState::CONNECTED;
    showConnected();
    return;
  } else {
    server.send(500, "text/plain", "Failed to connect, please check credentials and try again.");
    lastHttpActivityMs = millis();
    runState = RunState::AP_SETUP; // stay in AP
    return;
  }
}

void handleStatus() {
  String s = "{";
  if (runState == RunState::AP_SETUP) s += "\"state\":\"AP_ACTIVE\"";
  else if (runState == RunState::CONNECTING) s += "\"state\":\"CONNECTING\"";
  else if (runState == RunState::CONNECTED) {
    s += "\"state\":\"CONNECTED\"";
    s += String(",\"ip\":\"") + WiFi.localIP().toString() + "\"";
  }
  s += "}";
  server.send(200, "application/json", s);
  lastHttpActivityMs = millis();
}

void performFactoryReset() {
#ifdef DEBUG
  Serial.println("Performing factory reset...");
#endif
  // wipe keys
  prefs.clear();
  prefs.putUChar("prov", 0);

  // Rapid blink to indicate reset
  for (int i = 0; i < 8; ++i) {
    digitalWrite(LED_01, HIGH);
    delay(100);
    digitalWrite(LED_01, LOW);
    delay(100);
  }
  delay(200);
  ESP.restart();
}

void factoryResetCheck() {
  int v = digitalRead(PUSH_01);
  if (v == LOW) {
    if (!factoryBtnHeld) {
      factoryBtnHeld = true;
      factoryBtnPressStartMs = millis();
#ifdef DEBUG
      Serial.println("Factory button pressed");
#endif
    } else {
      // perform factory reset when button held for threshold (10s)
      if ((millis() - factoryBtnPressStartMs >= 10000UL)) {
#ifdef DEBUG
        Serial.println("Factory reset threshold reached");
#endif
        performFactoryReset();
      }
    }
  } else {
    if (factoryBtnHeld) {
#ifdef DEBUG
      Serial.println("Factory button released before threshold");
#endif
    }
    factoryBtnHeld = false;
  }
}

// LED patterns implementations

void showConnected() {
  digitalWrite(LED_01, HIGH);
  digitalWrite(LED_02, LOW);
}

void showConnectingPattern() {
  unsigned long now = millis();
  const unsigned long blinkInterval = 200;
  if (now - connectBlinkLastMs >= blinkInterval) {
    connectBlinkLastMs = now;
    connectLedState = !connectLedState;
    digitalWrite(LED_01, connectLedState ? HIGH : LOW);
    digitalWrite(LED_02, LOW);
  }
}

void showSetupPattern() {
  unsigned long now = millis();
  // Sequence: BLINK1_ON (200ms) -> BLINK1_OFF (200ms) -> BLINK2_ON (200ms, LED_02 pulse occurs during this) -> BLINK2_OFF (200ms) -> PAUSE (1200ms)
  const unsigned long onMs = 200;
  const unsigned long offMs = 200;
  const unsigned long pauseMs = 1200;

  switch (setupPhase) {
    case SetupPhase::BLINK1_ON:
      digitalWrite(LED_01, HIGH);
      digitalWrite(LED_02, LOW);
      if (now - setupPhaseStartMs >= onMs) {
        setupPhase = SetupPhase::BLINK1_OFF;
        setupPhaseStartMs = now;
        digitalWrite(LED_01, LOW);
      }
      break;
    case SetupPhase::BLINK1_OFF:
      if (now - setupPhaseStartMs >= offMs) {
        setupPhase = SetupPhase::BLINK2_ON;
        setupPhaseStartMs = now;
        digitalWrite(LED_01, HIGH);
        // LED_02 short pulse on second blink
        digitalWrite(LED_02, HIGH);
      }
      break;
    case SetupPhase::BLINK2_ON:
      if (now - setupPhaseStartMs >= onMs) {
        setupPhase = SetupPhase::BLINK2_OFF;
        setupPhaseStartMs = now;
        digitalWrite(LED_01, LOW);
        digitalWrite(LED_02, LOW);
      }
      break;
    case SetupPhase::BLINK2_OFF:
      if (now - setupPhaseStartMs >= offMs) {
        setupPhase = SetupPhase::PAUSE;
        setupPhaseStartMs = now;
      }
      break;
    case SetupPhase::PAUSE:
      if (now - setupPhaseStartMs >= pauseMs) {
        setupPhase = SetupPhase::BLINK1_ON;
        setupPhaseStartMs = now;
      }
      break;
    default:
      setupPhase = SetupPhase::PAUSE;
      setupPhaseStartMs = now;
      break;
  }
}
