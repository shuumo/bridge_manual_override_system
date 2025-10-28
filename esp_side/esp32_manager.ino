#include <WiFi.h>
#include <WiFiUdp.h>

const char* WIFI_SSID = "SSID";
const char* WIFI_PASS = "PASSWORD";

const unsigned int LOCAL_PORT = 4210;
const unsigned long HEARTBEAT_TIMEOUT_MS = 3000;

WiFiUDP Udp;
unsigned long lastPingTime = 0;
unsigned long lastOkSent = 0;

bool connectedToHost = false;
bool overrideMode = false;

enum class BridgeState { INACTIVE, IDLE, OPENING, OPEN, CLOSING, ERROR };
enum class ControlMode { AUTO, OVERRIDE };

BridgeState currentState = BridgeState::IDLE;
ControlMode currentMode = ControlMode::AUTO;

IPAddress hostIP;
uint16_t hostPort = 0;

String stateToString(BridgeState s) {
  switch(s) {
    case BridgeState::IDLE: return "IDLE";
    case BridgeState::OPENING: return "OPENING";
    case BridgeState::OPEN: return "OPEN";
    case BridgeState::CLOSING: return "CLOSING";
    case BridgeState::ERROR: return "ERROR";
    default: return "INACTIVE";
  }
}

String modeToString(ControlMode m) {
  return (m == ControlMode::AUTO) ? "AUTO" : "OVERRIDE";
}

void sendStatus() {
  if(!connectedToHost) return;

  String msg = "OK STATE=" + stateToString(currentState) + " MODE=" + modeToString(currentMode);
  Udp.beginPacket(hostIP, hostPort);
  Udp.print(msg);
  Udp.endPacket();

  lastOkSent = millis();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nBridge/Esp code Starting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  if(Udp.begin(LOCAL_PORT)) {
    Serial.printf("UDP listening on port %u\n", LOCAL_PORT);
  } else {
    Serial.println("Failed to bind UDP port!");
  }
}

void loop() {
  int packetSize = Udp.parsePacket();
  if(packetSize > 0) {
    char incoming[64];
    int len = Udp.read(incoming, sizeof(incoming) - 1);
    if(len > 0) incoming[len] = '\0';

    String msg = String(incoming);
    msg.trim();

    hostIP = Udp.remoteIP();
    hostPort = Udp.remotePort();
    connectedToHost = true;
    lastPingTime = millis();

    Serial.printf("udp from %s:%u -> %s\n",
                  hostIP.toString().c_str(), hostPort, msg.c_str());

    if(msg == "PING") {
      sendStatus();
      return;
    }

    if(msg.startsWith("MODE=")) {
      String mode = msg.substring(5);
      if(mode == "AUTO") {
        currentMode = ControlMode::AUTO;
        Serial.println("Switched to AUTO mode.");
      } else if(mode == "OVERRIDE") {
        currentMode = ControlMode::OVERRIDE;
        Serial.println("Switched to OVERRIDE mode.");
      }
      sendStatus();
      return;
    }

    if(msg.startsWith("SET_STATE")) {
      String state = msg.substring(String("SET_STATE").length());
      state.trim();

      if(currentMode == ControlMode::AUTO) {
        Serial.println("Ignoring SET_STATE because mode is AUTO.");
      } else {
        if(state == "IDLE") currentState = BridgeState::IDLE;
        else if(state == "OPENING") currentState = BridgeState::OPENING;
        else if(state == "OPEN") currentState = BridgeState::OPEN;
        else if(state == "CLOSING") currentState = BridgeState::CLOSING;
        else if(state == "ERROR") currentState = BridgeState::ERROR;

        Serial.printf("Manual override -> new state: %s\n", stateToString(currentState).c_str());
      }
      sendStatus();
      return;
    }
  }

  if(connectedToHost && (millis() - lastPingTime > HEARTBEAT_TIMEOUT_MS)) {
    connectedToHost = false;
    Serial.println("Connection lost (no heartbeat).");
  }

  if(currentMode == ControlMode::AUTO) {
    // TODO: auto logic here
    // e.g. check sensors, handle timing, etc.
    static unsigned long lastPrint = 0;
    if(millis() - lastPrint > 3000) {
      Serial.println("Running automatic control logic...");
      lastPrint = millis();
    }
  }

  delay(10);
}

