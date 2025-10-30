//IF IT STOPS CHECK VOLTAGE
//IF STOPS CHECK THE SCREWS ARENT TO TIGHT
// IF THE THINGS AROUND MOTOR ARE TO TIGHT THE FRICTION WILL STOP IT
//masking tape wires

// trial lifting bridgre with 3kg on it
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Arduino.h>
#include "Ultrasonic4.h"
#include "Motor.h"
#include "defs.h"

//green into ground
// blue onto power

// wifi global variables
const char* ssid = "arnavs";
const char* password = "arnav123";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;
char incomingPacket[255];
IPAddress remoteIp;
unsigned int remotePort;

unsigned long lastHeartbeat = 0;
unsigned long lastPingReceived = 0;
bool wifiConnected = false;

// manual override globals
enum ControlMode { AUTO_MODE, MANUAL_MODE };
ControlMode controlMode = AUTO_MODE;
bool manualSequenceActive = false;



// ========== BRIDGE CONFIGURATION ==========
#define BRIDGE_UP_ANGLE 40 //90.0      // Degrees for bridge fully raised // erin!!!!
#define BRIDGE_DOWN_ANGLE 0.0     // Degrees for bridge fully down
#define BRIDGE_MOTOR_RPM 20.0     // Motor speed - increased from 24
#define BRIDGE_GEAR_RATIO 43.8    // Your actual gear ratio

// ========== TIMING CONFIGURATION (milliseconds) ==========
#define YELLOW_LIGHT_DURATION 3000    // 3 seconds yellow warning
#define BOAT_PASSAGE_TIME 10000       // 10 seconds for boat to pass
#define DEBOUNCE_TIME 500             // IR sensor debounce


//ultrasonic pins
Ultrasonic4 us4(ULTRA_TRIG1, ULTRA_ECHO1,
                ULTRA_TRIG2, ULTRA_ECHO2);

// ========== STATE MACHINE ==========
enum BridgeState {
  IDLE,                    // Bridge down, road green, waiting
  BOAT_DETECTED,           // IR sensor triggered, debouncing
  ROAD_WARNING,            // Yellow lights on, warning drivers
  ROAD_CLOSED,             // Red lights on, waiting for bridge to lift
  BRIDGE_LIFTING,          // Motor raising bridge
  BOAT_PASSAGE,            // Bridge up, boat lights green, boat passing
  BRIDGE_LOWERING,         // Motor lowering bridge
  BRIDGE_STABILIZING       // Bridge down, stabilizing before reopening road
};

// ========== GLOBAL VARIABLES ==========
BridgeState currentState = IDLE;
unsigned long stateStartTime = 0;
unsigned long lastIRTrigger = 0;
bool irSensorTriggered = false;

bool ultraTriggered = false;
unsigned long lastUltraTrigger = 0;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("\n========== VERTICAL LIFT BRIDGE CONTROLLER ==========");
  Serial.println("Initializing...");
  
  // Initialize motor
  Motor::the().begin();
  Motor::the().setGearRatio(BRIDGE_GEAR_RATIO);
  Motor::the().setTargetRPM(BRIDGE_MOTOR_RPM);
  Motor::the().setTargetAngle(BRIDGE_DOWN_ANGLE);
  Serial.println("[MOTOR] Initialized");
  
  // Setup IR sensor and simulation button
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);

  //ultrasonic sensor
us4.begin();

  //  pinMode(SIMULATE_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[SENSOR] IR sensor configured");
  Serial.println("[BUTTON] Simulation button on pin 14 (ENA) - Press to trigger boat detection!");
  
  // Setup road lights
  pinMode(ROAD_LIGHT_1_RED, OUTPUT);
  pinMode(ROAD_LIGHT_1_YELLOW, OUTPUT);
  pinMode(ROAD_LIGHT_1_GREEN, OUTPUT);
  pinMode(ROAD_LIGHT_2_RED, OUTPUT);
  pinMode(ROAD_LIGHT_2_YELLOW, OUTPUT);
  pinMode(ROAD_LIGHT_2_GREEN, OUTPUT);
  Serial.println("[LIGHTS] Road lights configured");
  
  // Setup boat lights 
  pinMode(BOAT_LIGHT_3_RED, OUTPUT);
  pinMode(BOAT_LIGHT_3_YELLOW, OUTPUT);
  pinMode(BOAT_LIGHT_3_GREEN, OUTPUT);
  pinMode(BOAT_LIGHT_4_RED, OUTPUT);
  pinMode(BOAT_LIGHT_4_YELLOW, OUTPUT);
  pinMode(BOAT_LIGHT_4_GREEN, OUTPUT);
  Serial.println("[LIGHTS] Boat lights configured (2 sets)");
  
  // Initial state - road open, boat closed
  setRoadLights(false, false, true);  // Green
  setBoatLights(true, false, false);  // Red
  
  Serial.println("========== INITIALIZATION COMPLETE ==========");
  Serial.println("State: IDLE - Ready for operation");
  Serial.println("\n*** COMMANDS ***");
  Serial.println("  Press button on pin 14 - Start bridge sequence");
  Serial.println("  Type 't' - Test motor movement");
  Serial.println("  Type 'e' - Test encoder readings");
  Serial.println("  Type 'r' - Reset to IDLE\n");
  
  stateStartTime = millis();

WiFi.begin(ssid, password);
Serial.println("[WIFI] Connecting...");

unsigned long startAttemptTime = millis();
while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
  vTaskDelay(100 / portTICK_PERIOD_MS);   
  Serial.print(".");
  yield();                            
}

Serial.println();

if (WiFi.status() == WL_CONNECTED) {
  Serial.print("[WIFI] Connected to ");
  Serial.println(WiFi.localIP());
  wifiConnected = true;
  Udp.begin(localUdpPort);
  Serial.println("[UDP] Listening on port 4210");
} else {
  Serial.println("[WIFI] Failed to connect, starting in AUTO mode.");
  wifiConnected = false;
  controlMode = AUTO_MODE;
}

}

// ========== MAIN LOOP ==========
void loop() {
  // arnavs manual override code
  if (wifiConnected) {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(incomingPacket, 255);
    if (len > 0) incomingPacket[len] = 0;
    String msg = String(incomingPacket);
    msg.trim();
    Serial.print("[UDP] Received: ");
    Serial.println(msg);

    if (msg == "PING") {
      lastPingReceived = millis();
      sendStatus();
    } else if (msg == "MODE=AUTO") {
      controlMode = AUTO_MODE;
      Serial.println("[MODE] Switched to AUTO");
    } else if (msg == "MODE=OVERRIDE") {
      controlMode = MANUAL_MODE;
      Serial.println("[MODE] Switched to MANUAL OVERRIDE");
    } else if (controlMode == MANUAL_MODE) {
      if (msg == "OPEN") handleManualOpen();
      else if (msg == "CLOSE") handleManualClose();
      else if (msg == "STOP") handleManualStop();
    }
  }
}

// connection watchdog
if (wifiConnected && millis() - lastPingReceived > 3000) {
  Serial.println("[WIFI] Heartbeat lost, reverting to AUTO mode.");
  controlMode = AUTO_MODE;
  lastPingReceived = millis();
}




  // Always update motor control
  //Motor::the().updateMovement();
   Motor::the().updateMovementSmooth(); 
 // erin!! comment the above one out and uncomment the Smooth one and it should stop jerk
  
  // Check for serial commands for testing
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'T') {
      testMotor();
    } else if (cmd == 'r' || cmd == 'R') {
      Serial.println("\n*** RESETTING TO IDLE ***");
      Motor::the().reset();
      changeState(IDLE);
    } else if (cmd == 'e' || cmd == 'E') {
      testEncoder();
    }
  }
  
  // Read IR sensor with debouncing
  //checkIRSensor();

  //ultrasonic sensor
  checkUltrasonicSensors(us4);
  //delay(100); // optional, adjust polling rate erin!! idk if needed
  
  // State machine
  unsigned long currentTime = millis();
  unsigned long timeInState = currentTime - stateStartTime;
  if (controlMode == AUTO_MODE) {
    switch (currentState) {
      
      case IDLE:
        // Waiting for boat detection
        if (irSensorTriggered) {
          changeState(BOAT_DETECTED);
        }
        break;
      
      case BOAT_DETECTED:
        // Debounce period - confirm boat is really there
        if (timeInState >= DEBOUNCE_TIME) {
          if (irSensorTriggered) {
            Serial.println("[BOAT] Confirmed - Beginning bridge lift sequence");
            changeState(ROAD_WARNING);
          } else {
            Serial.println("[BOAT] False alarm - Returning to IDLE");
            changeState(IDLE);
          }
        }
        break;
      
      case ROAD_WARNING:
        // Yellow lights warning drivers
        if (timeInState >= YELLOW_LIGHT_DURATION) {
          changeState(ROAD_CLOSED);
        }
        break;
      
      case ROAD_CLOSED:
        // Red lights on, ready to lift
        // Give a brief moment for any last cars to see the red
        if (timeInState >= 1000) {
          changeState(BRIDGE_LIFTING);
        }
        break;
      
      case BRIDGE_LIFTING:
        // Wait for bridge to reach target angle
      //  Motor::the().updateMovementSmooth(); //erin!! uncomment and it should make bridge stop smoother
        if (Motor::the().isAtTarget()) {
          Serial.println("[MOTOR] Bridge fully raised!");
          changeState(BOAT_PASSAGE);
        }
        break;
      
      case BOAT_PASSAGE:
        // Boat lights green, waiting for boat to pass
        if (timeInState >= BOAT_PASSAGE_TIME) {
          Serial.println("[BOAT] Passage time complete - Closing for boats");
          changeState(BRIDGE_LOWERING);
        }
        break;
      
      case BRIDGE_LOWERING:
      // Motor::the().updateMovementSmooth(); //erin!! uncomment and it should make bridge stop smoother
        // Wait for bridge to return to down position
        if (Motor::the().isAtTarget()) {
          Serial.println("[MOTOR] Bridge fully lowered!");
          changeState(BRIDGE_STABILIZING);
        }
        break;
    // /*
      case BRIDGE_STABILIZING:
        // Brief stabilization period before reopening road
        if (timeInState >= 4000) { //was 1000 idk think we could improve bridge stabalizing
          changeState(IDLE);
        }
        break;
      //  */


        // Erin!! added idk work
  /* //case BRIDGE_STABILIZING:
      {
          // keep bridge stable, correct small drifts
          double err = fabs(Motor::the().getCurrentAngle() - Motor::the().getTargetAngle());
          if (err <= 1.0) {
              Motor::the().brake();  // stop completely if within ±1°
          } else {
              Motor::the().updateMovementSmooth(); // small corrections
          }

          // optional: after 1s, transition back to IDLE
          if (timeInState >= 1000) {
              changeState(IDLE);
          }
      }
      break;
  */ // till here if not incomment the obove
    }
  } else if (controlMode == MANUAL_MODE) {
    if (currentState == ROAD_WARNING || currentState == ROAD_CLOSED ||
        currentState == BRIDGE_LIFTING || currentState == BOAT_PASSAGE) {
      manualStepOpen();
    } else if (currentState == BRIDGE_LOWERING || currentState == BRIDGE_STABILIZING) {
      manualStepClose();
    }
  }

  // Print status periodically (every 500ms)
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint >= 500) {
    printStatus();
    lastPrint = currentTime;
  }

  if (wifiConnected && millis() - lastHeartbeat > 1000) {
    sendStatus();
    lastHeartbeat = millis();
  }

}

void handleManualOpen() {
  Serial.println("[MANUAL] OPEN command received");
  Motor::the().releaseStop();   // allow movement again if stopped!!
  if (currentState == IDLE || currentState == BRIDGE_STABILIZING) {
    changeState(ROAD_WARNING);
  }
}

void handleManualClose() {
  Serial.println("[MANUAL] CLOSE command received");
  Motor::the().releaseStop();   // allow movement again
  if (currentState == BOAT_PASSAGE || currentState == BRIDGE_LIFTING) {
    changeState(BRIDGE_LOWERING);
  }
}

void handleManualStop() {
  Serial.println("[MANUAL] STOP command received - braking motor.");
  Motor::the().emergencyStop();
  setRoadLights(true, false, false);
  setBoatLights(true, false, false);
}

void manualStepOpen() {
  switch (currentState) {
    case IDLE:
    case BRIDGE_STABILIZING:
      changeState(ROAD_WARNING);
      break;

    case ROAD_WARNING:
      if (millis() - stateStartTime >= YELLOW_LIGHT_DURATION)
        changeState(ROAD_CLOSED);
      break;

    case ROAD_CLOSED:
      if (millis() - stateStartTime >= 1000)
        changeState(BRIDGE_LIFTING);
      break;

    case BRIDGE_LIFTING:
      if (Motor::the().isAtTarget())
        changeState(BOAT_PASSAGE);
      break;

    case BOAT_PASSAGE:
      break;

    default:
      break;
  }
}

void manualStepClose() {
  switch (currentState) {
    case BOAT_PASSAGE:
      changeState(BRIDGE_LOWERING);
      break;

    case BRIDGE_LOWERING:
      if (Motor::the().isAtTarget())
        changeState(BRIDGE_STABILIZING);
      break;

    case BRIDGE_STABILIZING:
      if (millis() - stateStartTime >= 4000)
        changeState(IDLE);
      break;

    default:
      break;
  }
}




// ========== STATE CHANGE HANDLER ==========
void changeState(BridgeState newState) {
  Serial.print("\n>>> STATE TRANSITION: ");
  Serial.print(getStateName(currentState));
  Serial.print(" -> ");
  Serial.println(getStateName(newState));
  
  currentState = newState;
  stateStartTime = millis();
  
  // State entry actions
  switch (newState) {
    
    case IDLE:
      Serial.println("[ROAD] Opening to traffic");
      setRoadLights(false, false, true);  // Green
      setBoatLights(true, false, false);         // Red
      Motor::the().setTargetAngle(BRIDGE_DOWN_ANGLE);
      irSensorTriggered = false;  // Reset trigger
      break;
    
    case BOAT_DETECTED:
      Serial.println("[BOAT] Detected on IR sensor - Debouncing...");
      break;
    
    case ROAD_WARNING:
      Serial.println("[ROAD] WARNING - Yellow lights activated");
      setRoadLights(false, true, false);  // Yellow
      setBoatLights(true, false, false); // Red
      break;
    
    case ROAD_CLOSED:
      Serial.println("[ROAD] CLOSED - Red lights activated");
      setRoadLights(true, false, false);  // Red
      setBoatLights(true, false, false);  // Red
      break;
    
    case BRIDGE_LIFTING:
      Serial.println("[MOTOR] Lifting bridge...");
      Motor::the().setTargetAngle(BRIDGE_UP_ANGLE);
      setRoadLights(true, false, false);  // Red
      setBoatLights(true, false, false);  // Red
      break;
    
    case BOAT_PASSAGE:
      Serial.println("[BOAT] Passage authorized - Green lights for boats");
      setRoadLights(true, false, false);  // Red
      setBoatLights(false, false, true);    // Green
      break;
    
    case BRIDGE_LOWERING:
      Serial.println("[MOTOR] Lowering bridge...");
      Motor::the().setTargetAngle(BRIDGE_DOWN_ANGLE);
      setRoadLights(true, false, false);  // Red
      setBoatLights(false, true, false);  //Yellow
      break;
    
    case BRIDGE_STABILIZING:
      Serial.println("[MOTOR] Bridge stabilizing...");
      setRoadLights(true, false, false);  // Red
      setBoatLights(true, false, false);  // Red
      break;
  }
}

// IR SENSOR FUNCTIONS ==========
void checkIRSensor() {
  // Check simulation button first (active LOW when pressed)
 // bool buttonPressed = (digitalRead(SIMULATE_BUTTON_PIN) == LOW);
  
  // IR sensor typically LOW when object detected (if using active-low)
  // Adjust logic if your sensor is active-high
  bool sensorActive = (digitalRead(IR_SENSOR_PIN) == LOW);
  
  // Trigger if either button is pressed OR sensor is active
  // bool shouldTrigger = buttonPressed || sensorActive;
  bool shouldTrigger =  sensorActive;
  
  if (shouldTrigger && !irSensorTriggered) {
    unsigned long currentTime = millis();
    if (currentTime - lastIRTrigger > DEBOUNCE_TIME) {
      irSensorTriggered = true;
      lastIRTrigger = currentTime;
      //if (buttonPressed) {
        //Serial.println("[BUTTON] *** SIMULATION BUTTON PRESSED - Boat detected! ***");
      //} else {
        Serial.println("[SENSOR] IR triggered!");
      //}
    }
  }
}

//float d1, d2;
//us4.readDistances(d1, d2); //ERIN!! MAYBE delete

// Ultrasonic sensor function
void checkUltrasonicSensors(Ultrasonic4 &us4) {
    float d1, d2;
    us4.readDistances(d1, d2);
    

    // If any sensor detects an object closer than the threshold
    bool boatDetected = false;
    if ((d1 > 0 && d1 <= BOAT_DETECT_DISTANCE) ||
        (d2 > 0 && d2 <= BOAT_DETECT_DISTANCE)) {
       irSensorTriggered = true;
        boatDetected = true;
    }
/* //erin!! change depending on how many sensors
       // Only check the first 3 sensors
    bool boatDetected = false;
    if ((d1 > 0 && d1 <= BOAT_DETECT_DISTANCE) ||
        (d2 > 0 && d2 <= BOAT_DETECT_DISTANCE) ||
        (d3 > 0 && d3 <= BOAT_DETECT_DISTANCE)) {
        boatDetected = true;
    }
*/
    // Debounce logic
    unsigned long currentTime = millis();
    if (boatDetected && !ultraTriggered) {
        if (currentTime - lastUltraTrigger > ULTRA_DEBOUNCE_TIME) {
            ultraTriggered = true;
            lastUltraTrigger = currentTime;
            Serial.println("[ULTRASONIC] Boat detected!");
        }
    } else if (!boatDetected) {
        ultraTriggered = false; // reset trigger when no boat is detected
    }
}



// ========== LIGHT CONTROL FUNCTIONS ==========
void setRoadLights(bool red, bool yellow, bool green) {
  // Set both road light sets identically
  digitalWrite(ROAD_LIGHT_1_RED, red ? HIGH : LOW);
  digitalWrite(ROAD_LIGHT_1_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(ROAD_LIGHT_1_GREEN, green ? HIGH : LOW);
  
  digitalWrite(ROAD_LIGHT_2_RED, red ? HIGH : LOW);
  digitalWrite(ROAD_LIGHT_2_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(ROAD_LIGHT_2_GREEN, green ? HIGH : LOW);
}

void setBoatLights(bool red, bool yellow, bool green) {
  // Set both boat light sets identically
  digitalWrite(BOAT_LIGHT_3_RED, red ? HIGH : LOW);
  digitalWrite(BOAT_LIGHT_3_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(BOAT_LIGHT_3_GREEN, green ? HIGH : LOW);

  digitalWrite(BOAT_LIGHT_4_RED, red ? HIGH : LOW);
  digitalWrite(BOAT_LIGHT_4_YELLOW, yellow ? HIGH : LOW);
  digitalWrite(BOAT_LIGHT_4_GREEN, green ? HIGH : LOW);
}

// ========== STATUS DISPLAY ==========
void printStatus() {
  Serial.print("[STATUS] State: ");
  Serial.print(getStateName(currentState));
  Serial.print(" | Time: ");
  Serial.print((millis() - stateStartTime) / 1000.0, 1);
  Serial.print("s | Angle: ");
  Serial.print(Motor::the().getCurrentAngle(), 1);
  Serial.print("° (Target: ");
  Serial.print(Motor::the().getTargetAngle(), 1);
  Serial.print("°) | RPM: ");
  Serial.print(Motor::the().getCurrentRPM(), 1);
 // Serial.print(" | S1: ");
 // Serial.println(d1);
 Serial.print(" | S2: ");
 //// Serial.println(d2);
  Serial.print(" |sensor Target: ");
  Serial.println("20");
 
}

void sendStatus() {
  if (!wifiConnected) return;
  String stateStr = getStateName(currentState);
  String modeStr = (controlMode == AUTO_MODE) ? "AUTO" : "OVERRIDE";

  String msg = "OK STATE=" + stateStr + " MODE=" + modeStr;
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(msg);
  Udp.endPacket();
}


const char* getStateName(BridgeState state) {
  switch (state) {
    case IDLE: return "IDLE";
    case BOAT_DETECTED: return "BOAT_DETECTED";
    case ROAD_WARNING: return "ROAD_WARNING";
    case ROAD_CLOSED: return "ROAD_CLOSED";
    case BRIDGE_LIFTING: return "BRIDGE_LIFTING";
    case BOAT_PASSAGE: return "BOAT_PASSAGE";
    case BRIDGE_LOWERING: return "BRIDGE_LOWERING";
    case BRIDGE_STABILIZING: return "BRIDGE_STABILIZING";
    default: return "UNKNOWN";
  }
}

// ========== MOTOR TEST FUNCTION ==========
void testMotor() {
  Serial.println("\n========== MOTOR TEST MODE ==========");
  Serial.println("This will test the motor by moving it 45 degrees");
  Serial.println("Watch the motor and Serial output...\n");
  
  // Save current state
  BridgeState savedState = currentState;
  
  // Test sequence
  Motor::the().setTargetAngle(0);
  delay(100);
  
  Serial.println("[TEST] Setting target to 45 degrees...");
  Motor::the().setTargetAngle(45.0);
  
  unsigned long testStart = millis();
  unsigned long lastPrint = 0;
  
  while (millis() - testStart < 10000) {  // 10 second timeout
    Motor::the().updateMovement();
    
    if (millis() - lastPrint >= 200) {
      Serial.print("[TEST] Angle: ");
      Serial.print(Motor::the().getCurrentAngle(), 2);
      Serial.print("° | Target: ");
      Serial.print(Motor::the().getTargetAngle(), 2);
      Serial.print("° | RPM: ");
      Serial.print(Motor::the().getCurrentRPM(), 2);
      Serial.print(" | Error: ");
      Serial.print(Motor::the().getTargetAngle() - Motor::the().getCurrentAngle(), 2);
      Serial.println("°");
      lastPrint = millis();
    }
    
    if (Motor::the().isAtTarget()) {
      Serial.println("\n[TEST] ✓ Target reached!");
      break;
    }
  }
  
  if (!Motor::the().isAtTarget()) {
    Serial.println("\n[TEST] ✗ Timeout - motor may be stalled or blocked");
    Serial.println("Possible issues:");
    Serial.println("  1. PWM too low (check MIN_PWM in Motor.cpp)");
    Serial.println("  2. Motor wiring incorrect");
    Serial.println("  3. Mechanical binding");
    Serial.println("  4. Insufficient power supply");
    Serial.println("  5. Encoder not connected properly");
  }
  
  delay(1000);
  Serial.println("\n[TEST] Returning to 0 degrees...");
  Motor::the().setTargetAngle(0);
  
  testStart = millis();
  while (millis() - testStart < 10000) {
    Motor::the().updateMovement();
    if (Motor::the().isAtTarget()) {
      Serial.println("[TEST] ✓ Returned to zero");
      break;
    }
  }
  
  Serial.println("\n========== TEST COMPLETE ==========");
  Serial.println("Type 't' to test again, 'e' for encoder test, or 'r' to reset");
  Serial.println("Or press button to start normal sequence\n");
  
  // Restore state
  changeState(savedState);
}

// ========== ENCODER TEST FUNCTION ==========
void testEncoder() {
  Serial.println("\n========== ENCODER TEST MODE ==========");
  Serial.println("Testing raw encoder pin readings while motor runs...\n");
  
  // First, test raw pin readings WITH MOTOR RUNNING
  Serial.println("--- RAW PIN TEST WITH MOTOR (5 seconds) ---");
  Serial.println("Motor will spin forward, watch for A and B changes...\n");
  
  // Set motor to spin forward at slow speed
  Motor::the().setTargetRPM(10.0);  // Slow speed
  Motor::the().setTargetAngle(1000); // Large target so it keeps spinning
  
  unsigned long testStart = millis();
  int lastA = -1, lastB = -1;
  int changeCount = 0;
  
  while (millis() - testStart < 5000) {
    Motor::the().updateVelocity(); // Keep motor running
    
    int pinA = digitalRead(MOTOR_ENCODER);  // Pin 34
    int pinB = digitalRead(MOTOR_DIRECTION); // Pin 35
    
    if (pinA != lastA || pinB != lastB) {
      Serial.print("A: ");
      Serial.print(pinA);
      Serial.print(" B: ");
      Serial.print(pinB);
      Serial.print(" (Change #");
      Serial.print(++changeCount);
      Serial.println(")");
      lastA = pinA;
      lastB = pinB;
    }
    delay(1);
  }
  
  // Stop motor
  Motor::the().setTargetRPM(0);
  
  Serial.println("\n--- RAW PIN TEST RESULTS ---");
  if (changeCount == 0) {
    Serial.println("ERROR: No encoder changes detected while motor spinning!");
    Serial.println("Possible issues:");
    Serial.println("  1. Encoder not connected to pins 34 and 35");
    Serial.println("  2. Encoder not powered (check VCC and GND)");
    Serial.println("  3. Encoder broken or incompatible");
    Serial.println("  4. Motor spinning but encoder not attached to shaft");
    Serial.println("\nCheck your wiring:");
    Serial.println("  Encoder A  -> GPIO 34");
    Serial.println("  Encoder B  -> GPIO 35");
    Serial.println("  Encoder V+ -> 3.3V or 5V");
    Serial.println("  Encoder G  -> GND");
    Serial.println("\nWhat type of encoder do you have?");
    Serial.println("  - Built into motor?");
    Serial.println("  - Separate encoder disc?");
  } else {
    Serial.print("SUCCESS: Detected ");
    Serial.print(changeCount);
    Serial.println(" encoder transitions while motor spinning!");
    Serial.println("\nNow testing angle calculation...\n");
    
    // Now test if angle updates
    Serial.println("--- ANGLE TEST WITH MOTOR (5 seconds) ---");
    
    Motor::the().setTargetRPM(10.0);
    Motor::the().setTargetAngle(1000);
    
    double lastAngle = Motor::the().getCurrentAngle();
    testStart = millis();
    bool angleChanged = false;
    
    while (millis() - testStart < 5000) {
      Motor::the().updateVelocity();
      delay(100);
      
      double currentAngle = Motor::the().getCurrentAngle();
      double rpm = Motor::the().getCurrentRPM();
      
      Serial.print("Angle: ");
      Serial.print(currentAngle, 3);
      Serial.print("° | RPM: ");
      Serial.print(rpm, 2);
      Serial.print(" | Change: ");
      Serial.print(currentAngle - lastAngle, 3);
      Serial.println("°");
      
      if (abs(currentAngle - lastAngle) > 0.001) {
        angleChanged = true;
      }
      lastAngle = currentAngle;
    }
    
    // Stop motor
    Motor::the().setTargetRPM(0);
    
    Serial.println("\n--- ANGLE TEST RESULTS ---");
    if (!angleChanged) {
      Serial.println("ERROR: Pins detected but angle not updating!");
      Serial.println("This means the interrupt handler isn't working.");
      Serial.println("Possible issues:");
      Serial.println("  1. Pins 34/35 may not support interrupts on your ESP32");
      Serial.println("  2. Interrupt initialization failed in Motor.cpp");
      Serial.println("  3. ISR is in wrong memory (should be IRAM_ATTR)");
      Serial.println("  4. Encoder pulses too fast for interrupt to catch");
    } else {
      Serial.println("SUCCESS: Encoder is working correctly!");
    }
  }
  
  // Make sure motor stops
  Motor::the().setTargetRPM(0);
  delay(500);
  
  Serial.println("\n========== ENCODER TEST COMPLETE ==========");
  Serial.println("Type 'e' to test again, 'r' to reset\n");
}