
//motor pins
#define MOTOR_1 27
#define MOTOR_2 26
#define MOTOR_DIRECTION 35 //Was 35 THIS CABLE IS YELLOW
#define MOTOR_ENCODER 34  //Was 34 THIS CABLE IS WHITE
#define MOTOR_PWM 14

#define MOTOR_PWM_CH LEDC_CHANNEL_0
#define MOTOR_TIMER LEDC_TIMER_0
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8


// IR Sensor (BREAK BEAM)
#define IR_SENSOR_PIN 2  // BREAK BEAM pin
// #define SIMULATE_BUTTON_PIN 4  // ENA pin - press to simulate boat detection

// Road Traffic Lights Set 1 (Bridge Left)
#define ROAD_LIGHT_1_RED     23
#define ROAD_LIGHT_1_YELLOW  15
#define ROAD_LIGHT_1_GREEN   25

// Road Traffic Lights Set 2 (Bridge Right)
#define ROAD_LIGHT_2_RED     23
#define ROAD_LIGHT_2_YELLOW  15
#define ROAD_LIGHT_2_GREEN   25

// Boat Traffic Lights Set 3 (Ship Left)
#define BOAT_LIGHT_3_RED     17
#define BOAT_LIGHT_3_YELLOW  16
#define BOAT_LIGHT_3_GREEN   22 

// Boat Traffic Lights Set 4 (Ship Right 2)
#define BOAT_LIGHT_4_RED     17
#define BOAT_LIGHT_4_YELLOW  16
#define BOAT_LIGHT_4_GREEN   22

//Ultrasonic pins
// Ultrasonic Sensors (HC-SR04) — no conflicts
#define ULTRA_TRIG1 4
#define ULTRA_ECHO1 18  // input-only pin

#define ULTRA_TRIG2 5
#define ULTRA_ECHO2 19  // input-only pin


// Adjust this threshold for your boat detection distance (in cm)
#define BOAT_DETECT_DISTANCE 20 //  ERIN!! CHANGE T CHANGE BOAT DETECT DIST
#define ULTRA_DEBOUNCE_TIME 500  // ms


