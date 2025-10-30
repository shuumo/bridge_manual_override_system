#include "Ultrasonic4.h"

Ultrasonic4::Ultrasonic4(int trig1, int echo1,
                         int trig2, int echo2) {
    _trig[0] = trig1; _echo[0] = echo1;
    _trig[1] = trig2; _echo[1] = echo2;

}

void Ultrasonic4::begin() {
    for (int i = 0; i < 2; i++) {
        pinMode(_trig[i], OUTPUT);
        pinMode(_echo[i], INPUT);  // Use voltage divider on ESP32
        digitalWrite(_trig[i], LOW);
    }
}

void Ultrasonic4::readDistances(float &d1, float &d2) {
    d1 = readDistance(_trig[0], _echo[0]);
    d2 = readDistance(_trig[1], _echo[1]);
   
}

float Ultrasonic4::readDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 60000); // 60ms max
    if (duration == 0) return -1;  // Out of range
    return duration * 0.034 / 2.0; // convert to cm
}
