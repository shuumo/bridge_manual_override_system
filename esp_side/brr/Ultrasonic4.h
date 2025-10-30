#pragma once

#include <Arduino.h>

class Ultrasonic4 {
public:
    Ultrasonic4(int trig1, int echo1,
                int trig2, int echo2);

    void begin();                // Initialize pins
    void readDistances(float &d1, float &d2); // Read all 2 sensors

private:
    int _trig[2];
    int _echo[2];
    float readDistance(int trigPin, int echoPin);
};


