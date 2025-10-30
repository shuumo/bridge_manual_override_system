#pragma once

class Motor {
public:
    static Motor &the() {
        static Motor instance;
        return instance;
    }

    void begin();
    void reset() {
        zeroTicks();
        setTargetAngle(0);
        currentPWM = 0;
        lastControlMs = 0;
    }

    double getTargetAngle();
    double getTargetRPM();
    void setTargetAngle(double angleDeg);

    bool isAtTarget();
    bool isAtAngle(double angleDeg);
    
    double getCurrentAngle();
    double getCurrentRPM();

    void updateMovement();
    void updateMovementSmooth(); //Erin!!
    void updateVelocity();
    void setTargetRPM(double rpm);

    void setGearRatio(double ratio);
    double getGearRatio() const;

    void emergencyStop(); 
    void releaseStop();
    bool isStopped() const { return stopped; }



private:
    bool stopped = false;
    Motor() = default;
    Motor(const Motor &) = delete;
    Motor &operator=(const Motor &) = delete;

    void driveForward(int pwm);
    void driveBackward(int pwm);
    void brake();

    void zeroTicks();
    long getTicks();

    


private:
    int currentPWM = 0;
    double currentRPM = 0;
    double targetAngleDeg = 0;
    double targetRPM = 10.0; //erin!! if change volt change this
    unsigned long lastControlMs = 0;
    double gearRatio = 1.0;

    unsigned long lastUs = 0;
    long lastTicks = 0;
    double integrator = 0.0;
    double lastMeasuredRPM = 0.0;
    double lastDeriv = 0.0;
};