#include "Motor.h"
#include "defs.h"

#include <Arduino.h>
#include <driver/ledc.h>



namespace MotorImpl {
	constexpr int MAX_PWM = (1 << PWM_RESOLUTION) - 1;
	constexpr int MIN_PWM = 80; //This will change the speed of the motor! Can reasonably go a little higher for more speed but it's slow atm for safety
	constexpr double DEAD_BAND_DEG				= 2.0;

	constexpr double COUNTS_PER_MOTOR_REV = 2800;
	constexpr double ANGLE_PER_TICK_DEG = 360.0 / COUNTS_PER_MOTOR_REV;

	constexpr double Kp = 3.0;         // proportional (try raising)
    constexpr double Ki = 0.80;        // integral (start small)
    constexpr double Kd = 0.06;        // derivative (small damping)
    constexpr double Kff = 0.08;       // feedforward (PWM per RPM) - tune by experiment
    constexpr double DERIV_TAU = 0.02; // derivative low-pass time constant (s)

	volatile int64_t g_encoderTicks = 0;
	static volatile uint8_t g_lastAB = 0;
	static const int8_t QUAD_LUT[16] = {
		0 , +1, -1,  0,
		-1,  0,  0, +1,
		+1,  0,  0, -1,
		0 , -1, +1,  0,
	};

	// ISR: fast, in IRAM so it's safe on ESP32
	static void IRAM_ATTR quadISR() {
		uint8_t a = (uint8_t)gpio_get_level((gpio_num_t)MOTOR_ENCODER);
		uint8_t b = (uint8_t)gpio_get_level((gpio_num_t)MOTOR_DIRECTION);
		uint8_t s = (a << 1) | b;
		uint8_t idx = (g_lastAB << 2) | s;
		g_encoderTicks += QUAD_LUT[idx];
		g_lastAB = s;
	}

	static inline double ticksToDegrees(long ticks) {
		return (double)ticks * ANGLE_PER_TICK_DEG;
	}
}

void Motor::emergencyStop() {
    stopped = true;

    // Immediately stop PWM and apply brake
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH);

    digitalWrite(MOTOR_1, HIGH);
    digitalWrite(MOTOR_2, HIGH);
    currentPWM = 0;
    targetRPM = 0;
    integrator = 0;
    Serial.println("[MOTOR] EMERGENCY STOP triggered.");
}

void Motor::releaseStop() {
    if (stopped) {
        stopped = false;
        Serial.println("[MOTOR] Stop released — control resumed.");
    }
}



void Motor::begin() {
	setTargetAngle(0);

	ledc_timer_config_t pwmTimer = {
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.duty_resolution = (ledc_timer_bit_t)PWM_RESOLUTION,
			.timer_num = MOTOR_TIMER,
			.freq_hz = PWM_FREQ,
			.clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwmTimer);

	ledc_channel_config_t motorPWMChannel = {
			.gpio_num = MOTOR_PWM,
			.speed_mode = LEDC_LOW_SPEED_MODE,
			.channel = MOTOR_PWM_CH,
			.intr_type = LEDC_INTR_DISABLE,
			.timer_sel = MOTOR_TIMER,
			.duty = 0,
			.hpoint = 0
	};
	ledc_channel_config(&motorPWMChannel);

	pinMode(MOTOR_1, OUTPUT);
	pinMode(MOTOR_2, OUTPUT);
	pinMode(MOTOR_ENCODER, INPUT_PULLUP);
	pinMode(MOTOR_DIRECTION, INPUT_PULLUP);

	MotorImpl::g_lastAB = (gpio_get_level((gpio_num_t)MOTOR_ENCODER) << 1) | gpio_get_level((gpio_num_t)MOTOR_DIRECTION);
	// init last state from the pins
	//MotorImpl::g_lastAB = (gpio_get_level((gpio_num_t)MOTOR_ENCODER) << 1) | ((gpio_num_t)MOTOR_DIRECTION);

	// attach both pins to the same ISR (CHANGE)
	attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER), MotorImpl::quadISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(MOTOR_DIRECTION), MotorImpl::quadISR, CHANGE);
}

double Motor::getTargetAngle() { 
	return targetAngleDeg / gearRatio; 
}

double Motor::getTargetRPM() { 
	return targetRPM;
}

void Motor::setTargetAngle(double angleDeg) {
    targetAngleDeg = angleDeg * gearRatio;
}

double Motor::getCurrentAngle() {
    long ticks = getTicks();
    double motorAngle = MotorImpl::ticksToDegrees(ticks);
    return motorAngle / gearRatio;
}

bool Motor::isAtTarget() {
	return isAtAngle(getTargetAngle());
}

bool Motor::isAtAngle(double angleDeg) {
	const double currentAngle = getCurrentAngle();
	return fabs(angleDeg - currentAngle) <= MotorImpl::DEAD_BAND_DEG;
}

void Motor::updateMovement() {
    if (stopped) {
        brake();
        return;
    }

    double err = targetAngleDeg - MotorImpl::ticksToDegrees(getTicks());
    if (fabs(err) <= MotorImpl::DEAD_BAND_DEG) {
		unsigned long nowUs = micros();
		unsigned long dtUs = nowUs - lastUs;
		if (dtUs > 2000) lastUs = nowUs;

        brake();
        return;
    }

    if (err > 0) setTargetRPM(fabs(targetRPM));
    else setTargetRPM(-fabs(targetRPM));

    updateVelocity();
}

void Motor::setTargetRPM(double rpm) {
    targetRPM = rpm;
}

void Motor::updateVelocity() {
    unsigned long nowUs = micros();
    unsigned long dtUs = nowUs - lastUs;
    if (dtUs < 2000) return;
    lastUs = nowUs;

    long ticks = getTicks();
    long dTicks = ticks - lastTicks;
    lastTicks = ticks;

    double revs = (double)dTicks / MotorImpl::COUNTS_PER_MOTOR_REV;
    double rpmMeasured = 0.0;
    if (dtUs > 0) rpmMeasured = revs * (60000000.0 / (double)dtUs);
    currentRPM = rpmMeasured;
    double dt = dtUs / 1e6;

    double err = targetRPM - rpmMeasured;
    double ff = MotorImpl::Kff * targetRPM;
    double P = MotorImpl::Kp * err;

    integrator += err * dt;
    double out_unclamped = ff + P; // without I and D
    double maxI = (MotorImpl::MAX_PWM - out_unclamped);
    double minI = -MotorImpl::MAX_PWM - out_unclamped;
    if (MotorImpl::Ki > 0) {
        integrator = fmin(integrator, maxI / MotorImpl::Ki);
        integrator = fmax(integrator, minI / MotorImpl::Ki);
    }
    double I = MotorImpl::Ki * integrator;

    double meas_deriv = (rpmMeasured - lastMeasuredRPM) / dt;
    lastMeasuredRPM = rpmMeasured;
    double alpha = dt / (MotorImpl::DERIV_TAU + dt);
    lastDeriv = lastDeriv + alpha * ((-meas_deriv) - lastDeriv);
    double D = MotorImpl::Kd * lastDeriv;

    double out = ff + P + I + D;

    int pwm = (int) fabs(out);
    pwm = constrain(pwm, MotorImpl::MIN_PWM, MotorImpl::MAX_PWM);

    if (targetRPM > 0)       driveForward(pwm);
    else if (targetRPM < 0)  driveBackward(pwm);
    else                     brake();
}

void Motor::driveForward(int pwmVal) {
	if (pwmVal < MotorImpl::MIN_PWM) pwmVal = MotorImpl::MIN_PWM;
	if (pwmVal > MotorImpl::MAX_PWM) pwmVal = MotorImpl::MAX_PWM;
	if (currentPWM != pwmVal) {
		ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH, pwmVal);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH);
		currentPWM = pwmVal;
	}

	digitalWrite(MOTOR_1, HIGH);
	digitalWrite(MOTOR_2, LOW);
}

void Motor::driveBackward(int pwmVal) {
	if (pwmVal < MotorImpl::MIN_PWM) pwmVal = MotorImpl::MIN_PWM;
	if (pwmVal > MotorImpl::MAX_PWM) pwmVal = MotorImpl::MAX_PWM;
	if (currentPWM != pwmVal) {
		ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH, pwmVal);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH);
		currentPWM = pwmVal;
	}

	digitalWrite(MOTOR_1, LOW);
	digitalWrite(MOTOR_2, HIGH);
}

void Motor::brake() {
	if (currentPWM != MotorImpl::MAX_PWM) {
		ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH, MotorImpl::MAX_PWM);
		ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CH);
		currentPWM = MotorImpl::MAX_PWM;
	}

	digitalWrite(MOTOR_1, HIGH);
	digitalWrite(MOTOR_2, HIGH);
}

double Motor::getCurrentRPM() {
	return currentRPM;
}

long Motor::getTicks() {
    noInterrupts();
    long t = MotorImpl::g_encoderTicks;
    interrupts();
    return t;
}

void Motor::zeroTicks() {
	noInterrupts();
	MotorImpl::g_encoderTicks = 0;
	interrupts();
}

void Motor::setGearRatio(double ratio) {
    if (ratio <= 0) ratio = 1.0;
    gearRatio = ratio;
}

double Motor::getGearRatio() const {
    return gearRatio;
}


// erin!! added to reduce the jerk when the bridge closes
  /*
void Motor::updateMovementSmooth() {
    double currentAngle = getCurrentAngle();
    double targetAngle = getTargetAngle();
    double errorDeg = targetAngle - currentAngle;
    double distance = fabs(errorDeg);

    constexpr double SLOWDOWN_ZONE_DEG = 20.0;   // start slowing 20° before target
    constexpr double MIN_RPM = 5.0;              // prevent stalling

    // If we're close enough, stop
    if (distance <= MotorImpl::DEAD_BAND_DEG) {
        brake();
        return;
    }

    // Scale target speed based on distance remaining
    double rpmCommand = fabs(targetRPM);
    if (distance < SLOWDOWN_ZONE_DEG) {
        // Linearly scale down
        rpmCommand *= (distance / SLOWDOWN_ZONE_DEG);

        // Or use smooth curve:
        // rpmCommand *= pow(distance / SLOWDOWN_ZONE_DEG, 2); //try this if other one isnt smooth enough

        rpmCommand = fmax(rpmCommand, MIN_RPM);
    }

    // Set direction
    if (errorDeg > 0) setTargetRPM(rpmCommand);
    else setTargetRPM(-rpmCommand);

    updateVelocity();
}



  */
 /*
	void Motor::updateMovementSmooth() {
    double currentAngle = getCurrentAngle();
    double targetAngle  = getTargetAngle();
    double errorDeg     = targetAngle - currentAngle;
    double distance     = fabs(errorDeg);

    // ---- Tuning constants ----
    constexpr double SLOWDOWN_ZONE_DEG = 12.0;   // start slowing before target
    constexpr double MIN_RPM           = 10.0;   // ensures enough torque to start
    constexpr double MIN_SCALE         = 0.4;    // don't scale below 40% speed
    constexpr double STOP_BAND_DEG     = MotorImpl::DEAD_BAND_DEG;  // ~2 degrees

    // ---- Stop if within very close range ----
    if (distance <= STOP_BAND_DEG) {
        brake();                 // fully stop the motor (no drift)
        integrator = 0.0;        // reset PID integrator to prevent creeping
        currentPWM = 0;
        return;
    }

    // ---- Base speed ----
    double rpmCommand = fabs(targetRPM);
    if (rpmCommand < MIN_RPM)
        rpmCommand = MIN_RPM;    // baseline torque so motor starts instantly

    // ---- Smooth deceleration near target ----
    if (distance < SLOWDOWN_ZONE_DEG) {
        double scale = distance / SLOWDOWN_ZONE_DEG;  // 0–1 range
        scale = pow(scale, 1.2);                      // smooth slowdown curve
        scale = fmax(scale, MIN_SCALE);
        rpmCommand *= scale;
    }

    rpmCommand = fmax(rpmCommand, MIN_RPM);

    // ---- Determine direction ----
    if (errorDeg > 0)
        setTargetRPM(rpmCommand);    // move forward
    else
        setTargetRPM(-rpmCommand);   // move backward

    // ---- Run PID control loop ----
    updateVelocity();  // handles P, I, D, and feedforward internally
}
*/
void Motor::updateMovementSmooth() {
    if (stopped) {
        brake();
        return;
    }

    // === Tunable parameters ===
    const double DEAD_BAND_DEG = 0.5;    // stop within ±0.5°
    const double SLOWDOWN_ZONE_DEG = 10.0; // start slowing 10° before target
    const double MIN_RPM = 8.0;          // prevent stalling
    const double MAX_RPM = 80.0;         // top speed for smooth control

    // === Current and target angles ===
    double currentAngle = getCurrentAngle();
    double targetAngle  = getTargetAngle();
    double errDeg = targetAngle - currentAngle;
    double distance = fabs(errDeg);

    // === Stop if close enough ===
    if (distance <= DEAD_BAND_DEG) {
        brake();
        return;
    }

    // === Base RPM command proportional to distance ===
    double rpmCommand = MAX_RPM;

    // Slow down when near target
    if (distance < SLOWDOWN_ZONE_DEG) {
        double scale = distance / SLOWDOWN_ZONE_DEG;
        scale = pow(scale, 1.3);   // exponential taper
        rpmCommand *= fmax(scale, 0.3);  // don’t go below 30% of MAX
    }

    // Keep minimum RPM to prevent stalls
    rpmCommand = fmax(rpmCommand, MIN_RPM);

    // === Set direction ===
    if (errDeg > 0)
        setTargetRPM(rpmCommand);
    else
        setTargetRPM(-rpmCommand);

    // === Run the PID control loop ===
    updateVelocity();
}
