/**
 * Prototype 2 - DC Motors with Encoders and Cascaded PI Speed Control
 * 
 * This firmware implements a cascaded control architecture:
 *   - Outer loop: Guidance (vision-based, to be implemented externally)
 *   - Inner loop: Speed regulation with PI control per wheel
 * 
 * Features:
 *   - Quadrature encoder decoding for velocity measurement
 *   - PI speed control with anti-windup protection
 *   - Timer-based interrupt for consistent sample rate
 *   - Battery voltage compensation (optional)
 * 
 * Hardware Configuration:
 *   - 2x DC motors with gearbox (1:24 ratio typical)
 *   - 2x Incremental quadrature encoders
 *   - H-bridge motor driver (e.g., L298N, TB6612)
 *   - Arduino with sufficient interrupt pins
 * 
 * Author: Marc Duboc
 * License: MIT
 */

#include <FlexiTimer2.h>

//=============================================================================
// CONFIGURATION
//=============================================================================

// --- Encoder Pins ---
// Must be interrupt-capable pins on your Arduino board
#define ENC_L_A    2    // Left encoder channel A
#define ENC_L_B    3    // Left encoder channel B
#define ENC_R_A    18   // Right encoder channel A (Mega pin)
#define ENC_R_B    19   // Right encoder channel B (Mega pin)

// --- Motor Driver Pins ---
#define MOT_L_DIR  4    // Left motor direction pin
#define MOT_L_PWM  5    // Left motor PWM pin
#define MOT_R_DIR  9    // Right motor direction pin
#define MOT_R_PWM  8    // Right motor PWM pin

// --- Encoder Configuration ---
// Ticks per revolution = CPR * 4 (quadrature) * gear_ratio
// Example: 17 CPR encoder, 4x decoding, 24:1 gearbox = 17*4*24 = 1632
const float TICKS_PER_REV = 1632.0f;

// --- Physical Parameters ---
const float WHEEL_RADIUS = 0.032f;   // Wheel radius [m]
const float WHEEL_BASE = 0.12f;      // Distance between wheels [m]
const float BATTERY_VOLT = 11.1f;    // Nominal battery voltage [V]

// --- PI Controller Gains ---
// Tuned for first-order motor model: G(s) = 4 / (1 + 0.035s)
// Resulting margins: 66° phase margin, infinite gain margin
volatile float KP = 0.29f;           // Proportional gain [V/(rad/s)]
volatile float KI = 8.93f;           // Integral gain [V/(rad/s·s)]

// --- Anti-Windup Configuration ---
const float INTEGRAL_MAX = 2.0f;     // Maximum integral term [V]
                                     // Prevents integral wind-up during saturation

// --- Timing ---
const uint16_t SAMPLE_MS = 10;       // Control loop sample period [ms] (100 Hz)
const uint16_t TIMEOUT_MS = 1000;    // Reference timeout [ms] - stop if no update

// --- Debug ---
const uint16_t PRINT_INTERVAL_MS = 500;  // Serial print interval [ms]

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Encoder state (updated in ISR)
volatile long ticksL = 0;
volatile long ticksR = 0;

// Velocity measurements [rad/s]
volatile float omegaL = 0.0f;
volatile float omegaR = 0.0f;

// Velocity setpoints [rad/s]
volatile float vrefL = 0.0f;
volatile float vrefR = 0.0f;

// PI controller integral terms [V]
volatile float integralL = 0.0f;
volatile float integralR = 0.0f;

// Timing for reference timeout
volatile unsigned long lastRefUpdateMs = 0;

//=============================================================================
// MOTOR CONTROL FUNCTIONS
//=============================================================================

/**
 * Clamp a float value to a specified range.
 */
static inline float clampf(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

/**
 * Write voltage command to motor with direction handling.
 * 
 * @param dirPin Direction pin for H-bridge
 * @param pwmPin PWM pin for H-bridge
 * @param cmd Voltage command [-BATTERY_VOLT, +BATTERY_VOLT]
 */
static inline void motorWrite(int dirPin, int pwmPin, float cmd) {
    // Saturate command to battery voltage
    cmd = clampf(cmd, -BATTERY_VOLT, BATTERY_VOLT);
    
    // Calculate PWM duty cycle (0-255)
    int pwm = (int)(255.0f * fabsf(cmd) / BATTERY_VOLT);
    pwm = min(pwm, 255);
    
    // Set direction and apply PWM
    if (cmd >= 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
    analogWrite(pwmPin, pwm);
}

/**
 * Stop both motors immediately.
 */
void stopMotors() {
    analogWrite(MOT_L_PWM, 0);
    analogWrite(MOT_R_PWM, 0);
    integralL = 0.0f;
    integralR = 0.0f;
}

//=============================================================================
// ENCODER INTERRUPT SERVICE ROUTINES
//=============================================================================

// Quadrature decoding: increment/decrement based on channel states
void encL_A() {
    if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) {
        ticksL--;
    } else {
        ticksL++;
    }
}

void encL_B() {
    if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) {
        ticksL++;
    } else {
        ticksL--;
    }
}

void encR_A() {
    if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) {
        ticksR--;
    } else {
        ticksR++;
    }
}

void encR_B() {
    if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) {
        ticksR++;
    } else {
        ticksR--;
    }
}

//=============================================================================
// CONTROL LOOP (Timer ISR)
//=============================================================================

/**
 * PI speed control loop - called at SAMPLE_MS interval.
 * 
 * Implements:
 *   - Velocity calculation from encoder ticks
 *   - PI control with anti-windup
 *   - Motor voltage output
 */
void controlISR() {
    const float dt = SAMPLE_MS / 1000.0f;  // Sample time in seconds
    
    // Read and reset encoder ticks (atomic operation in ISR)
    long deltaL = ticksL;
    long deltaR = ticksR;
    ticksL = 0;
    ticksR = 0;
    
    // Calculate angular velocities [rad/s]
    // omega = (2*PI * ticks / ticks_per_rev) / dt
    omegaL = (2.0f * PI * (float)deltaL / TICKS_PER_REV) / dt;
    omegaR = (2.0f * PI * (float)deltaR / TICKS_PER_REV) / dt;
    
    // Safety timeout check
    unsigned long now = millis();
    if (now - lastRefUpdateMs > TIMEOUT_MS) {
        // No reference update received - stop motors
        stopMotors();
        return;
    }
    
    // Calculate velocity errors
    float errorL = vrefL - omegaL;
    float errorR = vrefR - omegaR;
    
    // PI control with anti-windup
    // u = Kp * e + Ki * integral(e)
    float uL = KP * errorL + integralL;
    float uR = KP * errorR + integralR;
    
    // Update integral terms with clamping (anti-windup)
    integralL += KI * dt * errorL;
    integralR += KI * dt * errorR;
    integralL = clampf(integralL, -INTEGRAL_MAX, INTEGRAL_MAX);
    integralR = clampf(integralR, -INTEGRAL_MAX, INTEGRAL_MAX);
    
    // Apply motor commands
    motorWrite(MOT_L_DIR, MOT_L_PWM, uL);
    motorWrite(MOT_R_DIR, MOT_R_PWM, uR);
}

//=============================================================================
// PUBLIC API (for external guidance layer)
//=============================================================================

/**
 * Set velocity references for both wheels.
 * Call this periodically from the guidance layer.
 * 
 * @param leftVel Left wheel velocity setpoint [rad/s]
 * @param rightVel Right wheel velocity setpoint [rad/s]
 */
void setVelocityRef(float leftVel, float rightVel) {
    vrefL = leftVel;
    vrefR = rightVel;
    lastRefUpdateMs = millis();
}

//=============================================================================
// MAIN PROGRAM
//=============================================================================

void setup() {
    // Configure encoder pins with pull-ups
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    
    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_L_B), encL_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_R_B), encR_B, CHANGE);
    
    // Configure motor driver pins
    pinMode(MOT_L_DIR, OUTPUT);
    pinMode(MOT_L_PWM, OUTPUT);
    pinMode(MOT_R_DIR, OUTPUT);
    pinMode(MOT_R_PWM, OUTPUT);
    stopMotors();
    
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial (timeout for standalone operation)
    }
    
    // Start control loop timer
    FlexiTimer2::set(SAMPLE_MS, controlISR);
    FlexiTimer2::start();
    
    Serial.println(F("Proto2 Motor Speed PI started"));
    Serial.print(F("Config: Kp="));
    Serial.print(KP, 3);
    Serial.print(F(", Ki="));
    Serial.print(KI, 3);
    Serial.print(F(", Ts="));
    Serial.print(SAMPLE_MS);
    Serial.println(F("ms"));
    
    // Initialize reference timeout
    lastRefUpdateMs = millis();
}

void loop() {
    // Demo: velocity reference sweep
    // Replace this with your guidance layer implementation
    static uint32_t sweepStart = millis();
    uint32_t elapsed = millis() - sweepStart;
    
    if (elapsed < 3000) {
        setVelocityRef(5.0f, 5.0f);      // Low speed forward
    } else if (elapsed < 6000) {
        setVelocityRef(10.0f, 10.0f);    // Medium speed forward
    } else {
        sweepStart = millis();            // Reset sweep
    }
    
    // Periodic debug output (matches log_parser.py format)
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
        lastPrint = millis();
        
        // Temporarily disable interrupts for consistent reads
        noInterrupts();
        float wL = omegaL;
        float wR = omegaR;
        float vL = vrefL;
        float vR = vrefR;
        float iL = integralL;
        float iR = integralR;
        interrupts();
        
        Serial.print(F("wL="));
        Serial.print(wL, 2);
        Serial.print(F(",wR="));
        Serial.print(wR, 2);
        Serial.print(F(",vref="));
        Serial.print(vL, 1);
        Serial.print(F(",I_L="));
        Serial.print(iL, 2);
        Serial.print(F(",I_R="));
        Serial.println(iR, 2);
    }
}
