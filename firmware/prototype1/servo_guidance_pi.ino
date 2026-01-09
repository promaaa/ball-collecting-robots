/**
 * Prototype 1 - Servo-Based Differential Drive with Pixy2 Lateral PI Guidance
 * 
 * This firmware implements PI control for lateral ball tracking using a Pixy2 camera
 * and servo-based differential steering. The robot detects colored balls (orange/white)
 * and adjusts its heading to center the ball in the camera frame.
 * 
 * Hardware Configuration:
 *   - Pixy2 camera (SPI interface)
 *   - 2x Continuous rotation servos
 *   - Arduino UNO or compatible
 * 
 * Control Strategy:
 *   - Visual error: pixel offset from frame center
 *   - PI controller: u = Kp * error + Ki * integral(error)
 *   - Differential steering: left/right servo speeds adjusted by u
 * 
 * Author: Marc Duboc
 * License: MIT
 */

#include <Pixy2.h>
#include <Servo.h>

//=============================================================================
// CONFIGURATION
//=============================================================================

// --- Hardware Pins ---
#define SERVO_LEFT_PIN   9    // Left servo signal pin
#define SERVO_RIGHT_PIN  8    // Right servo signal pin

// --- Pixy2 Settings ---
#define PIXY_SIG_ID      1    // Color signature ID to track (configure in Pixy2)
#define X_CENTER         160L // Pixel center for 320x200 resolution (320/2)

// --- PI Controller Gains ---
// Note: Tune these experimentally. Start with Kp only (Ki=0), then add Ki
const float KP = 0.8f;        // Proportional gain on pixel error [command/pixel]
const float KI = 0.0f;        // Integral gain [command/(pixel*sec)]
                              // Keep at 0 initially - servo asymmetry limits effectiveness

// --- Anti-Windup Configuration ---
const int32_t INTEGRAL_MAX = 50000;  // Maximum integral accumulator value [pixel*ms]
                                      // Prevents integral wind-up during saturation

// --- Motion Parameters ---
const int BASE_CMD = 0;       // Forward speed bias (0 = pure steering)
const int SAMPLE_MS = 50;     // Guidance update period [ms] (20 Hz)
const int CMD_MIN = -400;     // Minimum differential command
const int CMD_MAX = 400;      // Maximum differential command

// --- Servo Configuration ---
// Assumes: 90° = stop, 0°/180° = full speed in opposite directions
const int SERVO_STOP = 90;

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

Pixy2 pixy;
Servo servoL;
Servo servoR;

int32_t errIntegral = 0;      // Integral accumulator [pixel * ms]
unsigned long lastUpdateTime = 0;
bool ballDetected = false;

//=============================================================================
// UTILITY FUNCTIONS
//=============================================================================

/**
 * Clamp a value to a specified range.
 * 
 * @param value Value to clamp
 * @param minVal Minimum allowed value
 * @param maxVal Maximum allowed value
 * @return Clamped value
 */
static inline int clamp(int value, int minVal, int maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

/**
 * Clamp integral accumulator with anti-windup protection.
 * 
 * @param integral Current integral value
 * @return Clamped integral value
 */
static inline int32_t clampIntegral(int32_t integral) {
    if (integral > INTEGRAL_MAX) return INTEGRAL_MAX;
    if (integral < -INTEGRAL_MAX) return -INTEGRAL_MAX;
    return integral;
}

/**
 * Find the largest block of the target signature.
 * 
 * @return Index of largest matching block, or -1 if none found
 */
int findBestBlock() {
    int bestIndex = -1;
    uint32_t bestArea = 0;
    
    for (uint16_t i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature == PIXY_SIG_ID) {
            uint32_t area = (uint32_t)pixy.ccc.blocks[i].m_width * 
                           pixy.ccc.blocks[i].m_height;
            if (area > bestArea) {
                bestArea = area;
                bestIndex = i;
            }
        }
    }
    return bestIndex;
}

/**
 * Stop both servos (neutral position).
 */
void stopServos() {
    servoL.write(SERVO_STOP);
    servoR.write(SERVO_STOP);
}

//=============================================================================
// MAIN PROGRAM
//=============================================================================

void setup() {
    // Initialize servos
    servoL.attach(SERVO_LEFT_PIN);
    servoR.attach(SERVO_RIGHT_PIN);
    stopServos();  // Start in stopped state
    
    // Initialize serial for debugging
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        // Wait for serial connection (timeout after 3s for standalone operation)
    }
    
    // Initialize Pixy2 camera
    if (pixy.init() < 0) {
        Serial.println(F("ERROR: Pixy2 initialization failed!"));
        // Blink LED to indicate error (if available)
        while (1) {
            delay(500);  // Halt on camera failure
        }
    }
    
    Serial.println(F("Proto1 Servo Guidance PI started"));
    Serial.print(F("Config: Kp="));
    Serial.print(KP);
    Serial.print(F(", Ki="));
    Serial.print(KI);
    Serial.print(F(", Sample="));
    Serial.print(SAMPLE_MS);
    Serial.println(F("ms"));
}

void loop() {
    // Rate-limited control loop
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < SAMPLE_MS) {
        return;
    }
    lastUpdateTime = currentTime;
    
    // Get color-connected-components blocks from Pixy2
    pixy.ccc.getBlocks();
    
    if (pixy.ccc.numBlocks == 0) {
        // No detection - stop and reset integral
        stopServos();
        errIntegral = 0;  // Reset integral to prevent wind-up
        
        if (ballDetected) {
            Serial.println(F("NO_BALL"));
            ballDetected = false;
        }
        return;
    }
    
    // Find the best (largest) block matching our signature
    int bestIdx = findBestBlock();
    if (bestIdx < 0) {
        stopServos();
        return;
    }
    
    ballDetected = true;
    
    // Calculate pixel error (positive = ball is right of center)
    int x = pixy.ccc.blocks[bestIdx].m_x;
    int32_t error = x - X_CENTER;
    
    // Update integral with anti-windup clamping
    errIntegral += error * SAMPLE_MS;
    errIntegral = clampIntegral(errIntegral);
    
    // PI control law
    // u > 0 means turn right (reduce right speed, increase left speed)
    float u = KP * error + KI * (errIntegral / 1000.0f);
    
    // Convert to differential commands
    int leftCmd = BASE_CMD + (int)u;
    int rightCmd = BASE_CMD - (int)u;
    
    // Clamp commands to valid range
    leftCmd = clamp(leftCmd, CMD_MIN, CMD_MAX);
    rightCmd = clamp(rightCmd, CMD_MIN, CMD_MAX);
    
    // Map commands to servo angles
    // Left servo: positive command -> faster forward -> higher angle
    // Right servo: positive command -> faster forward -> lower angle (reversed)
    int servoLeft = map(leftCmd, CMD_MIN, CMD_MAX, SERVO_STOP, 180);
    int servoRight = map(rightCmd, CMD_MIN, CMD_MAX, SERVO_STOP, 0);
    
    // Apply to servos
    servoL.write(servoLeft);
    servoR.write(servoRight);
    
    // Debug output (matches log_parser.py format)
    Serial.print(F("ERR="));
    Serial.print(error);
    Serial.print(F(",U="));
    Serial.print(u, 1);
    Serial.print(F(",L="));
    Serial.print(servoLeft - SERVO_STOP);
    Serial.print(F(",R="));
    Serial.println(SERVO_STOP - servoRight);
}
