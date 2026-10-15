// Motor Control Pins
#define FORWARD_LEFT 5
#define BACK_LEFT 6
#define FORWARD_RIGHT 9
#define BACK_RIGHT 10
#define VACUUM_PIN 12
#define BRUSH_PIN 11

// Encoder Pins (must support interrupts)
#define ENCODER_LEFT 3
#define ENCODER_RIGHT 2

// Proximity Sensor Pins
#define SENSOR_DOWN_LEFT A0
#define SENSOR_DOWN_MIDDLE A1
#define SENSOR_DOWN_RIGHT A2

// Timing
const unsigned long SEND_INTERVAL = 50;    // ms -> Control & Odometry: 20 Hz
const unsigned long SENSOR_INTERVAL = 100; // ms -> Sensors: 10 Hz
unsigned long lastSendTime = 0;
unsigned long lastSensorTime = 0;

// Encoder State
volatile long encoderLeft = 0;
volatile long encoderRight = 0;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;

// Motor Direction
bool forwardLeft = false;
bool backLeft = false;
bool forwardRight = false;
bool backRight = false;

// PID Variables
float targetSpeedL = 0;
float targetSpeedR = 0;

float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;

float integralL = 0;
float integralR = 0;
float lastErrorL = 0;
float lastErrorR = 0;

void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(FORWARD_LEFT, OUTPUT);
    pinMode(BACK_LEFT, OUTPUT);
    pinMode(FORWARD_RIGHT, OUTPUT);
    pinMode(BACK_RIGHT, OUTPUT);
    pinMode(VACUUM_PIN, OUTPUT);
    pinMode(BRUSH_PIN, OUTPUT);

    // Encoder interrupts
    pinMode(ENCODER_LEFT, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), countLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), countRight, RISING);

    // Proximity sensors
    pinMode(SENSOR_DOWN_LEFT, INPUT);
    pinMode(SENSOR_DOWN_MIDDLE, INPUT);
    pinMode(SENSOR_DOWN_RIGHT, INPUT);

    // Initialize actuators to OFF
    digitalWrite(VACUUM_PIN, LOW);
    digitalWrite(BRUSH_PIN, LOW);
}

void countLeft() {
    if (forwardLeft) {
        encoderLeft++;
    } else if (backLeft) {
        encoderLeft--;
    }
}

void countRight() {
    if (forwardRight) {
        encoderRight++;
    } else if (backRight) {
        encoderRight--;
    }
}

void loop() {
    // 1. Process incoming commands
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        processCommand(line);
    }

    // 2. Control Loop & Send encoder data at 20 Hz
    unsigned long now = millis();
    if (now - lastSendTime >= SEND_INTERVAL) {
        lastSendTime = now;

        noInterrupts();
        long deltaLeft = encoderLeft - lastEncoderLeft;
        long deltaRight = encoderRight - lastEncoderRight;
        lastEncoderLeft = encoderLeft;
        lastEncoderRight = encoderRight;
        interrupts();

        // Run PID calculation
        updatePID(deltaLeft, deltaRight);

        // Send raw ticks for odometry back to Python
        Serial.print("E,");
        Serial.print(deltaLeft);
        Serial.print(",");
        Serial.println(deltaRight);
    }

    // 3. Send sensor data at 10 Hz
    if (now - lastSensorTime >= SENSOR_INTERVAL) {
        lastSensorTime = now;
        sendSensorData();
    }
}

void processCommand(String line) {
    char cmd = line.charAt(0);

    switch (cmd) {
    case 'M': {
        // Motor Command: "M,targetTicksL,targetTicksR" (e.g., M,12,12)
        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        if (secondComma > 0) {
            targetSpeedL = line.substring(firstComma + 1, secondComma).toFloat();
            targetSpeedR = line.substring(secondComma + 1).toFloat();
        }
        break;
    }

    case 'V': {
        // Vacuum Control: "V,0" or "V,1"
        int comma = line.indexOf(',');
        if (comma > 0) {
            int state = line.substring(comma + 1).toInt();
            digitalWrite(VACUUM_PIN, state == 1 ? HIGH : LOW);
            Serial.print("V,");
            Serial.println(state); // Acknowledge
        }
        break;
    }

    case 'B': {
        // Brush Control: "B,0" or "B,1"
        int comma = line.indexOf(',');
        if (comma > 0) {
            int state = line.substring(comma + 1).toInt();
            digitalWrite(BRUSH_PIN, state == 1 ? HIGH : LOW);
            Serial.print("B,");
            Serial.println(state); // Acknowledge
        }
        break;
    }

    case 'R': {
        // Read Single Sensor
        int firstComma = line.indexOf(',');
        int secondComma = line.indexOf(',', firstComma + 1);
        if (secondComma > 0) {
            char type = line.charAt(firstComma + 1); // 'a' or 'd'
            int pin = line.substring(secondComma + 1).toInt();

            if (type == 'a') {
                int value = analogRead(pin);
                Serial.print("SA,");
                Serial.print(pin);
                Serial.print(",");
                Serial.println(value);
            } else if (type == 'd') {
                int value = digitalRead(pin);
                Serial.print("SD,");
                Serial.print(pin);
                Serial.print(",");
                Serial.println(value);
            }
        }
        break;
    }
    default:
        Serial.println("?"); // Unknown command
        break;
    }
}

void updatePID(long actualL, long actualR) {
    // 1. Calculate Errors (Target Ticks - Actual Ticks)
    float errorL = targetSpeedL - actualL;
    float errorR = targetSpeedR - actualR;

    // 2. Calculate Integral
    integralL += errorL;
    integralR += errorR;

    // Anti-windup
    float maxIntegral = 255.0 / Ki;
    integralL = constrain(integralL, -maxIntegral, maxIntegral);
    integralR = constrain(integralR, -maxIntegral, maxIntegral);

    // Reset integral if target is 0
    if (targetSpeedL == 0)
        integralL = 0;
    if (targetSpeedR == 0)
        integralR = 0;

    // 3. Calculate Derivative
    float derivL = errorL - lastErrorL;
    float derivR = errorR - lastErrorR;

    // 4. Compute final PWM outputs
    int pwmL = (Kp * errorL) + (Ki * integralL) + (Kd * derivL);
    int pwmR = (Kp * errorR) + (Ki * integralR) + (Kd * derivR);

    lastErrorL = errorL;
    lastErrorR = errorR;

    if (targetSpeedL == 0 && targetSpeedR == 0) {
        setMotorPwm(0, 0);
    } else {
        setMotorPwm(pwmL, pwmR);
    }
}

void setMotorPwm(int left, int right) {
    int pwmLeft = constrain(abs(left), 0, 255);
    int pwmRight = constrain(abs(right), 0, 255);

    if (left > 0) {
        forwardLeft = true;
        backLeft = false;
    } else if (left < 0) {
        forwardLeft = false;
        backLeft = true;
    } else {
        forwardLeft = false;
        backLeft = false;
    }

    if (right > 0) {
        forwardRight = true;
        backRight = false;
    } else if (right < 0) {
        forwardRight = false;
        backRight = true;
    } else {
        forwardRight = false;
        backRight = false;
    }

    analogWrite(FORWARD_LEFT, forwardLeft ? pwmLeft : 0);
    analogWrite(BACK_LEFT, backLeft ? pwmLeft : 0);
    analogWrite(FORWARD_RIGHT, forwardRight ? pwmRight : 0);
    analogWrite(BACK_RIGHT, backRight ? pwmRight : 0);
}

void sendSensorData() {
    int left = analogRead(SENSOR_DOWN_LEFT);
    int middle = analogRead(SENSOR_DOWN_MIDDLE);
    int right = analogRead(SENSOR_DOWN_RIGHT);

    Serial.print("S,");
    Serial.print(left);
    Serial.print(",");
    Serial.print(middle);
    Serial.print(",");
    Serial.println(right);
}
