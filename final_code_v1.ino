#include <Servo.h>  // Include the Servo library

// Servo motor setup
Servo servo;
int servoPin = 10;



// Clamp (Servo) setup
const int CLAMP_SERVO_PIN = 9;
const int OPEN_ANGLE = 25;
const int CLOSE_ANGLE = 70;
const int INITIAL_ANGLE = 25;
const unsigned long MOVE_DELAY = 7;
Servo clampServo;
int currentClampAngle = INITIAL_ANGLE;
bool isClampAttached = true;

// Gear motor setup
#define IN1 46
#define IN2 50
// Gear linear actuator
#define IN3 42
#define IN4 40

// Sonar setup for distance measurements
const int trigPin1 = 28;  // Trigger pin for sonar 1
const int echoPin1 = 30;  // Echo pin for sonar 1
const int trigPin2 = 38;  // Trigger pin for sonar 2 (new sonar)
const int echoPin2 = 36;  // Echo pin for sonar 2 (new sonar)
long duration1, duration2;
float distance1, distance2;

// Define target distances for motor
int targetMotorDistance = 10;  // Target distance for motor (in cm)

// Variables to track consecutive sonar readings
int motorStopCount = 0;  // Count for consecutive motor readings less than target
const int CONSECUTIVE_READINGS_THRESHOLD = 3;  // Threshold for consecutive readings

// Pin Definitions
const int fsrPin = A0; // Analog pin connected to FSR
const int seriesResistor = 10000; // Resistor value in ohms (10kΩ)

// Calibration Constants (adjust as needed)
const float vcc = 5.0; // Operating voltage
const float fsrMaxResistance = 1000000; // Max resistance of FSR

void setup() {
  // Setup for all devices
  Serial.begin(9600);  // Start serial communication

  // Servo motor setup
  servo.attach(servoPin);

  // Clamp (Servo) setup
  clampServo.attach(CLAMP_SERVO_PIN);
  clampServo.write(INITIAL_ANGLE);

  // Gear motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Setup for Sonar 1
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  // Setup for Sonar 2
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  // Start with trigger pins low
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);

  // Print command instructions
  Serial.println("Enter commands:");
  Serial.println("'s' - Start sequence");
  Serial.println("'r' - Reset sequence");
}

void loop() {
  // Read and print sonar distances in real-time
  distance1 = getSonarDistance(trigPin1, echoPin1);  // Get the distance from sonar 1
  distance2 = getSonarDistance(trigPin2, echoPin2);  // Get the distance from sonar 2

  if (distance1 == -1) {
    Serial.println("Sonar 1: Out of range or invalid distance");
  } else {
    Serial.print("Sonar 1 Distance: ");
    Serial.print(distance1);
    Serial.println(" cm");
  }

  if (distance2 == -1) {
    Serial.println("Sonar 2: Out of range or invalid distance");
  } else {
    Serial.print("Sonar 2 Distance: ");
    Serial.print(distance2);
    Serial.println(" cm");
  }

  if (Serial.available() > 0) {  // Check if data is available to read
    char command = Serial.read();  // Read the input command
    command = tolower(command);  // Convert to lowercase for consistency

    // Remove unwanted line endings
    if (command == '\n' || command == '\r') {
      return;  // Ignore newline or carriage return characters
    }

    switch (command) {
      case 's':
        Serial.println("Starting sequence...");
        startSequence();  // Start the sequence
        break;

      case 'r':
        Serial.println("Resetting sequence...");
        resetSequence();  // Reset the sequence
        break;

      default:
        Serial.println("Invalid command. Use 's' to start or 'r' to reset.");
        break;
    }
  }

  delay(100);  // Update every 100 ms (adjust as needed)
}

// Function to get sonar distance
float getSonarDistance(int triggerPin, int echoPin) {
  // Send a 10µs pulse to the trigger pin
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Read the echo pin and calculate the duration of the pulse
  duration1 = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  float distance = duration1 * 0.034 / 2;

  return distance;
}

// Start the sequence
void startSequence() {
  // Step 1: Open the clamp
  openClamp();

  // Step 2: Move motor forward until target distance or condition met
  moveMotorForwardUntilDistance(targetMotorDistance);

  // Step 3: Wait for 1 second
  delay(500);

  // Step 4: Extend the linear actuator for 5 seconds
  extendLinearActuatorFor5Seconds();

  // Step 5: Rotate the servo forward
  rotateServoForwardUntilThreshold();

  // Step 6: Retract the linear actuator
  retractLinearActuatorUntilThreshold();

  // Step 7: Rotate the servo backward
  rotateServoBackwardUntilThreshold();

  // Step 8: Close the clamp
  closeClamp();

  // Step 9: Reverse Motor
  motorReverse();
}

// Reset the sequence
void resetSequence() {
  Serial.println("Resetting motors and actuators...");
  // Add logic here to reset the sequence if needed
}

// Function to open the clamp
void openClamp() {
  if (currentClampAngle != OPEN_ANGLE) {
    moveClampSmoothly(OPEN_ANGLE);
    currentClampAngle = OPEN_ANGLE;
    Serial.println("Clamp opened.");
  } else {
    Serial.println("Clamp is already open.");
  }
}

// Function to move motor forward until distance is less than or equal to target distance
void moveMotorForwardUntilDistance(int targetDistance) {
  motorStopCount = 0;  // Reset counter for motor stop condition
  while (motorStopCount < CONSECUTIVE_READINGS_THRESHOLD) {
    distance1 = getSonarDistance(trigPin1, echoPin1);

    if (distance1 < targetDistance) {
      motorStopCount++;
    } else {
      motorStopCount = 0;  // Reset count if condition is not met
    }
    Serial.print("Sonar 1 Distance: ");
    Serial.print(distance1);
    Serial.println(" cm");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
      // Small delay to avoid overwhelming the sensor and motor
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  Serial.println("Motor stopped after reaching target distance.");
}

// Function to extend the linear actuator for 5 seconds
void extendLinearActuatorFor5Seconds() {
  unsigned long startTime = millis();  // Record the start time

  // Extend the actuator for 5 seconds
  while (millis() - startTime < 12000) {
    digitalWrite(IN3, LOW);  // Move actuator forward
    digitalWrite(IN4, HIGH);  // Control direction
    delay(100);  // Small delay to allow actuator to extend
  }

  // Stop the actuator after 5 seconds
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Linear actuator extended for 5 seconds.");
}

// Function to move the clamp servo smoothly
void moveClampSmoothly(int targetAngle) {
  int step = (targetAngle > currentClampAngle) ? 1 : -1;  // Determine direction
  for (int angle = currentClampAngle; angle != targetAngle; angle += step) {
    clampServo.write(angle);
    delay(MOVE_DELAY);
  }
  clampServo.write(targetAngle);
}

// Function to rotate servo forward until 8cm or greater distance is detected
void rotateServoForwardUntilThreshold() {
  int motorStopCount2 = 0;
  
  while (motorStopCount2 < CONSECUTIVE_READINGS_THRESHOLD) {
    distance2 = getSonarDistance(trigPin2, echoPin2);

    if (distance2 >= ) {
      motorStopCount2++;
    } else {
      motorStopCount2 = 0;  // Reset count if condition is not met
    }
    
    Serial.print("Sonar 2 Distance (Forward): ");
    Serial.print(distance2);
    Serial.println(" cm");
    
    servo.write(0);  // Rotate the servo forward (adjust angle as needed)
    delay(100);  // Small delay to allow servo to move
  }

  // Stop the servo after reaching target condition
  servo.write(90);
  delay(1000);  // 1 second delay
}

// Function to retract the linear actuator until threshold is met
void retractLinearActuatorUntilThreshold() {
  Serial.println("Retracting linear actuator based on FSR reading...");

  while (true) {

  // Read the analog value from FSR
  int fsrValue = analogRead(fsrPin);
  
  // Convert to voltage
  float fsrVoltage = fsrValue * (vcc / 1023.0);

  // Calculate resistance
  float fsrResistance = (fsrVoltage == 0) ? fsrMaxResistance : (vcc-fsrVoltage) / (fsrVoltage / seriesResistor) ;

  // Print values
  Serial.print("Analog Value: ");
  Serial.print(fsrValue);
  Serial.print(" | Voltage: ");
  Serial.print(fsrVoltage, 2);
  Serial.print("V | Resistance: ");
  Serial.print(fsrResistance, 2);

  delay(500); // Delay for readability
    // Check if the analog value meets the threshold
    if (fsrValue >= 200) {
      Serial.print("FSR Value: ");
      Serial.print(fsrValue);
      Serial.println(" | Threshold met. Stopping actuator.");
      break; // Exit the loop once the condition is met
    }

    // Keep retracting the actuator
    digitalWrite(IN3, HIGH);  // Move actuator backward
    digitalWrite(IN4, LOW);   // Control direction

    // Print the FSR value for debugging
    Serial.print("FSR Value: ");
    Serial.println(fsrValue);

    delay(100); // Small delay for stability
  }
  delay(500);
  // Stop the actuator after threshold is met
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Linear actuator retraction stopped.");
  delay(1000);
}

// Function to rotate servo backward until 4cm or less distance is detected
void rotateServoBackwardUntilThreshold() {
  int motorStopCount2 = 0;
  
  while (motorStopCount2 < 2) {
    distance2 = getSonarDistance(trigPin2, echoPin2);

    if (distance2 <= 2) {
      motorStopCount2++;
    } else {
      motorStopCount2 = 0;  // Reset count if condition is not met
    }

    Serial.print("Sonar 2 Distance (Backward): ");
    Serial.print(distance2);
    Serial.println(" cm");
    
    servo.write(180);  // Rotate the servo backward
    delay(100);  // Small delay to allow servo to move
  }

  // Stop the servo after reaching target condition
  servo.write(90);
}
// Function to close the clamp
void closeClamp() {
  if (currentClampAngle != CLOSE_ANGLE) {
    moveClampSmoothly(CLOSE_ANGLE);
    currentClampAngle = CLOSE_ANGLE;
    Serial.println("Clamp closed.");
  } else {
    Serial.println("Clamp is already closed.");
  }
  delay(1000);
}
void motorReverse() {
        Serial.println("Gear Motor Moving Reverse");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        delay(2000);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
}
