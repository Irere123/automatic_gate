/**
 * @file automatic_gate.ino
 * @brief Controls an automatic gate system using an ultrasonic sensor, servo
 * motor, LEDs, and a buzzer.
 *
 * The gate opens when an object is detected within a threshold distance by the
 * ultrasonic sensor. A blue LED indicates the gate is open, and a red LED
 * indicates it's closed. A buzzer sounds intermittently while the gate is open.
 * The gate automatically closes after a set delay if no object is detected.
 */

#include <Servo.h> // Include the Servo library for controlling the servo motor.

// Pin definitions: Assigning microcontroller pins to various components.
const int triggerPin = 2;     // Pin connected to the TRIG pin of the ultrasonic
                              // sensor. Sends out ultrasonic pulses.
const int echoPin = 3;        // Pin connected to the ECHO pin of the ultrasonic
                              // sensor. Receives reflected ultrasonic pulses.
const int redLedAnodePin = 4; // Pin connected to the anode (+) of the red LED.
const int redLedCathodePin =
    8; // Pin connected to the cathode (-) of the red LED (if using a common
       // anode RGB LED, this might be different). For a standard LED, this
       // would typically go to GND. Assuming common cathode setup for
       // individual LEDs.
const int blueLedCathodePin =
    7;                    // Pin connected to the cathode (-) of the blue LED.
const int blueLedPin = 5; // Pin connected to the anode (+) of the blue LED.
const int servoPin = 6; // Pin connected to the signal wire of the servo motor.
const int buzzerPin = 12; // Pin connected to the buzzer.

// Constants: Define fixed values used throughout the program.
const float thresholdDistance = 30.0; // Distance in centimeters. If an object
                                      // is closer than this, the gate opens.
const int closedAngle =
    11; // Servo motor angle (in degrees) when the gate is closed.
const int openAngle =
    100; // Servo motor angle (in degrees) when the gate is open.
const unsigned long closeDelay =
    5000; // Time in milliseconds (5 seconds) the gate waits before closing
          // after an object is no longer detected.

Servo myServo;       // Create a Servo object to control the servo motor.
bool isOpen = false; // Boolean flag to track the current state of the gate
                     // (true if open, false if closed).
unsigned long lastDetectionTime =
    0; // Stores the time (in milliseconds) when an object was last detected.
       // Used for the auto-close timer.

// Buzzer timing variables: Control the intermittent beeping of the buzzer.
unsigned long lastBeepTime =
    0; // Stores the time (in milliseconds) when the buzzer last changed its
       // state (started or stopped beeping).
bool buzzerOn = false; // Boolean flag to track if the buzzer is currently
                       // sounding within its beep cycle.
const unsigned long beepOnTime = 50; // Duration in milliseconds for which the
                                     // buzzer stays ON during one beep.
const unsigned long beepCycle =
    500; // Total duration in milliseconds for one complete beep cycle (ON time
         // + OFF time).

/**
 * @brief Setup function, runs once when the Arduino starts or is reset.
 * Initializes pin modes, serial communication, servo, and initial states of
 * components.
 */
void setup() {
  // Initialize pin modes for the ultrasonic sensor.
  pinMode(triggerPin, OUTPUT); // triggerPin sends signals, so it's an OUTPUT.
  pinMode(echoPin, INPUT);     // echoPin receives signals, so it's an INPUT.

  // Initialize pin modes for the LEDs.
  pinMode(redLedAnodePin, OUTPUT);    // Controls the red LED.
  pinMode(redLedCathodePin, OUTPUT);  // Controls the cathode of the red LED.
  pinMode(blueLedCathodePin, OUTPUT); // Controls the cathode of the blue LED.

  // Set LED cathodes to LOW. This is typical for common cathode LEDs or
  // individual LEDs where the cathode is connected to a pin. If using common
  // anode, these would be HIGH and the anode pins would control the LEDs.
  digitalWrite(redLedCathodePin,
               LOW); // Connect red LED cathode to ground potential.
  digitalWrite(blueLedCathodePin,
               LOW); // Connect blue LED cathode to ground potential.

  pinMode(blueLedPin, OUTPUT); // Controls the blue LED.
  pinMode(buzzerPin, OUTPUT);  // Controls the buzzer.

  // Attach the servo motor to its designated pin and set its initial position.
  myServo.attach(servoPin);   // Associates the Servo object with the servoPin.
  myServo.write(closedAngle); // Set the servo to the closed position initially.

  // Set initial states for LEDs and buzzer.
  digitalWrite(redLedAnodePin, HIGH); // Turn the red LED ON (gate is closed).
  digitalWrite(blueLedPin, LOW);      // Turn the blue LED OFF.
  digitalWrite(buzzerPin, LOW);       // Ensure the buzzer is OFF.
}

/**
 * @brief Measures the distance to an object using the ultrasonic sensor.
 * @return The distance in centimeters.
 */
float getDistance() {
  // Send a short ultrasonic pulse.
  digitalWrite(triggerPin, LOW);  // Ensure the trigger pin is low initially.
  delayMicroseconds(2);           // Wait for 2 microseconds.
  digitalWrite(triggerPin, HIGH); // Set the trigger pin high to send a pulse.
  delayMicroseconds(10); // Keep the trigger pin high for 10 microseconds.
  digitalWrite(triggerPin, LOW); // Set the trigger pin low again.

  // Measure the duration of the echo pulse.
  // pulseIn() waits for the echoPin to go HIGH, times how long it stays HIGH,
  // then returns the duration in microseconds.
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance.
  // The speed of sound is approximately 343 m/s or 0.0343 cm/µs.
  // The pulse travels to the object and back, so the distance is (duration *
  // speed_of_sound) / 2. (duration * 0.0343) / 2 = duration / 58.3
  // (approximately) The code uses duration / 20.0, which might be a calibrated
  // or simplified value for this specific sensor or setup. A common conversion
  // factor is duration * 0.034 / 2 or duration / 58. Using 20.0 implies a
  // different speed of sound assumption or unit conversion. For typical
  // HC-SR04, distance (cm) = duration (µs) / 58. Let's assume 20.0 is a
  // specific calibration factor for this project.
  return duration / 20.0; // Convert duration to distance (units depend on this
                          // factor, likely cm).
}

/**
 * @brief Main loop function, runs repeatedly after setup().
 * Contains the core logic for gate operation: detecting objects,
 * opening/closing the gate, controlling LEDs, and managing the buzzer.
 */
void loop() {
  // Get the current distance from the ultrasonic sensor.
  float distance = getDistance();
  // Determine if an object is detected within the threshold.
  bool objectDetected =
      (distance < thresholdDistance &&
       distance > 0); // distance > 0 to avoid false positives from sensor error
  // Get the current time in milliseconds.
  unsigned long currentTime = millis();

  // Logic for when an object is detected.
  if (objectDetected) {
    // If the gate is currently closed, open it.
    if (!isOpen) {
      myServo.write(
          openAngle); // Command the servo to move to the open position.
      isOpen = true;  // Update the gate state to open.
      digitalWrite(redLedAnodePin, LOW); // Turn the red LED OFF.
      digitalWrite(blueLedPin, HIGH);    // Turn the blue LED ON.
    }
    // Update the time of the last detection.
    lastDetectionTime = currentTime;
  } else { // Logic for when no object is detected (or object has moved away).
    // If the gate is open and the close delay has passed since the last
    // detection.
    if (isOpen && (currentTime - lastDetectionTime > closeDelay)) {
      myServo.write(
          closedAngle); // Command the servo to move to the closed position.
      isOpen = false;   // Update the gate state to closed.
      digitalWrite(redLedAnodePin, HIGH); // Turn the red LED ON.
      digitalWrite(blueLedPin, LOW);      // Turn the blue LED OFF.
      digitalWrite(
          buzzerPin,
          LOW);         // Ensure the buzzer is turned OFF when the gate closes.
      buzzerOn = false; // Reset buzzer state flag.
    }
  }

  // Buzzer control: Sound the buzzer intermittently while the gate is open.
  if (isOpen) {
    // Check if it's time to start a new beep cycle.
    if (!buzzerOn && (currentTime - lastBeepTime >= beepCycle)) {
      digitalWrite(buzzerPin, HIGH); // Turn the buzzer ON.
      buzzerOn = true;               // Set the buzzer state to ON.
      lastBeepTime = currentTime;    // Record the time the beep started.
    }
    // Check if it's time to end the current beep (turn buzzer OFF).
    else if (buzzerOn && (currentTime - lastBeepTime >= beepOnTime)) {
      digitalWrite(buzzerPin, LOW); // Turn the buzzer OFF.
      buzzerOn = false;             // Set the buzzer state to OFF.
      // lastBeepTime is not updated here, as it marks the start of the ON
      // period or the start of the full cycle for the next ON. The next beep
      // will be triggered based on `beepCycle` from the `lastBeepTime` when it
      // was turned ON.
    }
  }

  // Short delay to prevent overwhelming the processor and to allow components
  // to react. This also affects the responsiveness of the system.
  delay(50); // Delay for 50 milliseconds.
}