# Automatic Gate System with Ultrasonic Sensor, Servo, LEDs, and Buzzer

## Overview
This project implements an automatic gate system using an Arduino, ultrasonic distance sensor, servo motor, LEDs, and a buzzer. The gate opens automatically when an object (such as a person or vehicle) is detected within a set distance, and closes after a delay when the object leaves. Visual and audio indicators show the gate's status.

## Features
- **Automatic Object Detection:** Uses an ultrasonic sensor to detect objects within a configurable distance.
- **Servo-Controlled Gate:** Opens and closes a gate using a servo motor.
- **LED Indicators:**
  - Red LED: Gate is closed
  - Blue LED: Gate is open
- **Buzzer Alert:** Sounds intermittently while the gate is open.
- **Auto-Close:** Gate closes automatically after a set delay if no object is detected.

## Hardware Requirements
- 1 × Arduino board (Uno, Nano, etc.)
- 1 × Ultrasonic sensor (e.g., HC-SR04)
- 1 × Servo motor
- 1 × Red LED
- 1 × Blue LED
- 2 × 220Ω resistors (for LEDs)
- 1 × Buzzer
- Jumper wires
- Breadboard (optional)

## Pin Connections
| Component         | Arduino Pin |
|------------------|-------------|
| Ultrasonic TRIG  | 2           |
| Ultrasonic ECHO  | 3           |
| Red LED Anode    | 4           |
| Red LED Cathode  | 8           |
| Blue LED Anode   | 5           |
| Blue LED Cathode | 7           |
| Servo Signal     | 6           |
| Buzzer           | 12          |

*Note: LED cathodes may be connected to GND or a digital pin depending on your setup.*

## How It Works
1. **Detection:** The ultrasonic sensor continuously measures the distance to the nearest object.
2. **Gate Opening:** If an object is detected within the threshold distance (default: 30 cm), the servo rotates to open the gate, the blue LED turns on, and the buzzer beeps intermittently.
3. **Gate Closing:** If no object is detected for a set delay (default: 5 seconds), the servo closes the gate, the red LED turns on, and the buzzer stops.
4. **Indicators:**
   - Red LED ON: Gate is closed
   - Blue LED ON: Gate is open
   - Buzzer: Beeps while gate is open

## Setup & Usage
1. Connect all components according to the pin mapping above.
2. Open `automatic_gate.ino` in the Arduino IDE.
3. Select your Arduino board and port.
4. Upload the code to your Arduino.
5. Power the Arduino. The gate will operate automatically based on object detection.

## Customization
- **Threshold Distance:** Change `thresholdDistance` to adjust how close an object must be to open the gate.
- **Gate Angles:** Adjust `openAngle` and `closedAngle` for your servo/gate setup.
- **Auto-Close Delay:** Change `closeDelay` (in milliseconds) to set how long the gate stays open after the object leaves.

## License
This project is open-source and free to use for educational and personal purposes. 