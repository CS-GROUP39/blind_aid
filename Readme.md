# BLIND AID (VERSION 4)
This repository contains code used by our wearable intended to aid the visually impaired get a better sense of their immediate surroundings. We aim for this product to make it easier and safer for those with visual impairments to travel with less inconvinience.

## PROJECTS OVERVIEW
### Core Components and Their Roles
- **Arduino Uno**: The brain of our system.

- **Ultrasonic Distance Sensor (HC-SR04)**: Used for initial object detection and proximity calculations. Provides a broader view of the immediate environment.

- **Time of flight sensor (VL53L0X)**: Paired with the ultrasonic distance sensor for more accurate close range proximity and distance calculations. Excels in scenarios where the HC-SR04 struggles such as detection of soft surfaces and narrow objects. 

- **PWM Vibration Motor Module**: To provide haptic feedback to the user. The strength and frequency will vary with object proximity, providing a strong but subtle alert when an obstacle lies ahead.

- **MPU6050(Gyroscope and Accelerometre)**: To detect changes in orientation and angular velocity, crucial for identifying a fall. It also has an accelerometer we intend to use to calculate an object's relative speed to the user to create more intuitive haptic feedback. Also comes with a temperature sensor we intend to use for refining distance calculations made using the HC-SR04, which vary depending on environmental temperarures.

- **SIM800L EVB**: Our projct will employ a SIM800L module to send SMS messages to an individual’s caretaker in the event that a fall is detected.

- **Power Source**: We chose a rechargeable 2000mAh Li-Po battery for portability and efficiency.

### Functional Components.
- **Charging module (TP4056)**: A module that enables safe battery recharging and power distribution to the rest of the circuit. Charges via a micro USB, but type C USB charging is possible thru a USB C-Micro USB connector.

- **DC-DC boost converter (MT3608)**: This steps up the Li-Po battery’s 3.7V to 6V, connecting to the Arduino’s VIN pin to provide sufficient voltage and current for the Arduino and the rest of the sensors.

- **P-Channel MOSFET (IRF9540N) + push button**: These two parts enable the system to be powered on or off, enabling power efficiency.

## HARDWARE CONNECTION GUIDE
**Li-Po Battery (2000 mAh, 3.7V)**:
- B+ → TP4056 B+
- B- → TP4056 B-

**TP4056 Charging Module**:
- Micro-USB/USB-C → 5V USB charger
- B+, B- → Li-Po battery
- OUT+ → P-MOSFET source
- OUT- → Boost converter Vin-, Arduino GND (common ground)

**P-MOSFET**:
- Source – TP4056 OUT+
- Drain - Boost converter Vin+
- Gate – Arduino D3 via 10kΩ resistor

**Push Button**:
- Leg 1 – Arduino GND
- Leg 2 – Arduino D3

**Boost Converter**:
- Vin+ → P-MOSFET Drain
- Vin- → TP4056 OUT-, Arduino GND
- Vout+ → Arduino VIN (set to ~5–7V)
- Vout- → Arduino GND

**HC-SR04 (Ultrasonic Sensor)**:
- VCC → Arduino 5V
- GND → Common ground
- TRIG → D6
- ECHO → D7

**VL53L0X** :
- VCC → Arduino 5V
- GND → Common ground
- SDA → Arduino A4
- SCL → Arduino A5

**MPU6050**:
- VCC → Arduino 3.3V
- GND → Common ground
- SDA → Arduino A4
- SCL → Arduino A5

**PWM Vibration Motor Module**:
- VCC → Arduino 5V
- GND → Common ground
- PWM → D9

**SIM800L EVB**:
- 5V → Arduino 5v
- GND → Arduino GND
- TX → D10 (SoftwareSerial)
- RX → D11 (SoftwareSerial)

## CODE IMPLEMENTATION
### Summary
The HC-SR04 will be used to detect objects greater than 2 meters away, the distance data it returns will be used also to calculate the incoming object's speed by measuring change in distance over time. The MPU6050's temperature sensor will provide temperature data to refine the speed calculations made using the HC-SR04's data. At distances of 2 meters or less, use the VL53L0X to perform distance calculations and utilize its data to calculate object speed. If the VL53L0X encounters an error or the object is out of range, fall back to the HC-SR04. In both cases, the relative speed of the object in the person's path (30 FOV) will be calculated, using the MPU6050 to get the wearer's speed to make the calculation. The vibration motor will have two types of vibration queues: pulses to indicate the incoming object's speed using the relative speed calculations, and vibration strength to convey proximity. If the object is stationary, convey this through a continuous vibration that grows stronger with proximity. The faster the object, the faster the pulses, the closer the object, the stronger the vibrations.
Once a drastic change in angular acceleration, tilt and speed is detected over a small time window, identify this as a potential fall. Trigger the SIM800L to send an SMS alert to the wearer's caretaker to alert them of the fall. Send any object proximity data if available to give context to whether the fall was on their own accord, or as aresult of a collision with an object.