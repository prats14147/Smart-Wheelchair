#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Pin number definitions
#define TRIG_PIN_1 13
#define ECHO_PIN_1 12
#define TRIG_PIN_2 7
#define ECHO_PIN_2 6
#define LDR_PIN A3
#define ROTATION_PIN A0
#define FORBAC_PIN A1
#define SPEED_PIN A2
#define LED_PIN 4
#define SOLAR_PIN 5
#define MOT_IN1 9
#define MOT_IN2 10
#define MOT_IN3 3
#define MOT_IN4 11
#define BATTERY_PIN A4

// Constants 
const int LDR_THRESHOLD = 500; 
const int POT_THRESHOLD = 512; 
const int AUTO_AVOID_THRESHOLD_LOW = 450;
const int AUTO_AVOID_THRESHOLD_HIGH = 550;

// LCD display initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variables
long duration1, duration2;
int distance1, distance2;
int ldrValue, batteryPercentage;
int rotationValue, forbacValue, speedValue;
float batteryVoltage;
float batteryPercent;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Pin Modes
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(ROTATION_PIN, INPUT);
  pinMode(FORBAC_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SOLAR_PIN, INPUT);
  pinMode(MOT_IN1, OUTPUT);
  pinMode(MOT_IN2, OUTPUT);
  pinMode(MOT_IN3, OUTPUT);
  pinMode(MOT_IN4, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
}

void loop() {
  // Read Ultrasonic Sensors
  distance1 = getDistance(TRIG_PIN_1, ECHO_PIN_1);
  distance2 = getDistance(TRIG_PIN_2, ECHO_PIN_2);

  // Read Potentiometers
  rotationValue = analogRead(ROTATION_PIN);
  forbacValue = analogRead(FORBAC_PIN);
  speedValue = analogRead(SPEED_PIN);

  // Read LDR and Solar
  ldrValue = analogRead(LDR_PIN);
  batteryPercentage = map(analogRead(SOLAR_PIN), 0, 1023, 0, 100);

  // LDR Light Control
  if (ldrValue < LDR_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH); // Turn on light
    lcd.setCursor(0, 1);
    lcd.print("Light: ON ");
  } else {
    digitalWrite(LED_PIN, LOW); // Turn off light
    lcd.setCursor(0, 1);
    lcd.print("Light: OFF");
  }

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("L:" + String(distance1) + " R:" + String(distance2));
  delay(500);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Bat:" + String(batteryPercentage) + "%");
  delay(500);
  lcd.clear();

  // Check for Manual or Auto Avoidance
  if (rotationValue < AUTO_AVOID_THRESHOLD_LOW || rotationValue > AUTO_AVOID_THRESHOLD_HIGH) {
    // Manual Control - Avoid obstacles if necessary
    if (distance1 < 20) {
      controlMotors(1023, POT_THRESHOLD, 512); // Turn right
      lcd.print("Obs: L");
      delay(1000);
      lcd.clear();
      lcd.print("Manual Avoid: L");
      delay(1000);
    } else if (distance2 < 20) {
      controlMotors(0, POT_THRESHOLD, 512); // Turn left
      lcd.print("Obs on: R");
      delay(1000);
      lcd.clear();
      lcd.print("Manual Avoid: R");
      delay(1000);
      lcd.clear();
    }
  } else {
    // Auto Control - Avoid obstacles automatically
    if (distance1 < 20) {
      controlMotors(1023, POT_THRESHOLD, 512); // Turn right
      lcd.print("obs on: L");
      delay(1000);
      lcd.clear();
      lcd.print("Auto Avoid: L");
      delay(1000);
    } else if (distance2 < 20) {
      controlMotors(0, POT_THRESHOLD, 512); // Turn left
      lcd.print("obs on: R");
      delay(1000);
      lcd.clear();
      lcd.print("Auto Avoid: R");
      delay(1000);
      lcd.clear();
    }
    
  }

  // Rechargeable Battery Readout
  int rawValue = analogRead(BATTERY_PIN);
  batteryVoltage = rawValue * (12.0 / 1023.0);
  batteryPercent = map(batteryVoltage * 100, 11.5 * 100, 12.6 * 100, 0, 100);
  batteryPercent = constrain(batteryPercent, 0, 100);

  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage, 2);
  Serial.print("Battery Percentage: ");
  Serial.print(batteryPercent);
  Serial.println("%");
  Serial.print("LDR: "); Serial.println(ldrValue);
  Serial.print("Battery: "); Serial.println(batteryPercentage);
  Serial.print("Rotation: "); Serial.println(rotationValue);
  Serial.print("Forward/Backward: "); Serial.println(forbacValue);
  Serial.print("Speed: "); Serial.println(speedValue);
  Serial.print("Left Distance: "); Serial.println(distance1);
  Serial.print("Right Distance: "); Serial.println(distance2);

  delay(500); // Delay for readability
}

// Function to Get Distance from Ultrasonic Sensor
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

// Function to Control Motors
void controlMotors(int rotation, int forbac, int speed) {
  int motorSpeed = map(speed, 0, 1023, 0, 255);
  analogWrite(MOT_IN1, 0);
  analogWrite(MOT_IN2, 0);
  analogWrite(MOT_IN3, 0);
  analogWrite(MOT_IN4, 0);

  if (forbac > POT_THRESHOLD) {
    // Forward
    analogWrite(MOT_IN1, motorSpeed);
    analogWrite(MOT_IN2, 0);
    analogWrite(MOT_IN3, motorSpeed);
    analogWrite(MOT_IN4, 0);
  } else {
    // Backward
    analogWrite(MOT_IN1, 0);
    analogWrite(MOT_IN2, motorSpeed);
    analogWrite(MOT_IN3, 0);
    analogWrite(MOT_IN4, motorSpeed);
  }

  if (rotation > POT_THRESHOLD) {
    // Right Turn
    analogWrite(MOT_IN1, motorSpeed);
    analogWrite(MOT_IN2, 0);
  } else if (rotation < POT_THRESHOLD) {
    // Left Turn
    analogWrite(MOT_IN3, motorSpeed);
    analogWrite(MOT_IN4, 0);
  }
}
