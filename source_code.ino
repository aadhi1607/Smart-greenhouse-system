#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
// Constants for sensors and actuators
const int tempPin = A10; // TMP36 temperature sensor
const int lightPin = A4; // Light sensor
const int moisturePin = A5; // Moisture sensor
const int ledPin = 23;   // LED
const int fanPin = 51;    // Fan
const int motorPin = 37; // Motor control

// Threshold values for sensors
const int lightThreshold = 500; // Threshold value for light
const int tempThreshold = 20;   // Temperature threshold in Celsius
const int moistureThreshold = 300; // Threshold value for moisture Threshold for moisture 0~300:dry soil  300~700:humid soil  700~950:in water

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600); // Initialize serial communication
  pinMode(ledPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  lcd.begin(16, 2);
}

void loop() {
  int tempReading = analogRead(tempPin);
  float voltage = tempReading * 3.3 / 1023.0; // Converting reading to voltage as 5V is the maximum voltage and 1023 is the maximum reading of TMP36 sensor
  float temperatureC = (voltage - 0.5) * 100; // Converting generated voltage into C
  int lightReading = analogRead(lightPin); // Read light intensity
  int moistureReading = analogRead(moisturePin); // Read moisture level

  Serial1.print(temperatureC); // Serial monitor print readings
  Serial1.print(",");
  Serial1.print(lightReading);
  Serial1.print(",");
  Serial1.print(moistureReading);
  Serial1.print(",");

  bool ledStatus = lightReading < lightThreshold; // Checking if the light reading is lesser than threshold value
  digitalWrite(ledPin, ledStatus ? HIGH : LOW);

  bool fanStatus = temperatureC > tempThreshold; // Checking if the temperature reading is greater than threshold value
  digitalWrite(fanPin, fanStatus ? HIGH : LOW);

  bool motorStatus = moistureReading < moistureThreshold; // Checking if the moisture reading is lesser than threshold value
  digitalWrite(motorPin, motorStatus ? HIGH : LOW);

  // Send actuator status to NodeMCU
  Serial1.print(ledStatus ? "ON" : "OFF");
  Serial1.print(",");
  Serial1.print(fanStatus ? "ON" : "OFF");
  Serial1.print(",");
  Serial1.println(motorStatus ? "ON" : "OFF");

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperatureC);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("MOISTURE: ");
  lcd.print(moistureReading);

  delay(1000);
}
