// Libraries
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <PulseSensorPlayground.h>

// Constants
#define sensorAnalog A1   // Analog pin connected to the MQ-3 sensor
#define LED 3             // Digital pin connected to the LED
#define buzzer 4          // Digital pin connected to the buzzer
#define threshold 500     // Alcohol level threshold (adjust as needed)
#define PulseWire 0       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0

// Global Variables
Adafruit_MLX90614 mlx;
PulseSensorPlayground pulseSensor;

// Setup function
void setup() {
  Serial.begin(9600);    // Initialize serial communication
  pinMode(LED, OUTPUT);  // Set the LED pin as output
  pinMode(buzzer, OUTPUT); // Set the buzzer pin as output
  pinMode(sensorAnalog, INPUT); // Set the MQ-3 sensor pin as input

  Serial.println("Adafruit MLX90614 test");

  mlx.begin();               // Initializing the MLX90614 sensor

  // Configure the PulseSensor object
  pulseSensor.analogInput(PulseWire);
  pulseSensor.blinkOnPulse(LED);
  pulseSensor.setThreshold(threshold);

  // Check if the pulseSensor object was created and began seeing a signal
  if (pulseSensor.begin()) {
    Serial.println("PulseSensor Object created!");
  }
}

// Main loop
void loop() {
  // Read alcohol level from the MQ-3 sensor
  int alcoholLevel = analogRead(sensorAnalog);
  Serial.print("Alcohol Level: ");
  Serial.println(alcoholLevel);

  // Read temperature from the MLX90614 sensor
  Serial.print("Ambient = ");
  Serial.print(mlx.readAmbientTempF());
  Serial.print("*F\tObject = ");
  Serial.print(mlx.readObjectTempF());
  Serial.println("*F");

  // Check if alcohol level is above threshold
  if (alcoholLevel > threshold) {
    digitalWrite(LED, HIGH);    // Turn on the LED
    digitalWrite(buzzer, HIGH); // Turn on the buzzer
  } else {
    digitalWrite(LED, LOW);     // Turn off the LED
    digitalWrite(buzzer, LOW);  // Turn off the buzzer
  }

  // Check if a heartbeat happened
  if (pulseSensor.sawStartOfBeat()) {
    int heartRate = pulseSensor.getBeatsPerMinute();
    Serial.println("Heartbeat Detected!");
    Serial.print("Heart Rate: ");
    Serial.println(heartRate);
  }

  delay(2000);  // Wait for 2 seconds before taking another reading
}
