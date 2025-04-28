#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP085.h> // Library for BMP180

#define LORA_FREQ 433E6  // Frequency for LoRa communication
#define CS_PIN 10        // Chip Select for LoRa
#define RESET_PIN 9      // Reset pin for LoRa
#define IRQ_PIN 2        // IRQ pin for LoRa

// BMP180 configuration
Adafruit_BMP085 bmp;     // BMP180 sensor object
bool bmpAvailable = false; // To track if BMP180 is available

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial);

  // Initialize LoRa
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("Starting LoRa failed!");
    while (1); // Stop if LoRa initialization fails
  }
  LoRa.setPins(CS_PIN, RESET_PIN, IRQ_PIN);  // Set LoRa pins
  Serial.println("LoRa Initialized!");

  // Initialize BMP180 sensor
  if (bmp.begin()) {
    bmpAvailable = true; // BMP180 is connected and functional
    Serial.println("BMP180 Initialized!");
  } else {
    bmpAvailable = false; // BMP180 is not connected or not working
    Serial.println("BMP180 not found!");
  }
}

void loop() {
  sendBMPData(); // Send BMP180 data over telemetry
  delay(1000);   // Send data at 1-second intervals
}

void sendBMPData() {
  // Get BMP180 sensor data
  float bmpAltitude = NAN;
  float pressure = NAN;
  float temperature = NAN;

  if (bmpAvailable) {
    bmpAltitude = bmp.readAltitude();        // Altitude in meters
    pressure = bmp.readPressure() / 100.0F;  // Pressure in hPa
    temperature = bmp.readTemperature();     // Temperature in Â°C

    // Check if the sensor values are invalid
    if (isnan(bmpAltitude) || isnan(pressure) || isnan(temperature)) {
      bmpAvailable = false; // Mark BMP180 as unavailable if values fail
    }
  }

  // Print data to Serial Monitor
  Serial.print(bmpAltitude, 2);
  Serial.print(",");
  Serial.print(pressure, 2);
  Serial.print(",");
  Serial.println(temperature, 2);

  // Send data over LoRa
  LoRa.beginPacket();
  LoRa.print(bmpAltitude, 2);
  LoRa.print(",");
  LoRa.print(pressure, 2);
  LoRa.print(",");
  LoRa.print(temperature, 2);
  LoRa.endPacket();
}