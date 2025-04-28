#include <TinyGPSPlus.h>

// The TinyGPSPlus object
TinyGPSPlus gps;

// Use Serial1 for GPS module communication on Teensy 4.1
#define gpsSerial Serial1 

void setup() {
  Serial.begin(115200);    // Debugging via Serial Monitor
  gpsSerial.begin(9600);   // GPS module's baud rate (default is 9600)

  Serial.println(F("Real-Time GPS Data Example"));
  Serial.println(F("Waiting for GPS data..."));
}

void loop() {
  // Process GPS data when available
  while (gpsSerial.available() > 0) {
    char data = gpsSerial.read();
    if (gps.encode(data)) { // If valid GPS data is received
      displayInfo();
    }
  }
}

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
  } else {
    Serial.print(F("INVALID"));
  }

  // Debugging: Display number of satellites
  Serial.print(F("  Satellites: "));
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print(F("INVALID"));
  }

  // Debugging: Display HDOP (Horizontal Dilution of Precision)
  Serial.print(F("  HDOP: "));
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.value());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

// void displayInfo() {
//     if (gps.location.isValid()) {
//         Serial.print(gps.location.lat(), 6);
//         Serial.print(",");
//         Serial.print(gps.location.lng(), 6);
//         Serial.println();
//     }
// }