//This code uses TeensyBMP180 library customised as per out need
#include <Teensy_BMP180.h>
// BMP180 configuration using secondary I2C port
Teensy_BMP180 bmp180(&Wire2);
const double SEA_LEVEL_PRESSURE_KATHMANDU = 1012.0; // Sea-level pressure for Kathmandu in hPa

// Pins for ignition system
const int mosfetPin = 15;  // MOSFET gate pin
const int buzzerPin = 32;  // Buzzer pin

// Altitude and ignition variables
float currentAltitude = 0.0, previousAltitude = 0.0, maxAltitude = 0.0;
bool ignitionTriggered = false;
bool ignitionCompleted = false;
unsigned long ignitionStartTime;
void setup() {
    Serial.begin(9600);

    // Initialize pins
    pinMode(mosfetPin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(mosfetPin, LOW);
    digitalWrite(buzzerPin, LOW);

    // Initialize BMP180 sensor
    bmp180.begin();  // No return value to check
    Serial.println("BMP180 Initialized!");
}

void loop() {
    // Get altitude from BMP180
    double altitude = getAltitude();
    if (!isnan(altitude)) {
        currentAltitude = altitude;
        Serial.print("Current Altitude: ");
        Serial.println(currentAltitude);

        // Update max altitude
        if (currentAltitude > maxAltitude) {
            maxAltitude = currentAltitude;
        }

        // Check for ignition trigger
        if (!ignitionTriggered && !ignitionCompleted && (maxAltitude - currentAltitude > 1.5)) {
            triggerIgnition();
        }

        // Check ignition duration
        if (ignitionTriggered && !ignitionCompleted) {
            checkIgnitionDuration();
        }

        // Update previous altitude
        previousAltitude = currentAltitude;
    } else {
        Serial.println("Failed to read altitude.");
    }

    delay(100); // Adjust delay as needed
}

// Function to calculate altitude with BMP180
double getAltitude() {
    if (bmp180.startTemperature() != 0) {
        double temperature;
        if (bmp180.getTemperature(temperature) != 0) {
            if (bmp180.startPressure() != 0) {
                double pressure;
                if (bmp180.getPressure(pressure, temperature) != 0) {
                    return bmp180.altitude(pressure, SEA_LEVEL_PRESSURE_KATHMANDU);
                }
            }
        }
    }
    return NAN; // Return NAN if any step fails
}

// Trigger the ignition sequence
void triggerIgnition() {
    digitalWrite(mosfetPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
    Serial.println("Ignition Triggered: MOSFET ON");

    ignitionTriggered = true;
    ignitionStartTime = millis();
}

// Check if ignition duration has elapsed
void checkIgnitionDuration() {
    if (millis() - ignitionStartTime >= 4000) { // 4-second duration
        digitalWrite(mosfetPin, LOW);
        digitalWrite(buzzerPin, LOW);
        Serial.println("Ignition Completed: MOSFET OFF");

        ignitionTriggered = false;
        ignitionCompleted = true;
    }
}