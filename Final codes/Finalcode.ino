#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <Teensy_BMP180.h>
#include <MPU6050.h>
#include "I2Cdev.h"

// For Launch detection
bool blinkState = false;
const float ACC_THRESHOLD = 16.0;  // Threshold acceleration in m/s² to detect launch
const int SAMPLE_COUNT = 1;       // Number of consistent samples for detection for inal we keep it 5
float ax_mps2, ay_mps2, az_mps2;   // Acceleration in m/s²
int consistentLaunchReadings = 0;
bool launched = false;
float initialAltitude = 0.0; 
double altitude;
int i;

#define GREEN_LED 28
#define YELLOW_LED 29

//MOSFET pins
#define MOSFET_PIN1 15
#define MOSFET_PIN2 14

// LoRa configuration
#define LORA_FREQ 433E6  // Frequency for LoRa communication
#define CS_PIN 10        // Chip Select for LoRa
#define RESET_PIN 9      // Reset pin for LoRa
#define IRQ_PIN 2        // IRQ pin for LoRa     

// GPS configuration
TinyGPSPlus gps;         // The TinyGPSPlus object
#define gpsSerial Serial1 // Serial1 for GPS module communication on Teensy 4.1 

// BMP180 configuration using secondary I2C port
Teensy_BMP180 bmp180(&Wire2);
const double SEA_LEVEL_PRESSURE_KATHMANDU = 1012; // Sea-level pressure for Kathmandu in hPa

// Altitude and trigger configuration
const int buzzerPin = 32;  // Pin connected to buzzer

// Altitude and ignition variables
float currentAltitude = 0.0, maxAltitude = 0.0;
bool DrogueEjection = false;
bool DrogueCompleted = false;
bool MainEjection = false;
bool MainCompleted = false;
unsigned long DrogueEjectionTime = NAN;
unsigned long MainEjectionTime = NAN;

// MPU6050 configuration
MPU6050 mpu;
int16_t ax, ay, az; // Accelerometer raw values
int16_t gx, gy, gz; // Gyroscope raw values

File dataFile;  // File object for logging data

unsigned long launchTime = NAN; // Time when the launch is detected
bool timerActivated = false;  // Flag to ensure the timer action happens only once

// Helper function for time-based logging
String getTimeStamp() {
    unsigned long currentMillis = millis();
    unsigned long seconds = currentMillis / 1000;
    unsigned long minutes = seconds / 60;
    seconds = seconds % 60;
    unsigned long hours = minutes / 60;
    minutes = minutes % 60;

    char timeBuffer[20];
    sprintf(timeBuffer, "%02lu:%02lu:%02lu", hours, minutes, seconds);
    return String(timeBuffer);
}
// SD card logging helper
void logToSD(String message) {
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.print(getTimeStamp());
        dataFile.print(" - ");
        dataFile.println(message);
        dataFile.close();
    } else {
        Serial.println(F("Error opening file for logging"));
    }
}

// Function declarations
String getTimeStamp();
void detectLaunch();
void logToSD(String message);
double getAltitude();
void TriggerDrogueEjection();
void CheckDrogueEjectionDuration();
void TriggerMainEjection();
void CheckMainEjectionDuration();
void sendCombinedTelemetry();
void Startup();
void initialization();

void setup() {
    Serial.begin(9600);
     Startup();

    // Initialize Serial for debugging
    initialization();
    logToSD("Sabaithok initialised");
    LoRa.beginPacket();
    LoRa.print("Sabaithok Initialized!!");
    LoRa.endPacket();
    Serial.println("Sabaithok initialized");

    for (i = 0 ; i<=10; i++){
    if (!isnan(altitude)) {
        initialAltitude = getAltitude();
        delay(10);
    }
    }
}

void loop() {
   // Process GPS data
    while (gpsSerial.available() > 0 ) {
        char data = gpsSerial.read();
        gps.encode(data);
    }

   // Get altitude from BMP180
    double altitude = getAltitude();
    if (!isnan(altitude)) {
        currentAltitude = altitude;

        if (currentAltitude > maxAltitude) {
            maxAltitude = currentAltitude;
        }

        // Check for ignition trigger
        if (!DrogueEjection && !DrogueCompleted && (maxAltitude - currentAltitude > 10)) {
            TriggerDrogueEjection();
        }

        // Check ignition duration
        if (DrogueEjection && !DrogueCompleted) {
            CheckDrogueEjectionDuration();
        }

        if (!MainEjection && !MainCompleted && DrogueCompleted && (millis() - DrogueEjectionTime >= 5000 )){
          TriggerMainEjection();
        }

        if (MainEjection && !MainCompleted){
          CheckMainEjectionDuration();
        }
    } else {
    LoRa.beginPacket();
    LoRa.print("Failed to read altitude.");
    LoRa.endPacket();
        Serial.println("Failed to read altitude.");
    }
    delay(100);
    if (gps.location.isValid()) {
       logToSD(String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6));
    } 

    detectLaunch();

    if (launched && !timerActivated && (millis() - launchTime >= 25000)) { //needs to be changed last minute
        if (!DrogueEjection && !DrogueCompleted) {
            Serial.println("Timer activated 30 seconds after launch detection");
            logToSD("Timer activated 30 seconds after launch detection");
            LoRa.beginPacket();
            LoRa.print("Timer activated 30 seconds after launch detection");
            LoRa.endPacket();
            TriggerDrogueEjection();
        }
        if (DrogueEjection && !DrogueCompleted) {
          CheckDrogueEjectionDuration();
        }
        if (!MainEjection && !MainCompleted && (millis() - DrogueEjectionTime >= 5000 )){ //difference in time also needed
          TriggerMainEjection();
        }
        if (MainEjection && !MainCompleted){
          CheckMainEjectionDuration();
          timerActivated = true;  
        }     
      }
    sendCombinedTelemetry(); 
    if (launched && (millis() - launchTime >= 37000)) {
    for (i=0;i<10;i++){
    digitalWrite(MOSFET_PIN1, HIGH);
    digitalWrite(buzzerPin, HIGH);
    Serial.println("drogue");
    delay(2000);
    digitalWrite(MOSFET_PIN1, LOW);
    digitalWrite(buzzerPin, LOW);
    delay(2000);
    digitalWrite(MOSFET_PIN2, HIGH);
    digitalWrite(buzzerPin, HIGH); 
    Serial.println("main"); 
    delay(2000); 
    digitalWrite(MOSFET_PIN2, LOW);
    digitalWrite(buzzerPin, LOW);
    delay(5000);
  }
  }
    delay(100); 
}

void sendCombinedTelemetry() {
  
    double latitude = gps.location.isValid() ? gps.location.lat() : NAN;
    double longitude = gps.location.isValid() ? gps.location.lng() : NAN;

    Serial.print("Lat: ");
    Serial.print(latitude, 6);
    Serial.print(", Lon: ");
    Serial.print(longitude, 6);
    Serial.print(", Altitude: ");
    Serial.println(currentAltitude, 2);

    LoRa.beginPacket();
    LoRa.print(latitude, 6);
    LoRa.print(",");
    LoRa.print(longitude, 6);
    LoRa.print(",");
    LoRa.print(currentAltitude, 2);
    LoRa.endPacket();
    
}

void TriggerDrogueEjection() {
   digitalWrite(MOSFET_PIN1, HIGH); 
    digitalWrite(buzzerPin, HIGH);
    logToSD("Drogue Parachute Ejected : MOSFET1 ON");
    LoRa.beginPacket();
    LoRa.print("Drogue Parachute Ejected : MOSFET1 ON");
    LoRa.endPacket();
    Serial.println("Drogue Parachute Ejected : MOSFET1 ON");
    DrogueEjection = true;
    DrogueEjectionTime = millis();
}

void CheckDrogueEjectionDuration() {
    if (millis() - DrogueEjectionTime >= 3000) { 
        digitalWrite(MOSFET_PIN1, LOW); 
        digitalWrite(buzzerPin, LOW);
        logToSD("Drogue Parachute Ejection Completed: MOSFET1 OFF");
        Serial.println("Drogue Parachute Ejection Completed: MOSFET1 OFF");
        LoRa.beginPacket();
        LoRa.print("Drogue Parachute Ejection Completed: MOSFET1 OFF");
        LoRa.endPacket();

        DrogueEjection = false;
        DrogueCompleted = true;
    }
}
void TriggerMainEjection(){
   digitalWrite(MOSFET_PIN2, HIGH);  // Turn the MOSFET ON
   digitalWrite(buzzerPin, HIGH); 
   //logging and printing
   logToSD("Main Parachute Ejected : MOSFET2 ON");
   Serial.println("Main Parachute Ejected : MOSFET2 ON");
   LoRa.beginPacket();
   LoRa.print("Main Parachute Ejected : MOSFET2 ON");
   LoRa.endPacket();
   MainEjectionTime = millis();
   MainEjection = true;
}

void CheckMainEjectionDuration() {
    if (millis() - MainEjectionTime >= 3000) { // Check if 4 seconds have passed
        digitalWrite(MOSFET_PIN2, LOW); 
        digitalWrite(buzzerPin, LOW);
        logToSD("TriggerMainEjection Completed: MOSFET2 OFF");
        Serial.println("TriggerMainEjection Completed: MOSFET2 OFF");
        LoRa.beginPacket();
        LoRa.print("TriggerMainEjection Completed: MOSFET2 OFF");
        LoRa.endPacket();

        MainEjection= false;
        MainCompleted = true;
    }
}

void detectLaunch(){
    // Read raw accel/gyro measurements
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert raw data to m/s² (assuming +/-2g sensitivity)
    ax_mps2 = (float)ax / 16384.0 * 9.8;
    ay_mps2 = (float)ay / 16384.0 * 9.8;
    az_mps2 = (float)az / 16384.0 * 9.8;
    float totalAccel = sqrt(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);

    if (!launched) {
        // Altitude launch detection
        if (currentAltitude - initialAltitude >= 10) {
            consistentLaunchReadings++;
        } else {
            consistentLaunchReadings = 0;  
        }

        // Acceleration-based launch detection
        if (totalAccel > ACC_THRESHOLD) {
            consistentLaunchReadings++;
        }
        if (consistentLaunchReadings >= SAMPLE_COUNT) {
            // Launch detected
            Serial.println("LAUNCH DETECTED!");
            launched = true;
            launchTime = millis();
            digitalWrite(buzzerPin, HIGH);        // Activate buzzer
            digitalWrite(GREEN_LED, HIGH);       // Turn on LED
            digitalWrite(YELLOW_LED, HIGH);  

            delay(100);    
            logToSD("LAUNCH DETECTED");
            
            // Telemetry
            LoRa.beginPacket();
            LoRa.print("LAUNCH DETECTED");
            LoRa.endPacket();

            digitalWrite(buzzerPin, LOW);        
            digitalWrite(GREEN_LED, LOW);       
            digitalWrite(YELLOW_LED, LOW);
        }
    }

    // if (!launched && consistentLaunchReadings == 0) {
    //     initialAltitude = currentAltitude; 
    // }

    // Log MPU data and altitude
    String mpuData = String(currentAltitude) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz);
    logToSD(mpuData);
    delay(100);  // Delay for stability
}

void Startup(){
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
    
    //MOSFET pins initialise
    pinMode(MOSFET_PIN1, OUTPUT);  // Set the MOSFET pin as an output
    digitalWrite(MOSFET_PIN1, LOW);  // Initialize the MOSFET to OFF state
    pinMode(MOSFET_PIN2, OUTPUT);  // Set the MOSFET pin as an output
    digitalWrite(MOSFET_PIN2, LOW);  // Initialize the MOSFET to OFF state
    delay(1000);
    
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
     delay(100);
}

void initialization() {
   // Initialize GPS module
    gpsSerial.begin(9600);  // Default baud rate for GPS modules  
    Serial.println("Initializing GPS and LoRa...");

    // Initialize LoRa
    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("Starting LoRa failed!");
        while (1); // Stop if LoRa initialization fails
    }
    LoRa.setPins(CS_PIN, RESET_PIN, IRQ_PIN);  // Set LoRa pins
    Serial.println("LoRa Initialized!");

     // Initialize BMP180 sensor
    bmp180.begin();  // No return value to check
    double testAltitude = getAltitude();
    if (isnan(testAltitude)) {
        Serial.println("BMP180 initialization failed!");
        LoRa.beginPacket();
        LoRa.print("BMP180 initialization failed!");
        LoRa.endPacket();
        while (1); // Halt if BMP180 fails
    } else {
        Serial.println("BMP180 Initialized!");
        LoRa.beginPacket();
        LoRa.print("BMP180 Initialized!");
        LoRa.endPacket();
    }

     // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.println("MPU6050 connection successful");
        LoRa.beginPacket();
        LoRa.print("MPU6050 connection successful");
        LoRa.endPacket();
    } else {
        Serial.println("MPU6050 connection failed");
        LoRa.beginPacket();
        LoRa.print("MPU6050 connection failed");
        LoRa.endPacket();
        while (1); // Halt if MPU6050 fails
    }

    // Initialize SD card
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println(F("SD card initialization failed!"));
        LoRa.beginPacket();
        LoRa.print("SD card initialization failed!");
        LoRa.endPacket();
        while (1);
    }
    Serial.println(F("SD card initialized"));
    LoRa.beginPacket();
    LoRa.print("SD card initialized");
    LoRa.endPacket();
   
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