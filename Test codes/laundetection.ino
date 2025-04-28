#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13  // Activity LED
#define BUZZER_PIN 27
#define LAUNCH_LED_PIN 28
#define EXIT_PIN 32

bool blinkState = false;

const float ACC_THRESHOLD = 15.0;  // Threshold acceleration in m/s² to detect launch
const int SAMPLE_COUNT = 1;       // Number of consistent samples for detection
float ax_mps2, ay_mps2, az_mps2;   // Acceleration in m/s²
int consistentLaunchReadings = 0;

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #endif

    Serial.begin(38400);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LAUNCH_LED_PIN, OUTPUT);
    pinMode(EXIT_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LAUNCH_LED_PIN, LOW);
    digitalWrite(EXIT_PIN, LOW);
}

void loop() {
    // Read raw accel/gyro measurements
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw data to m/s² (assuming +/-2g sensitivity)
    ax_mps2 = (float)ax / 16384.0 * 9.8;
    ay_mps2 = (float)ay / 16384.0 * 9.8;
    az_mps2 = (float)az / 16384.0 * 9.8;

    // Calculate the resultant acceleration vector magnitude
    float totalAccel = sqrt(ax_mps2 * ax_mps2 + ay_mps2 * ay_mps2 + az_mps2 * az_mps2);

    // Print acceleration values
    Serial.print("Accel (m/s^2):\t");
    Serial.print("X: "); Serial.print(ax_mps2); Serial.print("\t");
    Serial.print("Y: "); Serial.print(ay_mps2); Serial.print("\t");
    Serial.print("Z: "); Serial.print(az_mps2); Serial.print("\t");
    Serial.print("Total: "); Serial.println(totalAccel);

    // Check for launch condition
    if (totalAccel > ACC_THRESHOLD) {
        consistentLaunchReadings++;
        if (consistentLaunchReadings >= SAMPLE_COUNT) {
            // Launch detected
            Serial.println("Launch detected!");
            digitalWrite(BUZZER_PIN, HIGH);        // Activate buzzer
            digitalWrite(LAUNCH_LED_PIN, HIGH);   // Turn on LED
            digitalWrite(EXIT_PIN, HIGH);  
            delay(5000);       // Set exit condition
            digitalWrite(BUZZER_PIN, LOW);        // Activate buzzer
            digitalWrite(LAUNCH_LED_PIN, LOW);   // Turn on LED
            digitalWrite(EXIT_PIN, LOW);

            // Exit the loop
            while (1);  // Infinite loop to stop further execution
        }
    } else {
        consistentLaunchReadings = 0;  // Reset counter if threshold not met
    }

    // Blink activity LED
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    delay(100);  // Delay for stability
}