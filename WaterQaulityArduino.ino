// Blynk Template and Device Information
#define BLYNK_TEMPLATE_ID "TMPL6Ik1HURWC"
#define BLYNK_TEMPLATE_NAME "Water Quality Monitoring"
#define BLYNK_AUTH_TOKEN "etP-Vpnl20Ojrs9MMrHAs0wJnvA6sXqx"

#define BLYNK_PRINT Serial // Uncomment this line to see debug prints

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <math.h> // For pow()

// WiFi credentials
char ssid[] = "Laplace";     // Replace with your WiFi SSID
char pass[] = "silicon123"; // Replace with your WiFi Password

// Pin definitions
const int tdsSensorPin = 34; // Using GPIO34 for analog input

namespace device {
  const float aref = 3.3; // Reference voltage for ESP32
}

// Sensor data namespace
namespace sensor {
  float ec = 0;
  unsigned int tds = 0;
  const float ecCalibration = 1.0; // Calibration factor
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095)
  pinMode(tdsSensorPin, INPUT);
}

void loop() {
  Blynk.run();
  readTdsQuick();
  delay(1000); // Delay between readings
}

void readTdsQuick() {
  // Hardcoded temperature compensation as you don't have a temperature sensor
  float waterTemp = 25.0; // Assuming a constant temperature of 25°C

  // Read the analog value from TDS sensor and convert to voltage
  float rawEc = analogRead(tdsSensorPin) * device::aref / 4095.0;

  // Temperature compensation formula
  float temperatureCoefficient = 1.0 + 0.02 * (waterTemp - 25.0);

  // Calculate Electrical Conductivity (EC) with compensation
  sensor::ec = (rawEc / temperatureCoefficient) * sensor::ecCalibration;

  // Convert EC to TDS using the polynomial formula you provided
  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * pow(sensor::ec, 2) + 857.39 * sensor::ec) * 0.5;

  // Print values to Serial Monitor
  Serial.print("EC: "); Serial.print(sensor::ec, 2); Serial.println(" mS/cm");
  Serial.print("TDS: "); Serial.print(sensor::tds); Serial.println(" ppm");
  Serial.println("---------------------------");

  // Send data to Blynk app
  Blynk.virtualWrite(V0, sensor::tds); // TDS on V0
  Blynk.virtualWrite(V1, sensor::ec);  // EC on V1
}
