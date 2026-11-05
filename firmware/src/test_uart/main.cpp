#include <Arduino.h>

// ============================================
// UART Test - Basic serial communication test
// ============================================

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== UART Test Started ===");

  // Configure Serial1 for your device
  // Serial1.begin(1000000, SERIAL_8N1, 18, 17); // ESP32-S3 pins
  Serial1.begin(1000000); // Default pins

  Serial.println("Serial1 initialized at 1000000 baud");
}

void loop()
{
  // Echo test: forward data between Serial (USB) and Serial1 (device)
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);
    Serial.print("TX: 0x");
    Serial.println(c, HEX);
  }

  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.print("RX: 0x");
    Serial.println(c, HEX);
  }
}
