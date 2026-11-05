#include <Arduino.h>
#include <ETH.h>

// ============================================
// Ethernet Test - ESP32-PoE Ethernet check
// ============================================

static bool eth_connected = false;

void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      ETH.setHostname("esp32-poe-test");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
      Serial.print("ETH MAC: ");
      Serial.println(ETH.macAddress());
      Serial.print("Link Speed: ");
      Serial.print(ETH.linkSpeed());
      Serial.println(" Mbps");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Ethernet Test Started ===");

  WiFi.onEvent(WiFiEvent);
  ETH.begin();

  Serial.println("Waiting for Ethernet connection...");
}

void loop()
{
  if (eth_connected) {
    Serial.print("Connected - IP: ");
    Serial.println(ETH.localIP());
  } else {
    Serial.println("Waiting for Ethernet...");
  }
  delay(5000);
}
