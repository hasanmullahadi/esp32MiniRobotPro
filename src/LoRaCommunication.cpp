#include "LoRaCommunication.h"

LoRaCommunication::LoRaCommunication(int ss, int reset, int dio0, long freq, byte localAddr, byte dest) {
  ssPin = ss;
  resetPin = reset;
  dio0Pin = dio0;
  frequency = freq;
  lastSendTime = 0;
  interval = 2000; // Default to 2 seconds
  localAddress = localAddr;  // Set the local address
  destination = dest;        // Set the destination address
  msgCount = 0;              // Initialize message count
}

void LoRaCommunication::begin() {
  SPI.begin(18, 19, 23, ssPin);  // Initialize SPI with given pins

  LoRa.setPins(ssPin, resetPin, dio0Pin); // Set LoRa pins

  if (!LoRa.begin(frequency)) { // Initialize LoRa
    Serial.println("LoRa init failed. Check your connections.");
    while (true); // If failed, halt
  }

  Serial.println("LoRa init succeeded.");
}

void LoRaCommunication::sendMessage(String message) {
  if (millis() - lastSendTime > interval) {
    lastSendTime = millis();
    LoRa.beginPacket();                   // Start packet
    LoRa.write(destination);              // Add destination address
    LoRa.write(localAddress);             // Add sender address
    LoRa.write(msgCount);                 // Add message ID
    LoRa.write(message.length());         // Add payload length
    LoRa.print(message);                  // Add payload
    LoRa.endPacket();                     // Finish packet and send it
    Serial.println("Sending: " + message);
    msgCount++;                           // Increment message ID
  }
}

void LoRaCommunication::receiveMessage() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  // Read packet header bytes:
  int recipient = LoRa.read();          // Recipient address
  byte sender = LoRa.read();            // Sender address
  byte incomingMsgId = LoRa.read();     // Incoming msg ID
  byte incomingLength = LoRa.read();    // Incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) { // Check length for error
    Serial.println("Error: message length does not match length");
    return;
  }

  // If the message is for this device or broadcast
  if (recipient == localAddress || recipient == 0xFF) {
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
  } else {
    Serial.println("This message is not for me.");
  }
}

void LoRaCommunication::setInterval(unsigned long newInterval) {
  interval = newInterval;
}


void LoRaCommunication::receiveMessage(Adafruit_SSD1306 &display) {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  // Read packet header bytes:
  int recipient = LoRa.read();          // Recipient address
  byte sender = LoRa.read();            // Sender address
  byte incomingMsgId = LoRa.read();     // Incoming msg ID
  byte incomingLength = LoRa.read();    // Incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) { // Check length for error
    Serial.println("Error: message length does not match length");
    return;
  }

  // Display received message details on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Received from: 0x");
  display.println(String(sender, HEX));
  display.print("Sent to: 0x");
  display.println(String(recipient, HEX));
  display.print("Message ID: ");
  display.println(String(incomingMsgId));
  display.print("Message length: ");
  display.println(String(incomingLength));
  display.print("Message: ");
  display.println(incoming);
  display.print("RSSI: ");
  display.println(String(LoRa.packetRssi()));
  display.print("Snr: ");
  display.println(String(LoRa.packetSnr()));
  display.display();

  // Also print details to the Serial Monitor
  if (recipient == localAddress || recipient == 0xFF) {
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message ID: " + String(incomingMsgId));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println();
  } else {
    Serial.println("This message is not for me.");
  }
}
