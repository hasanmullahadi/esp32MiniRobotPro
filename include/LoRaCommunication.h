#ifndef LORACOMMUNICATION_H
#define LORACOMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_SSD1306.h>


class LoRaCommunication {
  private:
    int ssPin;
    int resetPin;
    int dio0Pin;
    long frequency;
    unsigned long lastSendTime;
    unsigned long interval;
    byte localAddress;
    byte destination;
    byte msgCount;

  public:
    LoRaCommunication(int ss, int reset, int dio0, long freq, byte localAddr, byte dest);
    void begin();
    void sendMessage(String message);
    void receiveMessage();
    void receiveMessage(Adafruit_SSD1306 &display);  // New method to display received message
    void setInterval(unsigned long newInterval);
};

#endif
