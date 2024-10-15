#ifndef ULTRASONICWRAPPER_H
#define ULTRASONICWRAPPER_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

class UltrasonicWrapper {
  private:
    uint8_t trigPin;
    uint8_t echoPin;
    long duration;
    float distance;

  public:
    UltrasonicWrapper(uint8_t trig, uint8_t echo);
    void begin();
    float getDistance();
    void displayDistance(Adafruit_SSD1306 &display);  // Method to display distance on OLED
};

#endif
