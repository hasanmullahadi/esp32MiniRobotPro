#ifndef DHTWRAPPER_H
#define DHTWRAPPER_H

#include <Arduino.h>
#include <DHTesp.h>
#include <Adafruit_SSD1306.h>

class DHTWrapper {
  private:
    DHTesp dht;
    float temperature;
    float humidity;

  public:
    DHTWrapper(uint8_t pin, DHTesp::DHT_MODEL_t type);
    void begin();
    float getTemperature();
    float getHumidity();
    float getHeatIndex(bool isFahrenheit = false);
    void displayData(Adafruit_SSD1306 &display);  // Method to display data on OLED
};

#endif
