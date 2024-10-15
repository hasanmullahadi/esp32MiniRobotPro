#include "DHTWrapper.h"

DHTWrapper::DHTWrapper(uint8_t pin, DHTesp::DHT_MODEL_t type) {
  dht.setup(pin, type);  // Initialize the DHT sensor with the specified pin and type
  temperature = 0.0;
  humidity = 0.0;
}

void DHTWrapper::begin() {
  // No initialization needed here since it's done in the constructor
}

float DHTWrapper::getTemperature() {
  temperature = dht.getTemperature();
  return temperature;
}

float DHTWrapper::getHumidity() {
  humidity = dht.getHumidity();
  return humidity;
}

float DHTWrapper::getHeatIndex(bool isFahrenheit) {
  if (isFahrenheit) {
    return dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true);
  } else {
    return dht.computeHeatIndex(temperature, humidity, false);
  }
}

void DHTWrapper::displayData(Adafruit_SSD1306 &display) {
  temperature = getTemperature();
  humidity = getHumidity();
  float heatIndexC = getHeatIndex(false);
  float heatIndexF = getHeatIndex(true);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Humidity: ");
  display.print(humidity, 1);
  display.println(" %");
  display.print("Temp: ");
  display.print(temperature, 1);
  display.println(" *C");
  display.print("Temp: ");
  display.print(dht.toFahrenheit(temperature), 1);
  display.println(" *F");
  display.print("HeatIndex: ");
  display.print(heatIndexC, 1);
  display.println(" *C");
  display.print("HeatIndex: ");
  display.print(heatIndexF, 1);
  display.println(" *F");
  display.display();
}
