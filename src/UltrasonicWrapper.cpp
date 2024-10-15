#include "UltrasonicWrapper.h"

UltrasonicWrapper::UltrasonicWrapper(uint8_t trig, uint8_t echo) {
  trigPin = trig;
  echoPin = echo;
  duration = 0;
  distance = 0.0;
}

void UltrasonicWrapper::begin() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

float UltrasonicWrapper::getDistance() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
  distance = (duration * 0.034) / 2;

  return distance;
}

void UltrasonicWrapper::displayDistance(Adafruit_SSD1306 &display) {
  distance = getDistance();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Distance: ");
  display.print(distance);
  display.println(" cm");
  display.display();
}
