#ifndef RobotWebServer_h
#define RobotWebServer_h

#include <WiFi.h>
#include <WebServer.h>

class RobotWebServer
{
public:
  // Constructor
  RobotWebServer(const char *ssid, const char *password, int port = 80);

  // Start the server
  void begin();

  // Handle incoming client requests
  void handleClient();

  // Set robot control callbacks
  void setMoveForwardCallback(void (*func)());
  void setMoveBackwardCallback(void (*func)());
  void setStopCallback(void (*func)());
  void setSpinCallback(void (*func)());

  // Set motor speed (setter and getter)
  void setMotorSpeed(int speed); // Setter
  int getMotorSpeed();           // Getter

  // Get distance from sonar sensor
  float getDistance();
  void setDistance(float dis);

  // Set and get RGB values for a given LED index
  void setRGBValue(int index, uint8_t r, uint8_t g, uint8_t b);
  void getRGBValue(int index, uint8_t &r, uint8_t &g, uint8_t &b);

  // Set and get acceleration values
  void setAcceleration(float x, float y, float z);
  void getAcceleration(float &x, float &y, float &z);

  // Set and get gyroscope values
  void setGyroscope(float x, float y, float z);
  void getGyroscope(float &x, float &y, float &z);

  void setDHTValues(float temperature, float humidity);
  void getDHTValues(float &temperature, float &humidity);

private:
  WebServer server;     // Web server instance
  const char *ssid;     // Wi-Fi SSID
  const char *password; // Wi-Fi password

  int motorSpeed; // Motor speed value stored in the server
  float distance;

  // RGB values for each of the 5 LEDs
  uint8_t rgbValues[5][3]; // Array to store RGB values for 5 LEDs [R, G, B]

  // Acceleration values
  float accelX, accelY, accelZ;

  // Gyroscope values
  float gyroX, gyroY, gyroZ;

  float dhtTemperature;
    float dhtHumidity;


  // Robot control function pointers
  void (*moveForwardCallback)();
  void (*moveBackwardCallback)();
  void (*stopCallback)();
  void (*spinCallback)();

  // Route handlers for web requests
  void handleRoot();         // Serves the HTML interface
  void handleCSS();          // Serves the CSS file
  void handleMoveForward();  // Moves the robot forward
  void handleMoveBackward(); // Moves the robot backward
  void handleStop();         // Stops the robot
  void handleSpin();         // Spins the robot

  // Handle motor speed setting and getting
  void handleSetSpeed(); // Set the motor speed
  void handleGetSpeed(); // Get the current motor speed

  // New route to handle sonar distance
  void handleGetDistance();

  // Handle RGB control
  void handleSetRGB(); // Set the RGB values for a given LED index
  void handleGetRGB(); // Get the RGB values for a given LED index

  // Handle acceleration data
  void handleGetAcceleration(); // Get acceleration data for the graph

  // Handle gyroscope data
  void handleGetGyroscope(); // Get gyroscope data for the graph

  // Handle DHT data
  void handleGetDHT(); 
};

#endif
