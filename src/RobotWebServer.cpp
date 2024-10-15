#include "RobotWebServer.h"
#include <SPIFFS.h>

RobotWebServer::RobotWebServer(const char *ssid, const char *password, int port)
    : server(port), ssid(ssid), password(password), moveForwardCallback(nullptr), moveBackwardCallback(nullptr), stopCallback(nullptr), spinCallback(nullptr)
{
}

void RobotWebServer::begin()
{
    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Define server routes
    server.on("/", std::bind(&RobotWebServer::handleRoot, this));
    server.on("/style.css", std::bind(&RobotWebServer::handleCSS, this));
    server.on("/move-forward", std::bind(&RobotWebServer::handleMoveForward, this));
    server.on("/move-backward", std::bind(&RobotWebServer::handleMoveBackward, this));
    server.on("/stop", std::bind(&RobotWebServer::handleStop, this));
    server.on("/spin", std::bind(&RobotWebServer::handleSpin, this));

    // New route for speed control
    // Routes for motor speed control
    server.on("/set-speed", std::bind(&RobotWebServer::handleSetSpeed, this)); // Setter route
    server.on("/get-speed", std::bind(&RobotWebServer::handleGetSpeed, this)); // Getter route

    // Route for distance reading
    server.on("/get-distance", std::bind(&RobotWebServer::handleGetDistance, this)); // Distance route

    // Route for RGB control
    server.on("/set-rgb", std::bind(&RobotWebServer::handleSetRGB, this));
    server.on("/get-rgb", std::bind(&RobotWebServer::handleGetRGB, this));

    // Routes for acceleration data
    server.on("/get-acceleration", std::bind(&RobotWebServer::handleGetAcceleration, this));

    // Routes for gyroscope data
    server.on("/get-gyroscope", std::bind(&RobotWebServer::handleGetGyroscope, this));

    server.on("/get-dht", std::bind(&RobotWebServer::handleGetDHT, this));

    server.begin(); // Start the server
    Serial.println("Server started");
}


void RobotWebServer::setDHTValues(float temperature, float humidity) {
    dhtTemperature = temperature;
    dhtHumidity = humidity;
}

void RobotWebServer::getDHTValues(float &temperature, float &humidity) {
    temperature = dhtTemperature;
    humidity = dhtHumidity;
}

// Create a new route to handle the DHT data request
void RobotWebServer::handleGetDHT() {
    String json = "{\"temperature\":" + String(dhtTemperature) + ",\"humidity\":" + String(dhtHumidity) + "}";
    server.send(200, "application/json", json);
}



// Set acceleration data
void RobotWebServer::setAcceleration(float x, float y, float z)
{
    accelX = x;
    accelY = y;
    accelZ = z;
}

// Get acceleration data
void RobotWebServer::getAcceleration(float &x, float &y, float &z)
{
    x = accelX;
    y = accelY;
    z = accelZ;
}

// Handle getting acceleration data
void RobotWebServer::handleGetAcceleration()
{
    String json = "{\"x\":" + String(accelX) + ",\"y\":" + String(accelY) + ",\"z\":" + String(accelZ) + "}";
    server.send(200, "application/json", json);
}

// Set gyroscope data
void RobotWebServer::setGyroscope(float x, float y, float z)
{
    gyroX = x;
    gyroY = y;
    gyroZ = z;
}

// Get gyroscope data
void RobotWebServer::getGyroscope(float &x, float &y, float &z)
{
    x = gyroX;
    y = gyroY;
    z = gyroZ;
}

// Handle getting gyroscope data
void RobotWebServer::handleGetGyroscope()
{
    String json = "{\"x\":" + String(gyroX) + ",\"y\":" + String(gyroY) + ",\"z\":" + String(gyroZ) + "}";
    server.send(200, "application/json", json);
}

// Setter for RGB values
void RobotWebServer::setRGBValue(int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= 0 && index < 5)
    {
        rgbValues[index][0] = r;
        rgbValues[index][1] = g;
        rgbValues[index][2] = b;
    }
}

// Getter for RGB values
void RobotWebServer::getRGBValue(int index, uint8_t &r, uint8_t &g, uint8_t &b)
{
    if (index >= 0 && index < 5)
    {
        r = rgbValues[index][0];
        g = rgbValues[index][1];
        b = rgbValues[index][2];
    }
}

// Handle setting RGB values from the web interface
void RobotWebServer::handleSetRGB()
{
    if (server.hasArg("index") && server.hasArg("color"))
    {
        int index = server.arg("index").toInt();
        String hexColor = server.arg("color");

        // Convert hex color to RGB values
        long colorValue = strtol(hexColor.c_str(), nullptr, 16);
        uint8_t r = (colorValue >> 16) & 0xFF;
        uint8_t g = (colorValue >> 8) & 0xFF;
        uint8_t b = colorValue & 0xFF;

        // Set the RGB value for the LED index
        setRGBValue(index, r, g, b);

        server.send(200, "text/plain", "RGB color updated");
    }
    else
    {
        server.send(400, "text/plain", "Missing index or color");
    }
}

// Handle getting RGB values for an LED index
void RobotWebServer::handleGetRGB()
{
    if (server.hasArg("index"))
    {
        int index = server.arg("index").toInt();
        uint8_t r, g, b;
        getRGBValue(index, r, g, b);

        // Respond with the current RGB values
        String rgbStr = String(r) + "," + String(g) + "," + String(b);
        server.send(200, "text/plain", rgbStr);
    }
    else
    {
        server.send(400, "text/plain", "Missing index");
    }
}

// Handler for returning the sonar distance
void RobotWebServer::handleGetDistance()
{

    String distanceStr = String(distance);
    server.send(200, "text/plain", distanceStr);
}

void RobotWebServer::setDistance(float dis)
{
    distance = dis;
}

// Setter for motor speed
void RobotWebServer::setMotorSpeed(int speed)
{
    motorSpeed = speed;
    Serial.print("Motor speed set to: ");
    Serial.println(motorSpeed);
}

// Getter for motor speed
int RobotWebServer::getMotorSpeed()
{
    return motorSpeed;
}

// Handle setting motor speed from web request
void RobotWebServer::handleSetSpeed()
{
    if (server.hasArg("value"))
    {
        int speed = server.arg("value").toInt();
        setMotorSpeed(speed); // Call the setter
        server.send(200, "text/plain", "Speed updated");
    }
    else
    {
        server.send(400, "text/plain", "Missing speed value");
    }
}

// Handle getting motor speed from web request
void RobotWebServer::handleGetSpeed()
{
    String speedValue = String(getMotorSpeed()); // Call the getter
    server.send(200, "text/plain", speedValue);
}

void RobotWebServer::handleClient()
{
    server.handleClient(); // Handle incoming HTTP requests
}

// Serve the HTML file from SPIFFS
void RobotWebServer::handleRoot()
{
    File file = SPIFFS.open("/index.html", "r");
    if (!file)
    {
        server.send(404, "text/plain", "File Not Found");
        return;
    }

    String html = file.readString();
    server.send(200, "text/html", html);
    file.close();
}

// Serve the CSS file from SPIFFS
void RobotWebServer::handleCSS()
{
    File file = SPIFFS.open("/style.css", "r");
    if (!file)
    {
        server.send(404, "text/plain", "File Not Found");
        return;
    }

    String css = file.readString();
    server.send(200, "text/css", css);
    file.close();
}

// Route handlers for movement commands
void RobotWebServer::handleMoveForward()
{
    if (moveForwardCallback)
    {
        moveForwardCallback(); // Call the linked function
    }
    server.send(200, "text/plain", "Moving forward");
}

void RobotWebServer::handleMoveBackward()
{
    if (moveBackwardCallback)
    {
        moveBackwardCallback(); // Call the linked function
    }
    server.send(200, "text/plain", "Moving backward");
}

void RobotWebServer::handleStop()
{
    if (stopCallback)
    {
        stopCallback(); // Call the linked function
    }
    server.send(200, "text/plain", "Stopping");
}

void RobotWebServer::handleSpin()
{
    if (spinCallback)
    {
        spinCallback(); // Call the linked function
    }
    server.send(200, "text/plain", "Spinning");
}

// Set callback functions
void RobotWebServer::setMoveForwardCallback(void (*func)())
{
    moveForwardCallback = func;
}

void RobotWebServer::setMoveBackwardCallback(void (*func)())
{
    moveBackwardCallback = func;
}

void RobotWebServer::setStopCallback(void (*func)())
{
    stopCallback = func;
}

void RobotWebServer::setSpinCallback(void (*func)())
{
    spinCallback = func;
}
