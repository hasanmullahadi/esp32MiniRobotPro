#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_MPU6050.h>

#include "RobotWebServer.h"
#include <SPIFFS.h>

// Wi-Fi credentials


const char *ssid ="YourUSSID";
const char *password ="YourPassword";
#define __SERVER_ON__
// Instantiate the web server
#ifdef __SERVER_ON__
RobotWebServer robotServer(ssid, password);
#endif

#include "LoRaCommunication.h"

#define SS_PIN 5
#define RESET_PIN 14
#define DIO0_PIN 16
#define FREQUENCY 433E6
#define LOCAL_ADDRESS 0xAA       // Unique address for this device
#define DESTINATION_ADDRESS 0xFF // Broadcast address
LoRaCommunication lora(SS_PIN, RESET_PIN, DIO0_PIN, FREQUENCY, LOCAL_ADDRESS, DESTINATION_ADDRESS);

// #include "DHTWrapper.h"

// // #define DHT_PIN 34
// // #define DHT_TYPE DHTesp::DHT22
// // DHTWrapper dht(DHT_PIN, DHT_TYPE);

#include "UltrasonicWrapper.h"

#define TRIG_PIN 32
#define ECHO_PIN 39 // DVN is GPIO 36
UltrasonicWrapper ultrasonic(TRIG_PIN, ECHO_PIN);

#include "MotorController.h"

#include "LEDController.h"

#define LED_DRIVER_ADDR 0x40

//LEDController ledController(LED_DRIVER_ADDR);

Adafruit_MPU6050 mpu;

// ADC1 Channels
#define ADC1_CH0 36 // VP (sensor VP)  MIC
#define ADC1_CH1 37 // NOT AVAILABLE
#define ADC1_CH2 38 // NOT AVAILABLE
#define ADC1_CH3 39 //  VN (sensor VN)
#define ADC1_CH4 32 //
#define ADC1_CH5 33 //
#define ADC1_CH6 34 //  are input only
#define ADC1_CH7 35 //  are input only

// ADC2 Channels
#define ADC2_CH0 4  //
#define ADC2_CH1 0  // NOT AVAILABLE
#define ADC2_CH2 2  // should not be used
#define ADC2_CH3 15 // USed for DHT
#define ADC2_CH4 13 //  CT1 Ref
#define ADC2_CH5 12 //  CT2 Ref
#define ADC2_CH6 14 //
#define ADC2_CH7 27 //
#define ADC2_CH8 25 //
#define ADC2_CH9 26 //

#define SCL_PIN 22 // on SCL
#define SDA_PIN 21 // on SDA

int g_motorSpeed = 50; // Global variable to store motor speed (0 to 100%)

// // MotrA
#define AIN1_PIN ADC2_CH0
#define AIN2_PIN ADC2_CH4

// // MotrB
#define BIN1_PIN ADC2_CH3
#define BIN2_PIN 25

// MotrC
#define CIN1_PIN ADC1_CH5
#define CIN2_PIN ADC2_CH8

// MotrD
#define DIN1_PIN ADC2_CH9
#define DIN2_PIN ADC2_CH7

/*
  DRV8837(AIN1_PIN, AIN2_PIN),
  DRV8837(BIN1_PIN, BIN2_PIN), for later when we fix the board V2
*/
MotorController motorController;

#define SCREEN_WIDTH 128         // OLED display width, in pixels
#define SCREEN_HEIGHT 64         // OLED display height, in pixels
#define SSD1306_I2C_ADDRESS 0x3C // or 0x3D, depending on your OLED module

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUM_RGB_LEDS 5

// void updateLEDColorByDistance(float distance)
// {
//     // Map the distance (0-100 cm) to RGB values (Red for close, Blue for far)
//     uint16_t red = 0;
//     uint16_t green = 0;
//     uint16_t blue = 0;

//     if (distance < 50)
//     {
//         // Closer = More Red
//         red = map(distance, 0, 50, 255, 0);
//         green = map(distance, 0, 50, 0, 255);
//     }
//     else
//     {
//         // Farther = More Blue
//         blue = map(distance, 50, 100, 0, 255);
//         green = map(distance, 50, 100, 255, 0);
//     }
//     for (int i = 0; i < NUM_RGB_LEDS; i++)
//         ledController.setRGB(i, red, green, blue); // Update RGB LED based on distance
// }

void testMotorDirection()
{
    if (Serial.available())
    {
        int motorIndex = Serial.parseInt(); // Read the motor index from the Serial input
        if (motorIndex >= 0 && motorIndex < 4)
        {
            // Activate the specified motor in forward motion
            motorController.setMotorSpeed(motorIndex, 200); // 100% speed, forward direction
            delay(1000);                                            // Run for 1 second
            motorController.setMotorSpeed(motorIndex, 0);                          // Stop with deceleration
            Serial.println("Motor " + String(motorIndex) + " forward motion test completed.");
        }
        else
        {
            Serial.println("Invalid motor index. Enter a number between 0 and 3.");
        }
    }
}

void moveForward()
{
    Serial.println("Moving Forward...");
    int speed = robotServer.getMotorSpeed();
    for (int i = 0; i < 4; i++)
    {
        motorController.setMotorSpeed(i, 255); // Set target speed to 100% with acceleration rate of 5
    }
}

void moveDiagonal(bool left)
{
    int speed = robotServer.getMotorSpeed();
    if (left)
    {
        // Diagonal left: Front left and back right forward, back left and front right backward
        motorController.setMotorSpeed(0, 200);  // Motor A forward
        motorController.setMotorSpeed(1, 200);  // Motor C forward
        motorController.setMotorSpeed(2, -200);// Motor B backward
        motorController.setMotorSpeed(3, -200); // Motor D backward
    }
    else
    {
        // Diagonal right: Front right and back left forward, front left and back right backward
        motorController.setMotorSpeed(0, -200);  // Motor A forward
        motorController.setMotorSpeed(1, -200);  // Motor C forward
        motorController.setMotorSpeed(2, 200);// Motor B backward
        motorController.setMotorSpeed(3,200); // Motor D backward
    }
}

void strafe(bool right)
{
    int speed = robotServer.getMotorSpeed();
    if (right)
    {
        // Strafe right: Left wheels move forward, right wheels move backward
        motorController.setMotorSpeed(0, 200);  // Motor A (Front Left) forward
        motorController.setMotorSpeed(1, -200); // Motor B (Back Left) backward
        motorController.setMotorSpeed(2, 200);  // Motor C (Back Right) forward
        motorController.setMotorSpeed(3, -200); // Motor D (Front Right) backward
    }
    else
    {
        // Strafe left: Left wheels move backward, right wheels move forward
        motorController.setMotorSpeed(0, -200); // Motor A (Front Left) backward
        motorController.setMotorSpeed(1, 200);  // Motor B (Back Left) forward
        motorController.setMotorSpeed(2, -200); // Motor C (Back Right) backward
        motorController.setMotorSpeed(3, 200);  // Motor D (Front Right) forward
    }
}

void spin()
{
    Serial.println("Motors Spin...");
    int speed = robotServer.getMotorSpeed();
    // Spin in place: Left wheels move forward, right wheels move backward
    motorController.setMotorSpeed(0, 200); // Motor A (Front Left) forward
    motorController.setMotorSpeed(1, 200);  // Motor B (Back Left) forward
    motorController.setMotorSpeed(2, -200); // Motor C (Back Right) backward
    motorController.setMotorSpeed(3, -200); // Motor D (Front Right) backward
}

void stopMotors()
{
    Serial.println("Stop Moving");
    for (int i = 0; i < 4; i++)
    {
        motorController.stopMotor(i); // Gradually stop with deceleration rate of 5
    }
}

void moveBackward()
{
    Serial.println("Moving backward...");
    int speed = robotServer.getMotorSpeed();
    // Add logic to move the robot backward
    // For example, all motors moving in the reverse direction
    for (int i = 0; i < 4; i++)
    {
         motorController.setMotorSpeed(i, -200); // Example motor control code for reverse
    }
}

#define ObstilcleDistance 40

void obstacleAvoidance(float distance)
{
    static unsigned long lastActionTime = 0; // For tracking time between actions
    static int state = 0;                    // To track the state of obstacle avoidance

    unsigned long currentTime = millis(); // Get the current time

    if (state == 0)
    { // Check if we are moving forward
        if (distance < ObstilcleDistance)
        {                 // Obstacle detected
            stopMotors(); // Stop to evaluate
            lastActionTime = currentTime;
            state = 1; // Move to next state to decide strafe direction
        }
        else
        {
            moveForward(); // No obstacle, keep moving forward
        }
    }
    else if (state == 1)
    { // After stopping, choose to strafe or diagonal
        if (currentTime - lastActionTime > 100)
        { // Wait for 100 ms before taking action
            if (random(0, 2) == 0)
            {
                Serial.println("Strafing right.");
                strafe(true); // Strafe right
            }
            else
            {
                Serial.println("Strafing left.");
                strafe(false); // Strafe left
            }
            lastActionTime = currentTime;
            state = 2; // Move to next state to check if obstacle is still there
        }
    }
    else if (state == 2)
    { // After strafing, check if the path is still blocked
        if (currentTime - lastActionTime > 1000)
        {                                                 // Wait for 1 second before checking again
            stopMotors();                                 // Stop after movement
            float newDistance = ultrasonic.getDistance(); // Check distance again
            if (newDistance < ObstilcleDistance)
            {
                Serial.println("Obstacle still detected. Starting to spin.");
                state = 3; // Move to spinning state
            }
            else
            {
                state = 0; // No obstacle, resume forward movement
            }
            lastActionTime = currentTime;
        }
    }
    else if (state == 3)
    {           // Spinning to find clear path
        spin(); // Keep spinning
        if (currentTime - lastActionTime > 500)
        { // Check every 500 ms for a clear path
            if (ultrasonic.getDistance() > ObstilcleDistance)
            { // Path cleared
                stopMotors();
                state = 0; // Return to moving forward
            }
            lastActionTime = currentTime;
        }
    }
}

void setup()
{

    // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
    // some i2c devices dont like this so much so if you're sharing the bus, watch
    // out for this!
    Wire.setClock(400000);

    Serial.begin(115200);
    while (!Serial)
    {
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }

   // ledController.begin();

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    // MPU setups

    // setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    motorController.begin();

    // for (int i =0;i<4;i++)
    // {
    //   motors[i].setSpeedAndDirection(100, true, 10); // Accelerate to 100% speed forward
    //   delay(500);

    //   motors[i].setSpeedAndDirection(50, false, 10); // Smoothly change to 50% speed in backward direction
    //   delay(500);

    //   motors[i].setSpeedAndDirection(75, true, 10); // Smoothly change to 75% speed forward
    //   delay(100);

    //   motors[i].stop(10); // Decelerate to stop
    // }

    // Initialize the OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }

    // Clear the buffer
    display.clearDisplay();

    // Test message to be displayed
    display.setTextSize(2);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.print(F("I'm 0x"));
    display.println(LOCAL_ADDRESS, HEX);
    display.display(); // Display the message

    // lora.begin(); // Initialize LoRa communication

    // dht.begin();

    ultrasonic.begin();
  //  ledController.setLED(5, 50);

    stopMotors();

    // Wifi stup, need it to be done here so to link the functions funcitonalities

    // Initialize SPIFFS
#ifdef __SERVER_ON__

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Connecting to WiFi..."));
    display.display();

    // Initialize SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("An error has occurred while mounting SPIFFS");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("SPIFFS Mount Failed"));
        display.display();
        return;
    }
    Serial.println("SPIFFS mounted successfully");

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Get IP address
    String ipAddress = WiFi.localIP().toString();
    Serial.print("IP Address: ");
    Serial.println(ipAddress);

    // Display IP address on OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("Connected to WiFi"));
    display.setCursor(0, 20);
    display.println(F("IP Address:"));
    display.setCursor(0, 35);
    display.println(ipAddress);
    display.display();

    // Set up the web server and control functions
    robotServer.setMoveForwardCallback(moveForward);
    robotServer.setMoveBackwardCallback(moveBackward);
    robotServer.setStopCallback(stopMotors);
    robotServer.setSpinCallback(spin);

    // Start the web server
    robotServer.begin();
#endif
    // testfong forward
}

void updateLEDsFromServer()
{
    // Loop through each RGB LED and update its color based on server values
    for (int i = 0; i < NUM_RGB_LEDS; i++)
    {
        uint8_t r, g, b;
        robotServer.getRGBValue(i, r, g, b); // Get the RGB value for the current LED
      //  ledController.setRGB(i, r, g, b);    // Apply the RGB value to the LED
    }
}

void loop()
{
    // lora.sendMessage("MiniRobot !!");
    // // Check if a message was received
    // lora.receiveMessage(display);
    //    display.setTextSize(1); // Normal 1:1 pixel scale
    // dht.displayData(display);
    //   delay(2000); // Update every 2 seconds

    // ultrasonic.displayDistance(display);
    //    delay(500);  // Update every 1 second

    // for (int i = 0; i<NUM_RGB_LEDS;i++)
    // {
    //     ledController.turnOff();
    //     ledController.setRGB(i,500,0,0);
    //     ledController.setRGB((i+1)%NUM_RGB_LEDS,0,500,0);
    //     // ledController.turnOffRGB((i+1)%NUM_RGB_LEDS);

    //     display.clearDisplay();

    //     // Test message to be displayed
    //     display.setTextSize(2); // Normal 1:1 pixel scale
    //     display.setTextColor(SSD1306_WHITE); // Draw white text
    //     display.setCursor(0, 0); // Start at top-left corner
    //     display.println(i);
    //     display.display(); // Display the message
    //     delay(2000);  // Delay to control the speed of the rainbow effect

    // }

    // latest test dive, Hello World Rrobot
    // Measure distance
    // float distance = ultrasonic.getDistance();

    // // Display the distance on the OLED screen
    // ultrasonic.displayDistance(display);

    // // Update RGB LED color based on the distance
    // updateLEDColorByDistance(distance);

    //   // Test motor direction based on serial input
    testMotorDirection();
    // for (int i = 0; i < 4; i++) {
    //     motors[i].update();
    // }

    // obstacleAvoidance( distance);

    // delay(20);  // Update every 500 ms

    // latest test dive, Hello World Rrobot, End

#ifdef __SERVER_ON__

    int speed = robotServer.getMotorSpeed();
    robotServer.setDistance(ultrasonic.getDistance());
    updateLEDsFromServer();


    // float temperature = dht.getTemperature();
    // float humidity = dht.getHumidity();
    // robotServer.setDHTValues(temperature, humidity); // Set the values in the server
    // dht.displayData(display);
    if (mpu.getMotionInterruptStatus())
    {
        /* Get new sensor events with the readings */
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        robotServer.setDHTValues(temp.temperature, 0); // Set the values in the server
        robotServer.setAcceleration(a.acceleration.x, a.acceleration.y, a.acceleration.z);
        robotServer.setGyroscope(g.gyro.x, g.gyro.y, g.gyro.z);
    }
    robotServer.handleClient();

#endif
}
