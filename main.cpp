// base library
#include <Arduino.h>
// web development libraries
#include <WiFi.h>
#include <WebServer.h>
// LCD Adafruit Screen libraries
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
// SD card library
#include <SD.h>

// call the functions
void setDigipot(uint8_t pot, uint8_t value);

// input of network credentials
const char* ssid = "WIFINAME"; // Username
const char* password = "WIFIPASSWORD"; // Password

// define LED pins
#define LED_RED       2        // LED_RED
#define LED_GREEN     4        // LED_GREEN
// define LCD screen pins
/*
#define TFT_CS        5         // CCS
#define TFT_DC        21        // D/C
#define TFT_RST       22        // RST
#define TFT_SCK       18        // SCK
#define TFT_MOSI      23        // MOSI (SI)
*/
// --- digitpot pin definitions
#define DIGI_VOLTAGE_CS 5
#define DIGI_CURRENT_CS 17
// --- global SPI instances ---
SPIClass mySPI(VSPI);
SPIClass digipotSPI(VSPI);

// SD chip select pin
#define SD_CS_PIN     99999         // CS

// ADC pin
#define VINPUT_PIN    36        // Vin (pin 3 == GPIO 36 or VP)
// define width and height of the display
#define TFT_WIDTH     240       // 240x240
#define TFT_HEIGHT    240

// create a web server on port 80
WebServer server(80);

// --- handle the server root and interface of the website ---
void handleRoot() {
  // shows current state of the LEDs
  String stateRED = digitalRead(LED_RED) ? "ON" : "OFF";
  String stateGREEN = digitalRead(LED_GREEN) ? "ON" : "OFF";

  // website html handling
  String html = "<!DOCTYPE html><html><head><title>ESP32 LED Control</title></head>"
                "<body style='text-align:center;font-family:sans-serif;'>"
                "<h2>ESP32 LED Control</h2>"
                "<p>RED LED is currently <strong>" + stateRED + "</strong>.</p>"
                "<p><a href='/red_on'><button style='padding:15px;font-size:18px;'>RED ON</button></a>"
                "<a href='/red_off'><button style='padding:15px;font-size:18px;'>RED OFF</button></a></p>"
                "<p>GREEN LED is currently <strong>" + stateGREEN + "</strong>.</p>"
                "<p><a href='/green_on'><button style='padding:15px;font-size:18px;'>GREEN ON</button></a>"
                "<a href='/green_off'><button style='padding:15px;font-size:18px;'>GREEN OFF</button></a></p>";
  // add digipot controls
  html += "<h2>Digital Potentiometers</h2>"
          "<form action='/vpot'>"
          "<label>Voltage Pot (0-255):</label>"
          "<input type='number' name='value' min='0' max='255'>"
          "<input type='submit' value='Set voltage'>"
          "</form>"
          "<form action='/cpot'>"
          "<label>Current Pot (0-255):</label>"
          "<input type='number' name='value' min='0' max='255'>"
          "<input type='submit' value='Set current'>"
          "</form>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}
// --- handle the ON command for LED_RED ---
void handleRedON() {
  // set led to high
  digitalWrite(LED_RED, HIGH);
  // shows current state of the LED
  server.sendHeader("Location", "/");
  server.send(303); // HTTP redirect to root
}
// --- handle the OFF command for LED_RED ---
void handleRedOFF() {
  // set led to low
  digitalWrite(LED_RED, LOW);
  // shows current state of the LED
  server.sendHeader("Location", "/");
  server.send(303); // HTTP redirect to root
}
// --- handle the ON command for LED_GREEN ---
void handleGreenON() {
  digitalWrite(LED_GREEN, HIGH);
  server.sendHeader("Location", "/");
  server.send(303);
}
// --- handle the OFF command for LED_GREEN ---
void handleGreenOFF() {
  digitalWrite(LED_GREEN, LOW);
  server.sendHeader("Location", "/");
  server.send(303);
}
// --- handle voltage digipot web route ---
void handleVolPot() {
  if (server.hasArg("value")) {
    int value = constrain(server.arg("value").toInt(), 0, 255);
    setDigipot(DIGI_VOLTAGE_CS, value);
    server.send(200, "text/plain", "Voltage Pot set to " + String(value));
  } else {
    server.send(400, "text/plain", "Missing/Wrong value parameter");
  }
}
// --- handle current digipot web route ---
void handleCurrentPot() {
  if (server.hasArg("value")) {
    int value = constrain(server.arg("value").toInt(), 0, 255);
    setDigipot(DIGI_CURRENT_CS, value);
    server.send(200, "text/plain", "Current Pot set to " + String(value));
  } else {
    server.send(400, "text/plain", "Missing/Wrong value parameter");
  }
}

// initialise st7789 controller
// Adafruit_ST7789 tft = Adafruit_ST7789(&mySPI, TFT_CS, TFT_DC, TFT_RST);

// function declarations

// --- setup code for initialisation ---
void setup() {
  // begin communication with ESP32 and device
  Serial.begin(115200);
  /*
  // set onboard (GPIO 23 & 17) as output
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  // turn red & green leds off initially
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);

  // sign into WiFi with your credentials
  WiFi.begin(ssid, password);
  // debugging tool
  Serial.print("Attempting to connect to WiFi.");
  // attempts to connect to WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // output message when connected to WiFi
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
  // define server roots
  server.on("/", handleRoot);
  server.on("/red_on", handleRedON);
  server.on("/red_off", handleRedOFF);
  server.on("/green_on", handleGreenON);
  server.on("/green_off", handleGreenOFF);
  // start server
  server.begin();
  Serial.println("HTTP server started");

  // LCD screen setup
  delay(500);
  Serial.println("About to begin SPI and LCD");
  // SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  mySPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);

  // initialise ST7789 LCD display
  tft.init(TFT_WIDTH, TFT_HEIGHT);   // SPI_MODE0 init (default)

  delay(1000);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLUE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(10, 10);             // set position 10 from left, 10 from top
  tft.setTextWrap(true);
  tft.println("HELLO WORLD");

  Serial.println("Printed Hello onto the screen");
   */
  // setup ADC
  analogReadResolution(12);         // 12-bit resolution: 0-4095
  analogSetAttenuation(ADC_11db);   // ranges from 0-3.3V

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Card Mount Failed");
    return;
  }
  Serial.println("Card Initialised.");
  float VoltageValue;

  File dataFile = SD.open("/log.txt", FILE_APPEND);
  if (dataFile) {
    float sensorValue = analogRead(VINPUT_PIN);
    if (sensorValue >= 3995) {
      VoltageValue = sensorValue/4095 * 95/27 * 3.3;
    } else {
      VoltageValue = (sensorValue + 100)/4095 * 95/27 * 3.3;
    }
    dataFile.println(VoltageValue);
    dataFile.flush();   // Ensure data is saved
    dataFile.close();
  } else {
    Serial.println("Failed to open file for writing.");
  }

  // --- digipot setup ---
  pinMode(DIGI_VOLTAGE_CS, OUTPUT);
  pinMode(DIGI_CURRENT_CS, OUTPUT);
  digitalWrite(DIGI_VOLTAGE_CS, HIGH);
  digitalWrite(DIGI_CURRENT_CS, HIGH);
  digipotSPI.begin(18, -1, 23, -1);   // SCLK = GPIO18, MOSI = GPIO23

  server.on("/vpot", handleVolPot);
  server.on("/cpot", handleCurrentPot);
  // ---------------------

  /*
  // setup TFT
  tft.init(TFT_WIDTH, TFT_HEIGHT);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); // white text on black
  */
}
// --- looping code for real-time logic ---
void loop() {
  // provides functionality to website
  server.handleClient();

  // read raw ADC value
  int adcValue = analogRead(VINPUT_PIN);

  // convert to voltage (assuming 3.3V reference)
  float voltage = (adcValue / 4095) * 12.0; // REMEMBER: voltage divider scales 12V to 3.3V

  // display voltage
  /*
  tft.setCursor(20, 100); // 20 from left, 100 down
  tft.print("Voltage: ");
  tft.print(voltage, 2);  // set 2 decimal places
  tft.print(" V  ");      // extra spaces to overwrite old values
  */
  delay(500);             // update every 0.5s
}

// --- function definitions ---
void setDigipot(uint8_t CS_PIN, uint8_t value) {
  uint8_t command = 0x11; // write to digipot
  digitalWrite(CS_PIN, LOW);
  digipotSPI.transfer(command);
  digipotSPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}
/*
void blinkLED(int pin, int instances, int delay_period) {
  for (int i = 0; i < instances; i++) {
    digitalWrite(pin, HIGH);
    delay(delay_period);
    digitalWrite(pin, LOW);
    delay(delay_period);
  }
}
*/
