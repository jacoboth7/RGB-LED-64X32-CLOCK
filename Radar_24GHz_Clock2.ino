#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <PxMatrix.h>
#include "SPIFFS.h"
#include <ld2410.h>



// Pin definitions for ESP32
#define P_LAT 22
#define P_A 19
#define P_B 23
#define P_C 18
#define P_D 5
#define P_OE 21

#define ESP32

// Define display size
#define MATRIX_WIDTH 64
#define MATRIX_HEIGHT 32
#define NO_FILES 20
#define MAX_BUFFER_SIZE 1024
char iBuffer[NO_FILES * MATRIX_WIDTH][MATRIX_HEIGHT];

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

bool ChangedToIdle = false;

unsigned long ms_current = 0;
unsigned long ms_previous = 0;
unsigned long ms_animation_max_duration = 400000;  // 10 seconds
unsigned long next_frame = 0;

// This defines the 'on' time of the display is us. The larger this number,
// the brighter the display. If too large the ESP will crash
uint8_t display_draw_time = 50;  //30-60 is usually fine

PxMATRIX display(MATRIX_WIDTH, MATRIX_HEIGHT, P_LAT, P_OE, P_A, P_B, P_C, P_D);

#include <FastLED.h>  // Aurora needs fastled

// #include "Effects.h"
// Effects effects;

// #include "Drawable.h"
// #include "Playlist.h"

// #include "Patterns.h"
// Patterns patterns;

// Some standard colors
uint16_t myRED = display.color565(255, 0, 0);
uint16_t myGREEN = display.color565(0, 255, 0);
uint16_t myBLUE = display.color565(0, 0, 255);
uint16_t myWHITE = display.color565(255, 255, 255);
uint16_t myYELLOW = display.color565(255, 255, 0);
uint16_t myCYAN = display.color565(0, 255, 255);
uint16_t myMAGENTA = display.color565(255, 0, 255);
uint16_t myBLACK = display.color565(0, 0, 0);

uint16_t myCOLORS[8] = { myRED, myGREEN, myBLUE, myWHITE, myYELLOW, myCYAN, myMAGENTA, myBLACK };


// Wi-Fi credentials
const char *ssid = "Botha-WiFi";      //"Schlebusch WiFi"; // //"Galaxy A54 5G EA55";
const char *password = "Botha12345";  //"Attiesch1997"; //

// NTP and time variables
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7200, 60000);  // Adjusted to +2 hours

byte _Second = 0, _Minute = 0, _Hour = 0;
unsigned long _Epoch;

// SEGMENTS
#include "Digit.h"
Digit digit0(&display, 2, 64 - 0 - 9 * 1, -7, display.color565(255, 0, 0));
Digit digit1(&display, 2, 64 - 1 - 9 * 2, -7, display.color565(255, 0, 0));
Digit digit2(&display, 2, 62 - 2 - 9 * 3, -7, display.color565(255, 0, 0));
Digit digit3(&display, 2, 62 - 3 - 9 * 4, -7, display.color565(255, 0, 0));
Digit digit4(&display, 2, 60 - 4 - 9 * 5, -7, display.color565(255, 0, 0));
Digit digit5(&display, 2, 60 - 5 - 9 * 6, -7, display.color565(255, 0, 0));

// State machine definitions
enum State {
  IDLE,
  DETECT_MOVING_TARGET,
  DEMO
};

State currentState = IDLE;
unsigned long stateStartTime = 0;
unsigned long TimeInCloseRange = 0;
unsigned long StartTimeInCloseRange = 0;
unsigned long TimeInIDLE = 0;
bool targetDetected = false, latch = false;

uint32_t lastReading = 0;
uint32_t TimeSinceMovingTarget = 0;
bool radarConnected = false;
bool NewInIDLE = true;

ld2410 radar;

#if defined(ESP32)
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 32
#define RADAR_TX_PIN 33
#endif

void IRAM_ATTR display_updater() {
  portENTER_CRITICAL_ISR(&timerMux);
  display.display(20);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void display_update_enable(bool is_enable) {

#ifdef ESP8266
  if (is_enable)
    display_ticker.attach(0.001, display_updater);
  else
    display_ticker.detach();
#endif

#ifdef ESP32
  if (is_enable) {
    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &display_updater);
    timerWrite(timer, 0);              //timerAlarmWrite(timer, 2000, true);
    timerAlarm(timer, 2000, true, 0);  //timerAlarmEnable(timer);
   
  } else {
    
  }
#endif
}

void setup() {

  MONITOR_SERIAL.begin(115200);
  //Set - up for the demo SPIFF file :
  //Mount the SPIFFS file
  // Wi-Fi setup
  WiFi.begin(ssid, password);
  Serial.print("Connecting.");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("connected");
  timeClient.begin();
  delay(100);

  // Display setup
  display.begin(16);
  display.setFastUpdate(true);
  display.clearDisplay();
  display.setTextColor(display.color565(255, 255, 255));
  display.setMuxPattern(BINARY);  //;
  // Rotate display
  //display.setRotate(2);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &display_updater);
  timerWrite(timer, 0);              //timerAlarmWrite(timer, 2000, true);
  timerAlarm(timer, 2000, true, 0);  //timerAlarmEnable(timer);
  // timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(timer, &display_updater, true);
  // timerAlarmWrite(timer, 2000, true);
  // timerAlarmEnable(timer);

  // Radar setup

#if defined(ESP32)
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
#endif
  delay(500);
  if (radar.begin(RADAR_SERIAL)) {
    MONITOR_SERIAL.println(F("LD2410 radar sensor initialised"));
  } else {
    MONITOR_SERIAL.println(F("Radar sensor not connected"));
  }

  delay(500);

  TimeInIDLE = millis();
}

// void listPatterns() {
//   patterns.listPatterns();
// }


void DigitalClock() {
   display.clearDisplay();
  TimeSinceMovingTarget = millis();
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    unsigned long unix_epoch = timeClient.getEpochTime();
    if (unix_epoch != _Epoch) {  //Serial.print("ChangedToIdle"); // Serial.println(ChangedToIdle);
      int Second = second(unix_epoch);
      int Minute = minute(unix_epoch);
      int Hour = hour(unix_epoch);
      //if ((ChangedToIdle == true) || (_Epoch == 0))  // If we didn't have a previous time. Just draw it without morphing.
      {
        digit1.DrawColon(display.color565(255, 255, 255));
        //display.showBuffer();
        digit3.DrawColon(display.color565(255, 255, 255));

        Serial.println("Epoch = 0");
        digit0.Draw(Second % 10);
        //display.showBuffer();
        digit1.Draw(Second / 10);
        //display.showBuffer();
        digit2.Draw(Minute % 10);
        // display.showBuffer();
        digit3.Draw(Minute / 10);
        //display.showBuffer();
        digit4.Draw(Hour % 10);
        //showBuffer();
        if (Hour >= 10) {
          digit5.Draw(Hour / 10);
          //display.showBuffer();
        }
       
        delay(500);
        digit1.DrawColon(display.color565(255, 255, 255));
        //display.showBuffer();
        digit3.DrawColon(display.color565(255, 255, 255));
         display.showBuffer();
        //display.showBuffer();
      } 
       _Epoch = unix_epoch;
    }
  }
}

void loop() {
  unsigned long currentTime = millis();
  radar.read();

  switch (currentState) {
    case IDLE:

      // Serial.println("State: IDLE");
      if (NewInIDLE == true) {
        NewInIDLE = false;
        TimeInIDLE = millis();
        display.clearDisplay();
        delay(50);
      }

      DigitalClock();

      break;

    case DETECT_MOVING_TARGET:
      //Serial.println("State: DETECT_MOVING_TARGET");

      if (radar.isConnected() && millis() - lastReading > 100) {
        lastReading = millis();
        if (radar.movingTargetDetected()) {
          stateStartTime = millis();
          targetDetected = true;
          //currentState = DEMO;
          display.clearDisplay();
          display.showBuffer();
          //delay(50);
          float MovingTargetRange = radar.movingTargetDistance() / 100.0;
          String number = String(MovingTargetRange, 1);
          int16_t x1, y1;
          uint16_t w, h;

          display.getTextBounds(number, 0, 0, &x1, &y1, &w, &h);
          display.setTextSize(1);
          display.setCursor((MATRIX_WIDTH - w + 2) / 2, (MATRIX_HEIGHT - h) / 2);
          display.print(number);
          display.showBuffer();

          uint16_t color = display.color565(255, 0, 0);
          int number_int = (int)MovingTargetRange;
          for (int i = 1; i < (7 - number_int); i++) {
            display.drawCircle(MATRIX_WIDTH / 2, MATRIX_HEIGHT / 2, 9 + i, color);
          }
          display.showBuffer();

          if (MovingTargetRange < 1.5) {
            if (latch == false) {
              StartTimeInCloseRange = millis();
              latch = true;
            }
            TimeInCloseRange = millis() - StartTimeInCloseRange;
          } else
            TimeInCloseRange = 0;

        } else if (millis() - stateStartTime > 5000) {
          currentState = IDLE;
          ChangedToIdle = true;
          display.clearDisplay();
          display.showBuffer();
          Serial.println("Change State to IDLE");
          //TimeInIDLE = millis();
          NewInIDLE = true;
          stateStartTime = millis();
        }
      }

      break;

    case DEMO:
      //Serial.println("State: DEMO");

      //Demo();

      break;
  }
}  // End of loop
