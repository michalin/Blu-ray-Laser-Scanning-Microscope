/* Copyright (C) 2022  Doctor Volt

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// #include <Arduino.h>
#include <ArduinoJson.h>
//#include "driver/ledc.h"

#define PINSTEPY_M1 18
#define PINSTEPY_M2 19
#define PINSTEPY_SLP 21
#define PINSTEPY_DIR 22
#define PINSTEPY_STP 23
#define PINPWM_SCAN_X 25
#define PINPOL_SCAN_X 26
#define PINPWN_FOCUS 27

#define PIN_SUM 32
#define PIN_FES 33
#define PINPWM_LSR 13
#define ACTIVE LOW
#define PASSIVE HIGH
#define MAX_SAMPLES 16
enum CMD
{
  CMD_IDLE,
  SCAN,
  STOP,
  FOCUS,
  GO_UP,
  GO_DN,
  GO_STOP
};
#define DIRY_UP 1
#define DIRY_DN 0
#define PWM_SCN_MAX_BITS 9
#define PWM_FOC_BITS 10
#define PWM_LSR_BITS 8


const char *ssid = "WLAN-KEY";
const char *password = "passphrase";
// const int ledPin = LED_BUILTIN;
const int PWMScnFreq = 40000000 >> PWM_SCN_MAX_BITS; // PWM Frequency for X-Scan for max. resolution
const int PWMChanScn = 0;     // PWM Channel for X-Scan
const int PWMFocFreq = 40000000 >> PWM_FOC_BITS;
const int PWMChanFoc = 2; // PWM Channel for focus
const int PWMLsrFreq = 100000;
const int PWMChanLsr = 4; // PWM Channel for laser
const int PWMChanSld = 6; // PWM Channel for manual sled positioning
int PWMRes;               // = (1 << PWMResBits) - 1;
int resolution;           // = 2 * (1 << PWMResBits) - 1;
// int YRes;
int scan_speed;
int nsamples; // Samples per dot for averaging
struct /*__attribute__((__packed__))*/ Point
{
  uint16_t C1;
  // uint16_t C2;
};
typedef uint16_t CVPoint;
CVPoint *linebuf;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t scanSemaphore;
TaskHandle_t taskHandle;

// Function declarations for web.cpp
void web_init(const char *ssid, const char *password);
void web_write(const char *data);
// void web_write_event(const char *event);
void web_cleanup();

// void scan_(void*);

// Macro to configure the STSPIN220 for 64 microsteps
#define STS220SET                      \
  digitalWrite(PINSTEPY_SLP, ACTIVE);  \
  digitalWrite(PINSTEPY_STP, LOW);     \
  digitalWrite(PINSTEPY_DIR, HIGH);    \
  digitalWrite(PINSTEPY_SLP, PASSIVE); \
  vTaskDelay(10);

void setXscan(int x) // Position of lens
{
  int d;
  if (x >= 0)
  {
    d = PWMRes - x - resolution / 30; // The "resolution/xx fixes the glitch arond the zero position"
    digitalWrite(PINPOL_SCAN_X, HIGH);
  }
  else
  {
    d = -x;
    digitalWrite(PINPOL_SCAN_X, LOW);
  }
  //  int d = (x >= 0) ? PWMRes - x : -x;
  //Serial.printf("x: %d, d: %d\n", x, d);
  //  digitalWrite(PINPOL_SCAN_X, (x >= 0));
  ledcWrite(PWMChanScn, d);
}

void scanX(int speed, CVPoint *linebuf)
{
  for (int i = 1; i <= 16; i++) // Move back lens smoothly before scanning to prevent bouncing
  {
    setXscan(-i * resolution / 32);
    // Serial.printf("%d", -i*PWMRes/8);
    vTaskDelay(10);
  }
  for (int scanX = -resolution / 2; scanX <= resolution / 2 - 1; scanX++)
  {
    setXscan(scanX);
    delayMicroseconds(1550 - 100 * speed);
    int c1 = 0;
    for (int i = 0; i < nsamples; i++)
    {
      c1 += analogRead(PIN_SUM);
      // c2 += analogRead(PIN_FES);
    }
    linebuf[resolution / 2 + scanX] = 16 * (c1 / nsamples); // Sum
  }
  setXscan(0); //Neutral position
}

/* This must be started as task to prevent triggering the async_tcp watchdog*/
void scan(void *task_break)
{
  const int nSteps = 1280;   // Will do one revolution (20*64 microsteps)
  static int microsteps = 0; // Number of microsteps passed
  const int pre_steps = 200;  // Number of steps the motor takes before scanning for the leadscrew to gain grip.
  Serial.println("Start scanning");
  STS220SET

  int yskip = 1 + floor(nSteps / (PWMRes + 1));

  digitalWrite(PINSTEPY_DIR, DIRY_UP);
  for (int i = 0; i < yskip * resolution / 2; i++)
  {
    digitalWrite(PINSTEPY_STP, HIGH);
    vTaskDelay(1);
    digitalWrite(PINSTEPY_STP, LOW);
    vTaskDelay(1);
    microsteps--;
  }
  // Advance the sled a bit, so that the leadscrew gets grip.
  digitalWrite(PINSTEPY_DIR, DIRY_DN);
  for (int i = 0; i < pre_steps / yskip; i++)
  {
    digitalWrite(PINSTEPY_STP, HIGH);
    vTaskDelay(1);
    digitalWrite(PINSTEPY_STP, LOW);
    vTaskDelay(1);
    microsteps++;
  }

  for (int i = 0; (i < resolution) && (!*((bool *)task_break)); i++)
  {
    scanX(scan_speed, linebuf);     // Scan line Forwards
    for (int i = 0; i < yskip; i++) // Forward by ySkip microsteps
    {
      digitalWrite(PINSTEPY_STP, HIGH);
      vTaskDelay(1);
      digitalWrite(PINSTEPY_STP, LOW);
      vTaskDelay(1);
      microsteps++;
    }
    xSemaphoreGive(scanSemaphore); // This triggers sending of the line in the loop() function
    vTaskSuspend(NULL);            // Suspend task during data transfer
    //if(*(bool*)task_break) break;
  }
  Serial.printf("Scan completed: %d\n", *((bool *)task_break));

  // Move sled back to intitial y position
  digitalWrite(PINSTEPY_DIR, DIRY_UP);
  for (int i = 0; i < microsteps; i++)
  {
    digitalWrite(PINSTEPY_STP, HIGH);
    delayMicroseconds(200);
    digitalWrite(PINSTEPY_STP, LOW);
    delayMicroseconds(200);
  }
  microsteps = 0;
  digitalWrite(PINSTEPY_SLP, ACTIVE);
  digitalWrite(PINPWM_SCAN_X, LOW);
  ledcWrite(PWMChanScn, 0);
  digitalWrite(PINPOL_SCAN_X, LOW);
  web_write("stop");
  vTaskDelete(NULL);
}

void handleWebMessage(const char *data, size_t len)
{
  DynamicJsonDocument jdoc(1024);
  deserializeJson(jdoc, data);
  static bool taskBreak;

  if (nsamples != (int)jdoc["nsamples"])
  {
    nsamples = jdoc["nsamples"];
    // Serial.printf("Number of samples per dot: %d\n", nsamples);
  }
  if (scan_speed != (int)jdoc["speed"])
  {
    scan_speed = jdoc["speed"];
    // Serial.printf("speed: %d\n", scan_speed);
  }
  static int pwmbits;
  if (pwmbits != (int)jdoc["pwmbits"])
  {
    pwmbits = jdoc["pwmbits"];
    Serial.printf("pwmbits: %d\n", pwmbits);
    ledcSetup(PWMChanScn, PWMScnFreq, pwmbits);
    PWMRes = (1 << pwmbits) - 1;
    resolution = 2 * (1 << pwmbits) - (1 << pwmbits - 3);
    Serial.printf("Resolution: %d\n", resolution);
  }
  static int focus;
  if (focus != (int)jdoc["focus"])
  {
    focus = jdoc["focus"];
    // Serial.printf("focus: %d\n", focus);
    ledcWrite(PWMChanFoc, focus);
  }
  static int laser;
  if (laser != (int)jdoc["laser"])
  {
    laser = jdoc["laser"];
    // Serial.printf("laser: %d\n", laser);
    ledcWrite(PWMChanLsr, laser);
  }
  static int xcoil;
  if (xcoil != (int)jdoc["xcoil"])
  {
    xcoil = jdoc["xcoil"];
    Serial.printf("xcoil: %d\n", xcoil);
    setXscan(xcoil);
  }

  switch ((int)jdoc["cmd"])
  {
  case CMD::SCAN:
    taskBreak = 0;
    xTaskCreate(scan, "scan", 16000, &taskBreak, 1, &taskHandle);
    break;
  case CMD::STOP:
    taskBreak = 1;
    break;
  case CMD::GO_UP:
  case CMD::GO_DN:
    // Serial.print("Move Sled ");
    STS220SET;
    (int)jdoc["cmd"] == GO_UP
        ? digitalWrite(PINSTEPY_DIR, DIRY_UP)
        : digitalWrite(PINSTEPY_DIR, DIRY_DN);
    ledcAttachPin(PINSTEPY_STP, PWMChanSld);
    ledcWriteTone(PWMChanSld, 100 * (scan_speed + 1));
    break;
  case CMD::GO_STOP:
    // Serial.println("Stop sled");
    ledcDetachPin(PINSTEPY_STP);
    digitalWrite(PINSTEPY_SLP, ACTIVE);
    break;
  }
}

/*void ARDUINO_ISR_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}*/

void setup()
{
  Serial.begin(115200);
  // digitalWrite(ledPin, LOW);
  scanSemaphore = xSemaphoreCreateBinary();
  // hw_timer_t *timer = timerBegin(0, 80, true);
  // timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, 100000, true);
  // timerAlarmEnable(timer);
  linebuf = (CVPoint*)calloc(sizeof(CVPoint), 2 << PWM_SCN_MAX_BITS);

  pinMode(PINSTEPY_SLP, OUTPUT);
  digitalWrite(PINSTEPY_SLP, ACTIVE);

  pinMode(PINSTEPY_M1, OUTPUT);
  pinMode(PINSTEPY_M2, OUTPUT);
  pinMode(PINSTEPY_STP, OUTPUT);
  pinMode(PINSTEPY_DIR, OUTPUT);

  // Set M1 and M2 for motor driver
  digitalWrite(PINSTEPY_M1, HIGH);
  digitalWrite(PINSTEPY_M2, HIGH);

  pinMode(PINPOL_SCAN_X, OUTPUT);
  adcAttachPin(PIN_SUM);
  adcAttachPin(PIN_FES);

  ledcSetup(PWMChanFoc, PWMFocFreq, PWM_FOC_BITS);
  ledcSetup(PWMChanLsr, PWMLsrFreq, PWM_LSR_BITS);
  ledcSetup(PWMChanSld, 0, 1);
  ledcAttachPin(PINPWN_FOCUS, PWMChanFoc);
  ledcAttachPin(PINPWM_LSR, PWMChanLsr);
  ledcAttachPin(PINPWM_SCAN_X, PWMChanScn);
  // handleWebMessage("{\"speed\":2,\"pwmbits\":6,\"yres\":128,\"cmd\":1}", 35);
  web_init(ssid, password);
}

void loop()
{
  char buf[16];
  if (xSemaphoreTake(scanSemaphore, 100) == pdTRUE)
  {
    String line;
    for (int i = 0; i < resolution; i++)
    {
      // sprintf(buf, "%d,%d,", linebuf[i].C1, linebuf[i].C2);
      sprintf(buf, "%d,", linebuf[i]);
      line += String(buf);
    }
    line[line.length() - 1] = 0; // Remove comma */
    // Serial.println(line);
    web_write(line.c_str());
    vTaskResume(taskHandle);
  }
  else
  {
    sprintf(buf, "{\"SUM\": %d}", analogRead(PIN_SUM));
    web_write(buf);
  }
  // Serial.println("loop");
  web_cleanup();
}

// Create test pattern
void scan_(void *task_break)
{
  Serial.printf("xres: %d, yres: %d, PWMRes: %d\n", resolution, resolution, PWMRes);
  for (int i = 0; (i < resolution) && (!*((bool *)task_break)); i++)
  {
    for (int scanX = -resolution / 2; scanX <= resolution / 2; scanX++)
    {
      if (scanX % 16)
        linebuf[resolution / 2 + scanX] = 256 * (resolution / 2 + scanX); // Sum
    }

    xSemaphoreGive(scanSemaphore); // This triggers sending of the line in the loop() function
    vTaskSuspend(NULL);
  }
  web_write("stop");
  vTaskDelete(NULL);
}
