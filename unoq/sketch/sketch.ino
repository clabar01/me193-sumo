// Sumo distance sensor (MCU side).
//
// Pings the HC-SR04 continuously and keeps the median of the last 3 pings.
// Python reads it over the Bridge and publishes it over MQTT.
// Bridge providers:
//   "get_distance" () -> cm (int), -1 = no target within MAX_CM
//   "set_out" (bool)  -> show / clear "OUT" on the LED matrix
//   "set_debug" (bool) -> when on, every ping is sent to Python with
//                         Bridge.notify("ping_debug", rise_us, echo_us, cm, median)
//
// Wiring: Trig on D2, Echo on D3 through a 1k/2k voltage divider.

// ArduinoGraphics must be included first: it turns on the matrix's text support.
#include <ArduinoGraphics.h>
#include <Arduino_LED_Matrix.h>
#include <Arduino_RouterBridge.h>

// ---------------- Tunable constants ----------------
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
const int MAX_CM = 80;                         // farther than this = no target
const int NO_TARGET = -1;
const int US_PER_CM = 58;                      // echo round trip time per cm
const unsigned long ECHO_START_TIMEOUT_US = 5000;  // echo should rise within this
const unsigned long BUSY_TIMEOUT_MS = 60;      // max wait for the last ping to end
const unsigned long PING_GAP_MS = 10;          // let echoes die down between pings
// ---------------------------------------------------

// Debug codes for echo_us when there's no measured pulse
const long ECHO_NONE = -1;       // echo never went high (no sensor? wiring?)
const long ECHO_TOO_FAR = -2;    // pulse longer than MAX_CM, stopped timing
const long ECHO_BUSY = -3;       // line still high from the last ping

Arduino_LED_Matrix matrix;

const int HISTORY = 3;
int history[HISTORY] = {NO_TARGET, NO_TARGET, NO_TARGET};
int history_pos = 0;

volatile int distance_cm = NO_TARGET;   // median, read by the Bridge thread
volatile bool want_out = false;         // set by the Bridge thread
bool showing_out = false;               // what's on the matrix now
volatile bool debug = false;            // set by the Bridge thread

// Filled in by ping() for debugging
long last_rise_us = 0;   // trigger -> echo going high (a real HC-SR04 waits for its
                         // 8-pulse burst first, so a near-0 value means noise)
long last_echo_us = 0;   // echo pulse length, or one of the ECHO_* codes

// One ping. Returns cm, or NO_TARGET if nothing within MAX_CM.
// We time the echo with micros() ourselves: on this core pulseIn() takes its
// timeout in milliseconds and its timing resolution is coarse.
int ping() {
  last_rise_us = 0;

  // With no target the sensor holds Echo high ~38 ms; wait for it to finish
  // (sleeping 1 ms at a time so the Bridge thread can run).
  unsigned long waited = 0;
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (waited++ >= BUSY_TIMEOUT_MS) { last_echo_us = ECHO_BUSY; return NO_TARGET; }
    delay(1);
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long start = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
    if (micros() - start > ECHO_START_TIMEOUT_US) { last_echo_us = ECHO_NONE; return NO_TARGET; }
  }

  // Stop timing as soon as the echo is longer than MAX_CM: no need to wait
  // for far-away echoes (the next ping() waits for the line to go low).
  unsigned long rise = micros();
  last_rise_us = rise - start;
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - rise > (unsigned long)MAX_CM * US_PER_CM) { last_echo_us = ECHO_TOO_FAR; return NO_TARGET; }
  }
  last_echo_us = micros() - rise;
  return last_echo_us / US_PER_CM;
}

// Median of the last 3 pings. NO_TARGET counts as "very far", so one missed
// echo doesn't hide a real target, but two do.
int median_of_history() {
  int a[HISTORY];
  for (int i = 0; i < HISTORY; i++) {
    a[i] = (history[i] == NO_TARGET) ? MAX_CM + 1 : history[i];
  }
  // sort 3 values
  if (a[0] > a[1]) { int t = a[0]; a[0] = a[1]; a[1] = t; }
  if (a[1] > a[2]) { int t = a[1]; a[1] = a[2]; a[2] = t; }
  if (a[0] > a[1]) { int t = a[0]; a[0] = a[1]; a[1] = t; }
  return (a[1] > MAX_CM) ? NO_TARGET : a[1];
}

int get_distance() {
  return distance_cm;
}

void set_out(bool out) {
  want_out = out;   // drawn from loop(), so only one thread touches the matrix
}

void set_debug(bool on) {
  debug = on;
}

void draw_matrix() {
  matrix.clear();
  if (want_out) {
    matrix.textFont(Font_4x6);           // 3 letters x 4 px = 12 px: fits 13 wide
    matrix.beginText(0, 1, 127, 0, 0);   // x, y, then r, g, b (any nonzero = on)
    matrix.print("OUT");
    matrix.endText();                    // no scroll: stays on screen
  }
  showing_out = want_out;
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  matrix.begin();
  matrix.clear();

  Bridge.begin();
  Bridge.provide("get_distance", get_distance);
  Bridge.provide("set_out", set_out);
  Bridge.provide("set_debug", set_debug);
}

void loop() {
  int cm = ping();
  history[history_pos] = cm;
  history_pos = (history_pos + 1) % HISTORY;
  distance_cm = median_of_history();

  if (debug) {
    Bridge.notify("ping_debug", (int32_t)last_rise_us, (int32_t)last_echo_us,
                  (int32_t)cm, (int32_t)distance_cm);
  }

  if (want_out != showing_out) {
    draw_matrix();
  }
  delay(PING_GAP_MS);
}
