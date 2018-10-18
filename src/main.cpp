/**
 * Arduino ESP8266 telnet server with some ansi experiments
 * @author: shawn_a
 */

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "FastLED.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#define NUM_LEDS 50
#define PIN D8
#define COLOR_ORDER RGB
#define INFOPort 50050
#define DISRUPTPort 50060
#define BUCKETS 100
#define STRANDS 15
#define WIFI_TIMEOUT 20000
#define INFO_TIMEOUT 10000
#define DISRUPTIONS 5

CRGB leds[NUM_LEDS];
MPU6050 accelgyro;

/* Set these to your desired AP credentials. */
const char* ssid     = "anthazoa";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "chaartdev";     // The password of the Wi-Fi network


#define BUCKET_WEIGHT 0.01
#define DAMPNING .95
#define INERTIA .10
#define SCALE 20
#define GAIN .008
#define MIN_HUE 140.0
#define MAX_HUE 0.0
#define SCINT_AMP 10.0
#define SCINT_T_FREQ .008
#define SCINT_S_FREQ 1.2
#define MAX_DISTURBANCE 1200.0
#define MIN_BRIGHTNESS 50.0
#define MAX_BRIGHTNESS 255.0
#define MIN_DISRUPTION 600.0
#define DISRUPTION_DELAY 5000


WiFiUDP Udp;

typedef struct Location {
  float x;
  float y;
} Location;

typedef struct Strand {
  uint32_t macAddress;
  Location location;
} Strand;

typedef struct Disruption {
  long time;
  uint8_t level;
} Disruption;

uint32_t myMac = 0;
Strand distanceTable[STRANDS];
Location myLocation;

float samples[BUCKETS];
int avgPtr = 0;
float avg = 500.0;
double instDisturbance = 0.0;
double disturbance = 0.0;
Disruption disruptions[DISRUPTIONS];
int nextDisruption = 0;

float distance(Location loc1, Location loc2) {
  float dx = loc1.x - loc2.x;
  float dy = loc1.y - loc2.y;
  return sqrt(dx*dx + dy*dy);
}

void setupAccelerometer() {
  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  delay(100);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

bool connectToWifi() {
  WiFi.begin(ssid, password);             // Connect to the network

  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");

  long waitStart = millis();

  int i = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - waitStart < WIFI_TIMEOUT) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Timeout connecting to WiFi");
    return true;
  }

  Serial.println('\n');
  Serial.println("Connection established!");
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  delay(4000); // ap delay

  return false;
}

bool retrieveInfoPacket() {
  Udp.begin(INFOPort);
  long waitStart = millis();
  int packetSize = 0;
  for (packetSize = Udp.parsePacket(); packetSize == 0 && millis() - waitStart < INFO_TIMEOUT; packetSize = Udp.parsePacket()) {
    delay(10);
  }

  if (packetSize == 0) {
    Serial.println("Info Timeout");
    return true;
  }

  Udp.read((char *) &distanceTable, sizeof(distanceTable));
  Udp.stop();

  for (int i=0; i<STRANDS; i++) {
    if (myMac == distanceTable[i].macAddress) {
      myLocation = distanceTable[i].location;
      Serial.printf("my location: %f, %f\n", distanceTable[i].location.x, distanceTable[i].location.y);
    }
  }
  for (int i=0; i<STRANDS; i++) {
    Serial.printf("%6x: %f, %f d: %f\n", distanceTable[i].macAddress, distanceTable[i].location.x, distanceTable[i].location.y, distance(myLocation, distanceTable[i].location));
  }
  Serial.println();

  return false;
}

void readDisruption() {
  int packetSize = Udp.parsePacket();
  if (packetSize == 0) {
    return;
  }

  uint8_t level;
  Udp.read((char *) &level, sizeof(level));

  int disruptIndex = (nextDisruption++) % DISRUPTIONS;
  disruptions[disruptIndex].time = millis();
  disruptions[disruptIndex].level = level;

  for (int i=0; i<DISRUPTIONS; i++) {
    Serial.printf("disrupt[%d]: %d, %d\n", i, disruptions[i].time, disruptions[i].level);
  }
}

void setup() {
  FastLED.addLeds<WS2811, PIN>(leds, NUM_LEDS);

  Serial.begin(115200);
  delay(1000); // serial delay

  pinMode(0, INPUT);
  for (int i=0; i<BUCKETS; i++) samples[i] = 500;
  for (int i=0; i<DISRUPTIONS; i++) {
    disruptions[i].time = 0;
    disruptions[i].level = 0;
  }

  setupAccelerometer();

  // determine macAddress as a 32 bit int
  uint8_t macAddress[6] = {0};

  WiFi.macAddress(&macAddress[0]);
  for (int i=0; i<6; i++) {
    Serial.printf("%d:", macAddress[i]);
  }
  Serial.println();

  for(int i=3; i < 6; i++) {
    myMac |= (macAddress[i] << (8*(5-i)));
  }
  Serial.printf("mac: %x\n", myMac);

  // connect to wifi
  if (connectToWifi()) {
    return;
  }

  if (retrieveInfoPacket()) {
    return;
  }

  Udp.begin(DISRUPTPort);
}

void loop() {
  readDisruption();

  int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float nextSample = GAIN * az;
  instDisturbance *= DAMPNING;
  instDisturbance += abs(nextSample - avg);
  disturbance += INERTIA*(instDisturbance - disturbance);

  avg += (nextSample - samples[avgPtr])*BUCKET_WEIGHT;
  samples[avgPtr] = nextSample;
  avgPtr = (avgPtr + 1) % BUCKETS;

  double cDisturbance = min(MAX_DISTURBANCE, max(0.0, disturbance)) / MAX_DISTURBANCE;
  uint8_t brightness = (uint8_t) (MIN_BRIGHTNESS + sqrt(cDisturbance) * (MAX_BRIGHTNESS - MIN_BRIGHTNESS));

  for(int i = 0; i < NUM_LEDS; i++) {
    double scintHue = MIN_HUE + SCINT_AMP * sin(SCINT_T_FREQ*millis() + SCINT_S_FREQ*i);
    uint8_t hue = (uint8_t) (scintHue + cDisturbance * (MAX_HUE - MIN_HUE));
    leds[i] = CHSV(hue, 255, brightness);
  }
  FastLED.show();

  delay(10); // to fast might crash terminals
}
