/**
 * Arduino ESP8266 telnet server with some ansi experiments
 * @author: shawn_a
 */

#include <ESP8266WiFi.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define FASTLED_ESP8266_RAW_PIN_ORDER
#include "FastLED.h"
#define NUM_LEDS 50
#define PIN D8
#define COLOR_ORDER RGB

CRGB leds[NUM_LEDS];
MPU6050 accelgyro;

/* Set these to your desired AP credentials. */
const char* ssid     = "Floor5GIG";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "innovation";     // The password of the Wi-Fi network
int port = 23;

// ansi stuff, could always use printf instead of concat
String ansiSAV  = "\033[s"; // save cursor
String ansiRET  = "\033[u"; // return

String ansiESC  = "\033[2J"; // esc
String ansiCL   = "\033[K";

String ansiEND  = "\033[0m";   // closing tag for styles
String ansiBOLD = "\033[1m";

String ansiBLUF = "\033[34m"; // blue foreground

// declare telnet server (do NOT put in setup())
WiFiServer TelnetServer(port);
WiFiClient Telnet;

#define BUCKETS 100
#define BUCKET_WEIGHT 0.01
#define DAMPNING .95
#define INERTIA .10
#define SCALE 20
#define GAIN .008
float samples[BUCKETS];
int avgPtr = 0;
float avg = 500.0;
double instDisturbance = 0.0;
double disturbance = 0.0;

#define MIN_HUE 140.0
#define MAX_HUE 0.0
#define SCINT_AMP 10.0
#define SCINT_T_FREQ .008
#define SCINT_S_FREQ 1.2
#define MAX_DISTURBANCE 1200.0
#define MIN_BRIGHTNESS 50.0
#define MAX_BRIGHTNESS 255.0

void handleTelnet(){
  if (TelnetServer.hasClient()){
  	// client is connected
    if (!Telnet || !Telnet.connected()){
      if(Telnet) Telnet.stop();          // client disconnected
      Telnet = TelnetServer.available(); // ready for new client
    } else {
      Telnet.print("Z: ");
      TelnetServer.available().stop();  // have client, block new conections
    }
  }

  if (Telnet && Telnet.connected() && Telnet.available()){
    // client input processing
    while(Telnet.available())
      Serial.write(Telnet.read()); // pass through
      // do other stuff with client input here
  }
}

void startAP(){
  WiFi.begin(ssid, password);             // Connect to the network

  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

  Serial.println('\n');
  Serial.println("Connection established!");
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  delay(4000); // ap delay

  TelnetServer.begin();
  Serial.print("Starting telnet server on port " + (String)port);

  // TelnetServer.setNoDelay(true); // ESP BUG ?
  Serial.println();
  delay(100);
}

void setup() {
  FastLED.addLeds<WS2811, PIN>(leds, NUM_LEDS);

  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  delay(1000); // serial delay

  pinMode(0, INPUT);
  for (int i=0; i<BUCKETS; i++) samples[i] = 500;

  startAP();

  Wire.begin();

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(100);
}

void loop() {
  handleTelnet();

  int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float nextSample = GAIN * az;
  instDisturbance *= DAMPNING;
  instDisturbance += abs(nextSample - avg);
  disturbance += INERTIA*(instDisturbance - disturbance);

  avg += (nextSample - samples[avgPtr])*BUCKET_WEIGHT;
  samples[avgPtr] = nextSample;
  avgPtr = (avgPtr + 1) % BUCKETS;

  Telnet.println(ansiSAV + (String)disturbance + ansiCL);
  Telnet.print(ansiBLUF + ansiBOLD);
  for (int i=0; SCALE*i < disturbance ; i++) Telnet.print("*");
  Telnet.print(ansiCL + ansiEND + "\033[1A\033[99D");

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
