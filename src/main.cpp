#include <Arduino.h>
#include <FastLED.h>
#include <DallasTemperature.h>
//#include <MHZ19.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads; /* Use thi for the 12-bit version */

const int RX_PIN = 10;                      // Rx pin which the MHZ19 Tx pin is attached to
const int TX_PIN = 11;                      // Tx pin which the MHZ19 Rx pin is attached to
//MHZ19 *myMHZ19 = new MHZ19(RX_PIN, TX_PIN); // Constructor for library

unsigned long getDataTimer = 0;

#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float humi;
float tempC;

//Temperature Probes
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorsTemp(&oneWire);

//OLED define
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(4);

// How many leds in your strip?
#define NUM_LEDS 1
#define DATA_PIN 7
// Define the array of leds
CRGB leds[NUM_LEDS];

void led(int R, int G, int B)
{
  // Turn the LED on, then pause
  leds[0] = CRGB(R, G, B);
  FastLED.show();
  delay(200);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}

void setup()
{
  Serial.begin(9600);

  //For DHT11
  dht.begin();

  //FOR O2 sensor
  ads.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();

  //FOR CO2 Sensor
 // myMHZ19->begin(RX_PIN, TX_PIN);
 // myMHZ19->setAutoCalibration(false);
 // MHZ19 myMHZ19;                           // Constructor for library
 // SoftwareSerial mySerial(RX_PIN, TX_PIN); // (Uno example) create device to MH-Z19 serial

  //for temperature probes

  sensorsTemp.begin();

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS); // GRB ordering is typical
  //For OLED I2C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    //  for(;;);
  }

  display.display(); //Display logo
  delay(1000);
  display.clearDisplay();
}

void loop()
{
  delay(1000);
  humi = dht.readHumidity();
  tempC = dht.readTemperature();

  int16_t adc0;
  float O2perc;
  adc0 = ads.readADC_SingleEnded(0);
  O2perc = adc0 * 0.0078125 * 22.067;

  sensorsTemp.requestTemperatures();

//  measurement_t m = myMHZ19->getMeasurement();

  Serial.print("Temperature is: ");
  Serial.println(sensorsTemp.getTempCByIndex(0));
  Serial.print("Temperature is: ");
  Serial.println(sensorsTemp.getTempCByIndex(1));

  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print("Oxygen %");
  display.setCursor(0, 15);
  display.print(O2perc);

  display.setCursor(0, 35);
  display.print("CO2 ppm");
  display.setCursor(0, 50);
//  display.print(m.co2_ppm);

  Serial.print("CO2 is: ");
 // Serial.println(m.co2_ppm);
 // if (m.co2_ppm >= 600 && m.co2_ppm <= 1500)
  {

//    led(100, 255, 0);
  }
 // if (m.co2_ppm >= 2500)
  {
 //   led(0, 255, 0);
  }
//  if (m.co2_ppm < 600)
  {
 //   led(255, 0, 0);
  }
}
