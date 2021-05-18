#include <Arduino.h>
#include <FastLED.h>
#include <DallasTemperature.h>
#include "MHZ19.h"
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>


Adafruit_ADS1015 ads; /* Use thi for the 12-bit version */

const int8_t RX_PIN = 10;                      // Rx pin which the MHZ19 Tx pin is attached to
const int8_t TX_PIN = 11;                      // Tx pin which the MHZ19 Rx pin is attached to

MHZ19 myMHZ19;
SoftwareSerial cO2serial(RX_PIN,TX_PIN);

#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//Temperature Probes
#define ONE_WIRE_BUS 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorsTemp(&oneWire);

//OLED define
#define SCREEN_WIDTH 100 // OLED display width, in pixels
#define SCREEN_HEIGHT 50 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Adafruit_SSD1306 display(4);

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

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
    cO2serial.begin(9600);
    myMHZ19.begin(cO2serial);
    myMHZ19.autoCalibration();
    

  //for temperature probes
  sensorsTemp.begin();

  //for LED
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS); // GRB ordering is typical
/* 
  //For OLED I2C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    //  for(;;);
  }
 */
  u8g2.begin();  
 // display.display(); //Display logo
//  delay(1000);
 // display.clearDisplay();
}

void loop()
{
  delay(1000);


  float humi;
  float tempC;
  
  humi = dht.readHumidity();
  tempC = dht.readTemperature();

  int16_t adc0;
  float O2perc;
  adc0 = ads.readADC_SingleEnded(0);
  O2perc = adc0 * 0.0078125 * 22.067;

  sensorsTemp.requestTemperatures();

  int cO2ppm;
  cO2ppm = myMHZ19.getCO2();

   Serial.print("CO2 ppm is: ");
  Serial.println(cO2ppm);


  Serial.print(F("Temperature is: "));
  Serial.println(sensorsTemp.getTempCByIndex(0));
  Serial.print(F("Temperature is: "));
  Serial.println(sensorsTemp.getTempCByIndex(1));

   
  u8g2.firstPage();
  do {
    //O2 display
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,16,"O2 %");
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.setCursor(0,35);
    u8g2.print(O2perc);

    //CO2 display
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(50,12,"CO2");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(50,20,"ppm");
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.setCursor(50,35);
    u8g2.print(cO2ppm);
    
    //Humidity display
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(100,12,"Rel");
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(100,20,"Hum");
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.setCursor(100,35);
    u8g2.print(humi);

     //CO2 warning display
    u8g2.setFont(u8g2_font_ncenB12_tr);

 if (cO2ppm >= 600 && cO2ppm <= 2500)
  {
    led(100, 255, 0);
    u8g2.drawStr(0,60,"CO2 Warning");
  }
  if (cO2ppm > 2500)
  {
    led(0, 255, 0);
     u8g2.drawStr(0,60,"CO2 HIGH");
  }
  if (cO2ppm < 600)
  {
    led(255, 0, 0);
     u8g2.drawStr(0,60,"All Normal");
  }



  } while ( u8g2.nextPage() );
  //delay(1000);



}
