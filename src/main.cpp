#include <Arduino.h>
#include <FastLED.h>
#include <DallasTemperature.h>
#include "MHZ19.h"
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>                            
#include <SoftwareSerial.h>    

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//General Cariable setup
double  calibrationv; //used to store calibrated value

 const int8_t RX_PIN = 0;                      // Rx pin which the MHZ19 Tx pin is attached to
 const int8_t TX_PIN = 1;                      // Tx pin which the MHZ19 Rx pin is attached to

//CO2 Sensor
MHZ19 myMHZ19;
SoftwareSerial co2Serial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

#define DHTPIN 3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//Temperature Probes
#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorsTemp(&oneWire);

U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // same as the NONAME variant, but may solve the "every 2nd line skipped" problem

// How many leds in your strip?
#define NUM_LEDS 2
#define DATA_PIN 6
// Define the array of leds
CRGB leds[NUM_LEDS];

void led(int l, int R, int G, int B)
{
  // Turn the LED on, then pause
  leds[l] = CRGB(R, G, B);
  FastLED.show();
  delay(200);
  leds[l] = CRGB(0, 0, 0);
  FastLED.show();
}


//Take 20 readings and avaraging it to exclude minor diviations of the reading
int calibrate(){
  
  int32_t adc0=0;
  int32_t result;
  for(int i=0; i<=49; i++)
       {
         adc0=adc0+ads.readADC_SingleEnded(0);
      }
    result=adc0/50;
    return result;
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
    co2Serial.begin(9600);
    myMHZ19.begin(co2Serial);
    myMHZ19.autoCalibration();
    

  //for temperature probes
  sensorsTemp.begin();

  //for LED
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS); // GRB ordering is typical

  u8g2.begin();  

  
  //O2 sensor calibration
  calibrationv=calibrate();


}
int progress = 0;
void loop()
{


    int32_t adc0=0;
    double o2Perc;//After calculations holds the current O2 percentage
    double currentmv; //the current mv put out by the oxygen sensor;
    double calibratev;

    for(int i=0; i<=19; i++)
       {
         
         adc0=adc0+ads.readADC_SingleEnded(0);
       }
            
      currentmv = adc0/20;
      calibratev=calibrationv;
    
     Serial.print("O2 sensor calibration ");                  
        Serial.print(adc0); Serial.print("  ");   
         Serial.print(calibratev); Serial.print("  ");  
           Serial.println(currentmv);  
     Serial.println("--------------------------"); 

      o2Perc=(currentmv/calibratev)*20.2;
    
      if(o2Perc>99.99){
        o2Perc=99.99;
      }
    
      if(o2Perc>70.00){
        led(0,255,0,0);
      }
      if(o2Perc<=70.00&&o2Perc>=20){
        led(0,255,255,0);
      }
      if(o2Perc<20.00){
        led(0,0,255,0);
      }

  int cO2ppm;
  cO2ppm = myMHZ19.getCO2();

 if(myMHZ19.errorCode == RESULT_OK)              // RESULT_OK is an alis for 1. Either can be used to confirm the response was OK.
        {
            Serial.print("CO2 Value successfully Recieved: ");
            Serial.println(cO2ppm);
            Serial.print("Response Code: ");
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
        }

        else 
        {
            Serial.println("Failed to recieve CO2 value - Error");
            Serial.print("Response Code: ");
            Serial.println(myMHZ19.errorCode);          // Get the Error Code value
        }  

        





    
      if(cO2ppm<1000){
        led(1,255,0,0);
      }
      if(cO2ppm<=2000.00&&cO2ppm>=1000){
        led(1,255,255,0);
      }
      if(cO2ppm>2000){
        led(1,0,255,0);
      }


  static char outco2[15];
  static char outo2[15];
  static char outhumi[15];
  static char outheati[15];
  static char outtemp[15];
 
  
  float humi = dht.readHumidity();
  float tempC = dht.readTemperature();
  float heatIndex = dht.computeHeatIndex(tempC, humi, false);

  sensorsTemp.requestTemperatures();

  dtostrf(cO2ppm,5,0,outco2);
  dtostrf(o2Perc,5,2,outo2);
  dtostrf(humi,5,2,outhumi);
  dtostrf(tempC,5,2,outtemp);
  dtostrf(heatIndex,5,2,outheati);

     Serial.print("Temperature (C) from DHT11: ");                  
        Serial.print(tempC); Serial.print("  ");   Serial.println(outtemp);  
     Serial.println("--------------------------"); 

     Serial.print("CO2 ppm ");                  
        Serial.print(cO2ppm);  Serial.print("  ");  Serial.println(outco2);  
     Serial.println("--------------------------");    

     Serial.print("O2 % ");                  
        Serial.print(o2Perc);  Serial.print("  ");  Serial.println(outo2);  
     Serial.println("--------------------------");    

     Serial.print("Humidity from DHT 11 ");                  
        Serial.print(humi);  Serial.print("  ");  Serial.println(outhumi);  
     Serial.println("--------------------------");    

     Serial.print("Heat Index from DHT 11 ");                  
        Serial.print(heatIndex);  Serial.print("  "); Serial.println(outheati);  
     Serial.println("--------------------------");    
 
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
  u8g2.drawStr(0,10,outco2);	// write something to the internal memory
  u8g2.drawStr(50,20,outo2);	// write something to the internal memory
  u8g2.drawStr(0,30,outhumi);	// write something to the internal memory
  u8g2.drawStr(50,40,outtemp);	// write something to the internal memory
  u8g2.drawStr(0,50,outheati);	// write something to the internal memory


  u8g2.sendBuffer();					// transfer internal memory to the display
 
  delay(1000); 

}
