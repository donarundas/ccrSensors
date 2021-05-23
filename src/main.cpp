#include <Arduino.h>
#include <FastLED.h>
#include <DallasTemperature.h>
#include "MHZ19.h"
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>                            
#include <SoftwareSerial.h>    

//For O2 sensor ADC converter
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//General Calibration for O2 setup
double  calibrationv; //used to store calibrated value


//CO2 Sensor
const int8_t RX_PIN = 0;                      // Rx pin which the MHZ19 Tx pin is attached to
const int8_t TX_PIN = 1;                      // Tx pin which the MHZ19 Rx pin is attached to
MHZ19 myMHZ19;
SoftwareSerial co2Serial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

// DHT Humidity and temperature sensor
#define DHTPIN 3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//Temperature Probes
#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorsTemp(&oneWire);

//Display Selector
//U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// LED HUD setup

#define NUM_LEDS 2
#define DATA_PIN 6
CRGB leds[NUM_LEDS];
void led(int l, int G, int R, int B)
{
  // Turn the LED on, then pause
  leds[l] = CRGB(G, R, B);
  FastLED.show();
  delay(200);
  leds[l] = CRGB(0, 0, 0);
  FastLED.show();
}


//O2 sensor calibration function with 30 inputs
int calibrate(){
  
  int16_t adc0=0;
  int16_t result;
  Serial.print("O2 sensor calibration ");  
  for(int i=0; i<=9; i++)
       {
         adc0=adc0+ads.readADC_SingleEnded(0);
        
                     
        Serial.print(adc0); Serial.print(" - ");   

      }
      
     Serial.println("--------------------------"); 
    result=adc0/10;
    return result;
}


//*************** DISPLAY FUNCTIONS *****************//

void u8g2_prepare(void) {
 
  u8g2.setFontRefHeightExtendedText();//Ascent will be the largest ascent of "A", "1" or "(" of the current font. Descent will be the descent of "g" or "(" of the current font.
  u8g2.setDrawColor(1);        //Defines the bit value (color index) for all drawing functions. All drawing functions will change the display memory value to this bit value. The default value is 1.
  u8g2.setFontPosTop();    /*When you use drawStr to display strings, the default criteria is to display the lower-left coordinates of the characters.
                        XXXX  */
  u8g2.setFontDirection(0);    //Set the screen orientation: 0 -- for normal display
    
}

/*
 * Draw the grid
*/
void grid() {

  u8g2.drawFrame(0,0,u8g2.getDisplayWidth(),u8g2.getDisplayHeight() );//Start drawing an empty box of width w and height h at a coordinate of (0,0)
  u8g2.drawLine(50,0,50,64);
  u8g2.drawLine(50,21,128,21);
  u8g2.drawLine(50,42,128,42);
  u8g2.drawLine(0,32,50,32);
}

/*
 * Draw the Header Texts
*/

void headerGrids(){

  u8g2.drawStr( 1, 0, "O2 %");
  u8g2.drawStr( 55, 0, "CO2 ppm");
  u8g2.drawStr( 55, 43, "Rel. Humi");
  u8g2.drawStr( 55, 22, "Temperature");
 // u8g2.drawStr( 47, 34, "Heat Index");
  u8g2.drawStr( 1, 33, "Check:");
}






void setup()
{

  delay (2000);
  
  //Open Serial Port - Remove for production
  Serial.begin(9600);
  
  //For DHT11
  dht.begin();

  //FOR O2 sensor
  ads.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();
 
  //FOR CO2 Sensor
  co2Serial.begin(9600);
  myMHZ19.begin(co2Serial);
  myMHZ19.autoCalibration(); //Autocalibration of the MHZ19 JST plus Sensor
    

  //for temperature probes
  sensorsTemp.begin();

  //for LED
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS); // GRB ordering is typical

 // u8g2.begin();  



  u8g2_prepare();

 
  //O2 sensor calibration
  calibrationv=calibrate();

}
void loop()
{

delay(2000);
    int32_t adc0=0;
    double o2Perc;//After calculations holds the current O2 percentage
    double currentmv; //the current mv put out by the oxygen sensor;
    double calibratev;

 Serial.print("O2 sensor readings post");  

    for(int i=0; i<=9; i++)
       {
         
         adc0=adc0+ads.readADC_SingleEnded(0);
          Serial.print(adc0); Serial.print("  ");   
       }
            
      currentmv = adc0/10;
      calibratev=calibrationv;
    
                    
      
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
   u8g2.setFont(u8g2_font_courB08_tf);    //Set the font to "u8g2_font_courB12_tf"
    grid();
    headerGrids();
  
  u8g2.drawStr(55,11,outco2);	// write something to the internal memory
  u8g2.drawStr(55,52,outhumi);	// write something to the internal memory
  u8g2.drawStr(55,32,outtemp);	// write something to the internal memory
 // u8g2.drawStr(0,50,outheati);	// write something to the internal memory
  
  u8g2.setFont(u8g2_font_courB12_tf); 
  u8g2.drawStr(1,10,outo2);	// write something to the internal memory
 // u8g2.drawStr(1,45,"CO2");	// write something to the internal memory

  u8g2.sendBuffer();					// transfer internal memory to the display
 
  delay(1000); 

}
