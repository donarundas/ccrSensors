#include <Arduino.h>
#include <FastLED.h>
#include <DallasTemperature.h>
#include "MHZ19.h"
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>                            
#include <SoftwareSerial.h>    
#include <SPI.h>
#include <SD.h>

//For O2 sensor ADC converter
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//General Calibration for O2 setup
#define o2sensors 1
double  calibrationv[o2sensors]={0}; //used to store calibrated value
int8_t calCheck[o2sensors]={0};
double calibratev[o2sensors]={0};


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


File myDataFile;
File myDataFile2;


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
int calibrate(int8_t sensor){
  
  int32_t adc=0;
  int32_t result;

  Serial.println("------------------------------- ");
  Serial.print("Calibratioin in Progress for Sensor ");Serial.println(sensor);
  Serial.println("------------------------------- ");
     for(int i=0; i<=29; i++)
       {
         delay(100);
         adc=adc+ads.readADC_SingleEnded(sensor);
         Serial.print(i); Serial.print(" - ");Serial.print(adc);
      }

 Serial.println("------------------------------- ");
  Serial.print("Calibratioin Completed for Sensor ");Serial.println(sensor);
  Serial.println("------------------------------- ");

   Serial.print("ADC Value is: ");Serial.println(adc);


    result=adc/30;
    calCheck[sensor]=1;

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
void headerGrids1(){

  u8g2.drawStr( 1, 0, "O2 - 1 %");
  u8g2.drawStr( 55, 0, "CO2 ppm");
  u8g2.drawStr( 55, 43, "Rel. Humi");
  u8g2.drawStr( 55, 22, "Temperature");
 // u8g2.drawStr( 47, 34, "Heat Index");
  u8g2.drawStr( 1, 33, "O2 - 2 %");
}






void setup()
{

  delay (2000);
  
  //Open Serial Port - Remove for production
  Serial.begin(9600);


  Serial.print("Initializing SD card...");

  if (!SD.begin(2)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

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

  u8g2.begin();  



  u8g2_prepare();

 
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myDataFile = SD.open("test.txt", FILE_WRITE);
// if the file opened okay, write to it:
  if (myDataFile) {
    Serial.print("Writing to Data Logger.txt...");
    myDataFile.println("Initializing Control Systems");
    myDataFile.println("Starting Calibration and Data Recording");
    myDataFile.println("v1.0");
    myDataFile.println("Copyright Reb-Tek");
    myDataFile.println("----------------------------");
    myDataFile.println("");
    myDataFile.println("");
    myDataFile.println("");
    myDataFile.close();

    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening Data Logger.txt");
  }

  //O2 sensor calibration
//  calibrationv=calibrate();

}
void loop()
{
// make a string for assembling the data to log:
  String dataString = "";

    int32_t adc[o2sensors]={0};
    double o2Perc[o2sensors]={0};//After calculations holds the current O2 percentage
    double currentmv[o2sensors]={0}; //the current mv put out by the oxygen sensor;


    for(int i=0; i<o2sensors; i++)
       {
         for(int j=0; j<=19;j++){

              adc[i]=adc[i]+ads.readADC_SingleEnded(i);
              
              Serial.print(" adc");Serial.print(i);Serial.print(" : ");Serial.print(adc[i]);
         }
        Serial.println("------------------------------- ");
        currentmv[i] = adc[i]/20;
        calibratev[i]=calibrationv[i];
        o2Perc[i]=(currentmv[i]/calibratev[i])*20.9;

        Serial.println("O2 sensor Calculation:");
        Serial.print("Sensor Reading - ");Serial.print(currentmv[i]);
        Serial.print("- Calibrated Value - ");Serial.print(calibratev[i]);Serial.print("- ");
        Serial.print("- O2 %age - ");Serial.println(o2Perc[i]);

        
        if(o2Perc[i]>99.99){
        o2Perc[i]=99.99;

        if(o2Perc[i]>70.00){
        led(i,255,0,0);
        }
        
        if(o2Perc[i]<=70.00&&o2Perc[i]>=20){
        led(i,255,255,0);
        }
        if(o2Perc[i]<20.00){
        led(i,0,255,0);
        }


        }
          
        }

     
    
     
  int cO2ppm;
  cO2ppm = myMHZ19.getCO2();
 
      if(cO2ppm<1000){
        led(2,255,0,0);
      }
      if(cO2ppm<=2000&&cO2ppm>=1000){
        led(2,255,255,0);
      }
      if(cO2ppm>2000){
        led(2,0,255,0);
      }


  static char outco2[15];
  static char outo2[o2sensors][15];
  static char outhumi[15];
  static char outheati[15];
  static char outtemp[15];
 
  
  float humi = dht.readHumidity();
  float tempC = dht.readTemperature();
  float heatIndex = dht.computeHeatIndex(tempC, humi, false);

  sensorsTemp.requestTemperatures();

  dtostrf(cO2ppm,5,0,outco2);
  
  dataString += outco2;
  dataString += ",";

  dtostrf(humi,5,2,outhumi);
  dataString += outhumi;
  dataString += ",";

  dtostrf(tempC,5,2,outtemp);
  dataString += outtemp;
  dataString += ",";

  dtostrf(heatIndex,5,2,outheati);
  dataString += outheati;
  dataString += ",";

  dtostrf(o2Perc[0],5,2,outo2[0]);
  dataString += outo2[0];
  dataString += ",";
  

   u8g2.clearBuffer();					// clear the internal memory
   u8g2.setFont(u8g2_font_courB08_tf);    //Set the font to "u8g2_font_courB12_tf"
    grid();
    

  
  u8g2.drawStr(55,11,outco2);	// write something to the internal memory
  u8g2.drawStr(55,52,outhumi);	// write something to the internal memory
  u8g2.drawStr(55,32,outtemp);	// write something to the internal memory

  
  u8g2.setFont(u8g2_font_courB12_tf); 
  u8g2.drawStr(1,10,outo2[0]);	// write something to the internal memory

  if(o2sensors>1){
    for(int i=1;i<o2sensors;i++){
      dtostrf(o2Perc[i],5,2,outo2[i]);
      
  dataString += outo2[i];
  dataString += ",";
      
      u8g2.drawStr(1,42,outo2[1]);	// write something to the internal memory

       u8g2.setFont(u8g2_font_courB08_tf);
      headerGrids1();
      }
  }
  else{
     u8g2.setFont(u8g2_font_courB08_tf);
      headerGrids();
  }
  

  
  myDataFile2 = SD.open("log.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myDataFile2) {
    Serial.print("Writing to test.txt...");
    myDataFile2.print(dataString); 
    myDataFile2.println("..");
    // Flush the file:
    myDataFile2.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }





  u8g2.sendBuffer();					// transfer internal memory to the display
 
 for(int i=0; i<o2sensors; i++){
  if(calCheck[i]!=1){
    
  //O2 sensor calibration
  calibrationv[i]=calibrate(i);

  }
 }



  delay(500);  

}
