#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <Adafruit_ssd1306syp.h>

//Define OLED PINS
#define SDA_PIN 2
#define SCL_PIN 3

//Init OLED Display
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);

//This is required for temperature sensor
#define DS18B20 0x28

//This is the temperature sensor init
OneWire ds(10);

// Initialize TinyGPSPlus library
TinyGPSPlus gps;

// The GPS sensor is plugged to PINs 15 and 14
SoftwareSerial gps_serial(15, 14);		// RX connected to 15, TX connected to 14

int maxspeed=0;
bool done=false;
double startTime=0.0;
double coordStartLat=0.0;
double coordStartLng=0.0;
char avg_final[20];

/************************ TEMPERATURE SENSOR *************************/
boolean getTemperature(float *temp){
  byte data[9], addr[8];  // data : Data read from scratchpad     ||   addr : 1-Wire module address

  if (!ds.search(addr)) { // Look for the 1-Wire module
    ds.reset_search();    // Reset search module
    return false;         // In case of an error
  }

  if (OneWire::crc8(addr, 7) != addr[7]) // Check the address has been correctly received
    return false;                        // Return an error if not...

  if (addr[0] != DS18B20) // Check that we have a DS18B20 sensor
      return false;       // Return an error if not...

  ds.reset();             // 1-Wire bus reset
  ds.select(addr);        // Select the DS18B20 sensor

  ds.write(0x44, 1);      // Get temperature

  ds.reset();             // 1-Wire bus reset
  ds.select(addr);        // Select the DS18B20 sensor
  ds.write(0xBE);         // Send scratchpad read request

  for (byte i = 0; i < 9; i++) // Read scratchpad
    data[i] = ds.read();       // Store bytes received

  // Convert temperature to Celsius
  *temp = ((data[1] << 8) | data[0]) * 0.0625; 

  // No error!
  return true;
}
/************************ END TEMPERATURE SENSOR *************************/


/************************ MAIN DISPLAY MANAGEMENT *************************/
void displayInfo(int *maxspeed)
{
  float temp;
  float currentSpeed = 0.0;
  int numSatellites = 0;
  if(gps.speed.isValid()) currentSpeed=gps.speed.kmph();
 
  if (gps.location.isValid())
  {
    if(getTemperature(&temp)){ // If temperature correctly received (prevents screen flickering)
    Serial.print(F("location: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(F("location:"));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(", "));

    display.update();
    display.clear();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(30,26);
  
/********* DISPLAY SPEED *********/
    Serial.print(F("Speed:"));
    Serial.print(currentSpeed,2);

    display.print(floor(currentSpeed), 0);
    display.print(F("km/h"));
/********* END DISPLAY SPEED *********/

/********* DISPLAY TEMP *********/
    display.setTextSize(1);
    display.setCursor(90,0);
    display.print(temp,1);    //1 to specify one decimal digit only
    display.print((char)247); //Degrees Celcius symbol
    display.print('C');
/********* END DISPLAY TEMP *********/

/********* DISPLAY MAX *********/
  if(*maxspeed<currentSpeed) *maxspeed=currentSpeed;
  display.setCursor(0,55);
  display.print(*maxspeed);
  display.print(F("km/h"));
/********* END DISPLAY MAX *********/

/********* DISPLAY SAT *********/
  if(gps.satellites.isValid()) numSatellites = gps.satellites.value();
  display.setCursor(40,6);
  display.print(numSatellites);
  display.print(F(" Sats"));
/********* END DISPLAY SAT *********/

  const double ceri_LAT = 43.730178;
  const double ceri_LONG = 4.224686;
  double distanceKm = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),ceri_LAT,ceri_LONG) / 1000.0;
  double courseTo = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),ceri_LAT,ceri_LONG);
  
/********* DISPLAY DISTANCE *********/    
  display.setCursor(80,55);
  display.print(distanceKm);  
  display.print(F("km"));
/********* END DISPLAY DISTANCE *********/


/********* DISPLAY AVG *********/
/*
    double heure=(gps.time.hour()+1)+(gps.time.minute()/60.0);
    double coordLat=gps.location.lat();
    double coordLng=gps.location.lng();
    double temps=heure-startTime;
    double distance=TinyGPSPlus::distanceBetween(coordStartLat,coordStartLng,coordLat,coordLng) / 1000.0;
    double avg=distance/temps;
    
    avg=floor(avg*10)/10;        //Permet d'arrondir a 0.0
    dtostrf(avg, 5, 1, avg_final);  // au lieu de 0.00
    
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(-10,45);
    display.print(avg_final); 
    display.print("km/h");
    */
/********* END DISPLAY AVG *********/

    }
  }
  else
  {
    Serial.print(F("INVALID"));
    display.update();
    display.clear();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,25);
    display.print("Init GPS...");
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    /********* DISPLAY TIME *********/ 
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    if (gps.time.hour() < 9) display.print("0");
    display.print(gps.time.hour()+1);
    display.print(":");
    if (gps.time.minute() < 9) display.print("0");
    display.print(gps.time.minute());
/********* END DISPLAY TIME *********/
  
  }
  else
  {
    Serial.print(F("INVALID"));
    display.update();
    display.clear();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,25);
    display.print("Init GPS...");
  }
    Serial.println();
}
/********************** END MAIN DISPLAY MANAGEMENT *********************/

/************************** GPS SMART DELAY *****************************/
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gps_serial.available())
      gps.encode(gps_serial.read());
  } while (millis() - start < ms);
}
/************************** END GPS SMART DELAY *************************/

/******************************* MAIN SETUP *****************************/
void setup()
{
  //Init standard serial   
  Serial.begin(115200);
  //Init GPS serial
  gps_serial.begin(9600);
  //Init OLED display
  display.initialize();
}
/**************************** END MAIN SETUP ****************************/

void loop()
{
  smartDelay(300);
  /*
  while (gps_serial.available())
      gps.encode(gps_serial.read());*/
      
/***** INIT VARIABLES FOR AVERAGE SPEED CALCULATIONS *****/
    if(!done){    
      startTime=(gps.time.hour()+1)+(gps.time.minute()/60.0);
      coordStartLat=gps.location.lat();
      coordStartLng=gps.location.lng();
      if(gps.location.isValid()){			// Just to make sure we do this loop once
         done=true;
      } 
    }
/*** END INIT VARIABLES FOR AVERAGE SPEED CALCULATIONS ****/

  // Go and display!
  displayInfo(&maxspeed);

  //Fatal error? GPS not correctly detected...
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

  //Not yet able to detect satellites correctly? Let's have a breath...
  if(gps.satellites.isValid()==false) 
  {
    Serial.println("No a valid count of satellites...");
    smartDelay(1000);
  }
}
