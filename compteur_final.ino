#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ssd1306syp.h>
#define SDA_PIN 2
#define SCL_PIN 3
Adafruit_ssd1306syp display(SDA_PIN,SCL_PIN);

#define DS18B20 0x28

OneWire  ds(10);

// Création du gps via la librairie TinyGPSPlus
TinyGPSPlus gps;

// Connexion série pour le GPS
SoftwareSerial gps_serial(15, 14);		// Port choisi sur Arduino UNO, le TX ne nous interesse pas

int maxspeed=0;
int verif=0;
double heureDepart=0.0;
double coordDepartLat=0.0;
double coordDepartLng=0.0;
char avg_final[20];

/************************ CAPTEUR TEMPERATURE *************************/
boolean getTemperature(float *temp){
  byte data[9], addr[8];  // data : Données lues depuis le scratchpad     ||   addr : adresse du module 1-Wire détecté

  if (!ds.search(addr)) { // Recherche un module 1-Wire
    ds.reset_search();    // Réinitialise la recherche de module
    return false;         // Retourne une erreur
  }

  if (OneWire::crc8(addr, 7) != addr[7]) // Vérifie que l'adresse a été correctement reçue
    return false;                        // Si le message est corrompu on retourne une erreur

  if (addr[0] != DS18B20) // Vérifie qu'il s'agit bien d'un DS18B20
      return false;         // Si ce n'est pas le cas on retourne une erreur

  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20

  ds.write(0x44, 1);      // On lance une prise de mesure de température

  ds.reset();             // On reset le bus 1-Wire
  ds.select(addr);        // On sélectionne le DS18B20
  ds.write(0xBE);         // On envoie une demande de lecture du scratchpad

  for (byte i = 0; i < 9; i++) // On lit le scratchpad
    data[i] = ds.read();       // Et on stock les octets reçus

  // Calcul de la température en degré Celsius
  *temp = ((data[1] << 8) | data[0]) * 0.0625; 

  // Pas d'erreur
  return true;
}

void displayInfo(int *maxspeed)
{
  float temp;
  int vitesse=gps.speed.kmph();
  //Serial.print(F("Localisation: ")); 
  if (gps.location.isValid())
  {
    if(getTemperature(&temp)){ // Si le capteur de température est bien connecté (permet l'affichage de toutes les infos sur l'écran sans clignotements)
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(","));
    Serial.println(gps.speed.kmph(), 6);
    Serial.print(F("Vitesse: ")); 
    Serial.print(gps.speed.kmph());
    
    display.update();
    display.clear();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(30,26);
  
/********* AFFICHAGE VITESSE *********/
    display.print(vitesse);
    display.print("km/h");
/********* FIN AFFICHAGE VITESSE *********/

/********* AFFICHAGE TEMP *********/
    display.setTextSize(1);
    display.setCursor(90,0);
    display.print(temp,1); //1 to specify one decimal digit only
    display.print((char)247); //Degrees Celcius symbol
    display.print('C');
/********* FIN AFFICHAGE TEMP *********/

/********* AFFICHAGE MAX *********/
  if(*maxspeed<gps.speed.kmph()) *maxspeed=gps.speed.kmph();
  display.setCursor(0,55);
  display.print(*maxspeed);
    display.print("km/h");
/********* FIN AFFICHAGE MAX *********/

/********* AFFICHAGE SAT *********/
  display.setCursor(40,6);
  display.print(gps.satellites.value());
  display.print(" Sats");
/********* FIN AFFICHAGE SAT *********/

  const double ceri_LAT = 43.730178;
  const double ceri_LONG = 4.224686;
  double distanceKm = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),ceri_LAT,ceri_LONG) / 1000.0;
  double courseTo = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),ceri_LAT,ceri_LONG);
  
/********* AFFICHAGE DISTANCE *********/    
  display.setCursor(80,55);
  display.print(distanceKm);  
  display.print("km");
/********* FIN AFFICHAGE DISTANCE *********/


/********* AFFICHAGE AVG *********/
/*
    double heure=(gps.time.hour()+1)+(gps.time.minute()/60.0);
    double coordLat=gps.location.lat();
    double coordLng=gps.location.lng();
    double temps=heure-heureDepart;
    double distance=TinyGPSPlus::distanceBetween(coordDepartLat,coordDepartLng,coordLat,coordLng) / 1000.0;
    double avg=distance/temps;
    
    avg=floor(avg*10)/10;        //Permet d'arrondir a 0.0
    dtostrf(avg, 5, 1, avg_final);  // au lieu de 0.00
    
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(-10,45);
    display.print(avg_final); 
    display.print("km/h");
    */
/********* FIN AFFICHAGE AVG *********/

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
    /********* AFFICHAGE HEURE *********/ 
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    if (gps.time.hour() < 9) display.print("0");
    display.print(gps.time.hour()+1);
    display.print(":");
    if (gps.time.minute() < 9) display.print("0");
    display.print(gps.time.minute());
/********* FIN AFFICHAGE HEURE *********/
  
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
    
/************************ FIN CAPTEUR TEMPERATURE *************************/

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

void setup()
{
  Serial.begin(115200);
  gps_serial.begin(9600);
  display.initialize();

}

void loop()
{
  while (gps_serial.available())
      gps.encode(gps_serial.read());
      
/************************ INITIALISATION DES VARIABLES POUR LA MOYENNE (coordoonnées et heure du départ) *************************/
    if(verif==0){
      heureDepart=(gps.time.hour()+1)+(gps.time.minute()/60.0);
      coordDepartLat=gps.location.lat();
      coordDepartLng=gps.location.lng();
      if(gps.location.isValid()){			// Permet d'incrémenter la variable verif afin de faire la boucle précedente une seule fois
         verif++; 
      } 
    }
/************************ FIN INITIALISATION DES VARIABLES POUR LA MOYENNE (coordoonnées et heure du départ) *************************/
  displayInfo(&maxspeed);

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

if(gps.satellites.isValid()==false) smartDelay(500);
}

