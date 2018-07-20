#include <ESP8266WiFi.h>
#include <FirebaseArduino.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <user_interface.h>

#include "DHTesp.h"

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif

DHTesp dht;

extern "C" {
#include "user_interface.h"
}


os_timer_t amp;
bool tickOccured;

volatile int watchdogCount = 0;

/*Calibração do solo*/
//solo seco [800 1023]
//solo semi-umido [630 750]
//solo umido ]451 600]

#define FIREBASE_HOST "projetotcc2-39dd8.firebaseio.com"
#define FIREBASE_AUTH "Lhdsekyvbfw7VaxDc5oPOezIhmAlRKgHnADJhRk4"
#define WIFI_SSID "sweethome"
#define WIFI_PASSWORD "vitorkamila"
#define DHTPIN 14
#define DHTTYPE DHT11

int R = 12;
int G = 13; 
int B = 15;
int porta_analogica = 0;
int valor_sensor = 0;

WiFiUDP ntpUDP;
int16_t utc = -3; //UTC -3:00 Brazil

NTPClient timeClient(ntpUDP, "a.st1.ntp.br", utc*3600, 60000);
uint32_t currentMillis = 0;
uint32_t previousMillis = 0;
 
String status_solo;
float umidade_do_ar;
String hora_;
String timetemps_;

//DHT dht(DHTPIN, DHTTYPE);
  

// start of timerCallback
void timerCallback(void *pArg) {

      tickOccured = true;

} 

void user_init(void){
      
      os_timer_setfn(&amp,timerCallback, NULL);
      os_timer_arm(&amp, 1800000 , true);
}



void forceUpdate(void) {
  timeClient.forceUpdate();
}


String hora(void)
{
   currentMillis = millis();
   if (currentMillis - previousMillis > 1000) 
   {
     previousMillis = currentMillis;        
   }
   return timeClient.getFormattedTime();
}

unsigned long timestemp(void){

   currentMillis = millis();
   if (currentMillis - previousMillis > 1000) 
   {
     previousMillis = currentMillis;        
   }
   return timeClient.getEpochTime();
}  

void setup() {
  
  Serial.begin(115200); 
  pinMode(porta_analogica,INPUT);
  pinMode(R,OUTPUT);
  pinMode(G,OUTPUT);
  pinMode(B,OUTPUT);  
  
  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  dht.setup(14, DHTesp::DHT11); // Connect DHT sensor to GPIO 17
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  timeClient.begin();
  timeClient.update();  
  tickOccured = false;
  user_init();

}


void loop()
{ 
    if(tickOccured == true)
    {       
       valor_sensor = analogRead(porta_analogica);
 //    Serial.println(valor_sensor);
      if(valor_sensor> 800)
      {
        status_solo = "solo seco";
 //       Serial.println("solo seco");
        digitalWrite(R, HIGH);
        digitalWrite(G, LOW);
        digitalWrite(B, LOW);
      }else if(valor_sensor > 630 && valor_sensor < 750){
        status_solo = "solo semi-umido";
//        Serial.println("solo semi-umido");
        digitalWrite(B, HIGH);
        digitalWrite(R, LOW);
        digitalWrite(G, LOW);
      }else if(valor_sensor < 600){
        status_solo = "umido";
   //     Serial.println("umido");
        digitalWrite(G, HIGH);
        digitalWrite(R, LOW);
        digitalWrite(B, LOW);
      }

      int h = dht.getHumidity();
      int t = dht.getTemperature();
      Serial.println(h);
      Serial.println(t);

      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");   
        ESP.restart();     
        return;
      }         
    
      // append a new value to /logs 
      // handle error
       if (Firebase.failed())
       {
          Serial.print("pushing /logs failed:");
          Serial.println(Firebase.error());  
          ESP.restart();
          return;
       }

         hora_ = hora();
         timetemps_ = timestemp();

         StaticJsonBuffer<200> jsonBuffer;
         JsonObject& root = jsonBuffer.createObject();
         root["ground"] = status_solo;
         root["humidity"] = h;// falta acertar esse valor na aquisição
         root["temperature"] = t;              
         root["hour"] = hora_;
         root["timestemp"] = timetemps_;
         String name = Firebase.push("/status", root);
         delay(1000); 
         tickOccured = false; 
    }
    yield();  // or delay(0);
}
