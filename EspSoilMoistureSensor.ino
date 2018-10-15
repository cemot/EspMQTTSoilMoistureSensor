
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>


#include <DHTesp.h>
#include "config.h"
#include <ESP8266WiFi.h>
#define SLEEP_TIME 1200 * 1000 * 1000

WiFiClient client;
char ssid[] = WIFIHOTSPOT;
char pw[] = WIFIKEY;

const int pin_clk = 5;
const int pin_soil = A0; 
const int pin_led = 12;
const int pin_read = 2;

Adafruit_MQTT_Client mqtt(&client,MQTTSERVER,MQTTPORT);
Adafruit_MQTT_Publish dataOut = Adafruit_MQTT_Publish(&mqtt,MQTTTOPIC);

/** Initialize DHT sensor */
DHTesp dht;
/** Task handle for the light value read task */



void setup_wifi() {
  WiFi.begin(ssid,pw);
}

void setup() {
  Serial.begin(115200);
  dht.setup(pin_read,DHTesp::DHT11);
  
  pinMode(pin_clk, OUTPUT);
  pinMode(pin_soil, INPUT);
  pinMode(pin_led, OUTPUT);
  digitalWrite(pin_led, LOW);
  pinMode(pin_read, INPUT);
  
  analogWriteFreq(40000);
  analogWrite(pin_clk, 400);
  delay(500);
  digitalWrite(pin_led, LOW);
  setup_wifi();



  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
}

float readSoilVal(int n) {
  float valtemp = 0;
  float ValBuf[n];
  int i = 0;
  int j = 0;

  for(i=0;i<n;i++){
    ValBuf[i] = analogRead(pin_soil);  
  }

  for(i=0;i<n-1;i++){
    for(j=i+1;j<n;j++){
      if(ValBuf[i]>ValBuf[j]){
        valtemp = ValBuf[i];
        ValBuf[i] = ValBuf[j];
        ValBuf[j] = valtemp;
      }
    }
  }

  valtemp = 0;
  for(i=1;i<n-1;i++){
    valtemp+=ValBuf[i];
  }
  valtemp/=(n-2);
  return valtemp;
}


void loop() {
  MQTT_connect();
  //mqtt.handle();
  delay(5000);
  //mqtt.handle();
  float soil = readSoilVal(8);
  TempAndHumidity lastValues = dht.getTempAndHumidity();
 /* Serial.println("Temperature: " + String(lastValues.temperature,0));
  Serial.println("Humidity: " + String(lastValues.humidity,0));
  Serial.println(soil);*/
  //mqtt.publish("espSoil",String(soil)+","+String(lastValues.temperature,0)+","+String(lastValues.temperature,0));
  //mqtt.handle();
  Serial.println("trying to send poop");
   char buff[20];
   snprintf (buff, sizeof(buff), "%.2f,%.2f,%.2f",soil,lastValues.temperature,lastValues.humidity);
  dataOut.publish(buff);
  delay(1000);
  mqtt.disconnect();
  ESP.deepSleep(SLEEP_TIME, WAKE_RF_DEFAULT);
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}
