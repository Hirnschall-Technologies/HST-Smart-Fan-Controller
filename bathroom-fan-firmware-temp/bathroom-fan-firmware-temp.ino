#include <DHT.h>
#include <ESP8266WiFi.h>
const char *ssid = "DLan";
const char *password = "PW2015dIr615!";
//---------------------------------------------------
#include <ESP8266WebServer.h>
ESP8266WebServer server(80);  //listen to port 80

#define DHT1PIN 14
#define DHT2PIN 4

#define FANPWMPIN 5

#define L1VCC 13
#define L2VCC 12
#define SENSORDELAY 500

#define DHTTYPE    DHT22

#define MAX_ACCEPTABLE_HUMIDITY_DELTA_ON 12
#define MAX_ACCEPTABLE_HUMIDITY_DELTA_OFF 7
#define MAX_ACCEPTABLE_HUMIDITY 97

#define TEMP_READ_INTERVALL_IN_MS 1000
#define MIN_LIGHT_ON_TIME_TOILET 180000 //3min
#define TOILET_FAN_ON_TIME 300000 //5min
#define MANUAL_ON_TIME 300000 //5min
#define MANUAL_OFF_TIME 1200000 //20min

DHT dht1(DHT1PIN, DHTTYPE);
DHT dht2(DHT2PIN, DHTTYPE);


int L1Threshold = 200;
int L2Threshold = 200;

float hum1=0;
float hum2=0;
float temp1=0;
float temp2=0;

int L1 = 0;
int L2 = 0;

bool L1IsOn=false;
bool L2IsOn=false;

short currentFanStatus =0;

unsigned long previousMillis =0;
unsigned long fanOnAt =0;
unsigned long L2OnAt = 0;
unsigned long manualAt = 0;


float dist(float x, float y){
  return x>y?x-y:y-x;
}

int readLightSensor(int pin,unsigned int sensordelay){
  digitalWrite(pin,HIGH);
  delay(sensordelay);
  int val = analogRead(A0);
  digitalWrite(pin,LOW);
  return val;
}

void getStatus(){
  if(server.hasArg("status")){
    server.send ( 200, "text/html",  String(currentFanStatus));
  }else if(server.hasArg("dht1")){
    server.send ( 200, "application/json",  "{\"temperature\":" + String(temp1) + ",\"humidity\":" + String(hum1) + "}" );
  }else if(server.hasArg("dht2")){
    server.send ( 200, "application/json",  "{\"temperature\":" + String(temp2) + ",\"humidity\":" + String(hum2) + "}" );
  }else if(server.hasArg("light1")){
    server.send ( 200, "application/json",  "{\"lightlevel\":" + String(L1) + "}" );
  }else if(server.hasArg("light2")){
    server.send ( 200, "application/json",  "{\"lightlevel\":" + String(L2) + "}" );
  }else if(server.hasArg("on")){
    manualAt = millis();
    digitalWrite(FANPWMPIN, HIGH);
    currentFanStatus =1;
    server.send ( 200, "text/html",  String(currentFanStatus));
    }else if(server.hasArg("off")){
    manualAt = millis();
    digitalWrite(FANPWMPIN, LOW);
    currentFanStatus =0;
    server.send ( 200, "text/html",  String(currentFanStatus));
  }else{
    server.send ( 200, "application/json",  "{\"temperature1\":" + String(temp1) + ",\"humidity1\":" + String(hum1) + "\"temperature2\":" + String(temp2) + ",\"humidity2\":" + String(hum2) +", \"lightlevel1\":" + String(L1) +",\"lightlevel2\": " + String(L2) +"}" );
  }
}
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(L1VCC,OUTPUT);
  pinMode(L2VCC,OUTPUT);
  pinMode(FANPWMPIN,OUTPUT);
  dht2.begin();
  dht1.begin();

  Serial.print("Connecting to ");
  Serial.println(ssid);
  // Print local IP address and start web server
  Serial.println("");
  //set mac address
  uint8_t mac[6] {0x5C, 0x26, 0x1A, 0x44, 0x86, 0x68}; 
  wifi_set_macaddr(0, const_cast<uint8*>(mac));
  //disable ap advertising 
  WiFi.mode(WIFI_STA);
  //connect to wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.on ( "/", getStatus );
  //
  server.begin(); //start the webserver
  Serial.println("Webserver started");
}



// the loop routine runs over and over again forever:
void loop() {
    server.handleClient();
    
    unsigned long currentMillis = millis();

    unsigned long manualTime = currentFanStatus?MANUAL_ON_TIME:MANUAL_OFF_TIME;

    if((previousMillis > currentMillis || currentMillis - previousMillis >= TEMP_READ_INTERVALL_IN_MS) && (manualAt && currentMillis - manualAt <= manualTime))
    {
      previousMillis = currentMillis;
      if(currentFanStatus)
        Serial.println("manual on ends in: " + String(MANUAL_ON_TIME - (currentMillis - manualAt)) +"ms");
      else
        Serial.println("manual off ends in: " + String(MANUAL_OFF_TIME - (currentMillis - manualAt)) +"ms");

      int tempL1 = readLightSensor(L1VCC,SENSORDELAY);
      int tempL2 = readLightSensor(L2VCC,SENSORDELAY);
      hum1 = dht1.readHumidity(); //bathroom
      hum2 = dht2.readHumidity(); //wc
      temp1 = dht1.readTemperature(); //bathroom
      temp2 = dht2.readTemperature(); //wc

      L1IsOn = tempL1> L1Threshold;   //bathroom
      L2IsOn = tempL2 > L2Threshold;   //toilet
      L1 = tempL1;
      L2 = tempL2;
      
    }else if ((previousMillis > currentMillis || currentMillis - previousMillis >= TEMP_READ_INTERVALL_IN_MS) && (!manualAt || currentMillis - manualAt > manualTime))
    {
      previousMillis = currentMillis;
      int tempL1 = readLightSensor(L1VCC,SENSORDELAY);
      int tempL2 = readLightSensor(L2VCC,SENSORDELAY);
      hum1 = dht1.readHumidity(); //bathroom
      hum2 = dht2.readHumidity(); //wc
      temp1 = dht1.readTemperature(); //bathroom
      temp2 = dht2.readTemperature(); //wc

      //turn fan on:
      Serial.println(dist(hum1,hum2));
      Serial.println(dist(hum1,hum2)>MAX_ACCEPTABLE_HUMIDITY_DELTA_ON);
      if(dist(hum1,hum2)>MAX_ACCEPTABLE_HUMIDITY_DELTA_ON || hum1> MAX_ACCEPTABLE_HUMIDITY){
        //bath humidity is high. keep fan running
        //fanOnAt = currentMillis;  //if the humidity is low again we instantly turn the fan off. this line would keep the fan running for TOILET_FAN_ON_TIME after humidity goes back down
        digitalWrite(FANPWMPIN, HIGH);
        currentFanStatus =1;
      }
      
      if(!L2IsOn && tempL2 > L2Threshold)
        {
          //toilet light was turned on now
          L2OnAt = currentMillis;
        }else if(L2IsOn && tempL2 <= L2Threshold){
          //toilet light was turned off now
          //check how long it was on for
          if(L2OnAt > currentMillis || currentMillis - L2OnAt > MIN_LIGHT_ON_TIME_TOILET){
            //turn fan on
            fanOnAt = currentMillis;
            digitalWrite(FANPWMPIN, HIGH);
            currentFanStatus =1;
          }
        }



        //turn fan off:
        if(dist(hum1,hum2)<MAX_ACCEPTABLE_HUMIDITY_DELTA_OFF && hum1 < MAX_ACCEPTABLE_HUMIDITY && (fanOnAt > currentMillis || currentMillis - fanOnAt > TOILET_FAN_ON_TIME)){
          digitalWrite(FANPWMPIN, LOW);
          currentFanStatus =0;
        }
      
      //check which lights are on/off
      L1IsOn = tempL1> L1Threshold;   //bathroom
      L2IsOn = tempL2 > L2Threshold;   //toilet
      L1 = tempL1;
      L2 = tempL2;
      //check toilet light. if it was on for MIN_LIGHT_ON_TIME_TOILET turn the fan on for TOILET_FAN_ON_TIME
    }
}
