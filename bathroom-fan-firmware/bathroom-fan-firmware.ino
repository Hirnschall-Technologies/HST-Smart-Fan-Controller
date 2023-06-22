#include <DHT.h>
#include <ESP8266WiFi.h>
const char *ssid = "DLan";
const char *password = "PW2015dIr615!";
//---------------------------------------------------
#include <ESP8266WebServer.h>
ESP8266WebServer server(80);  //listen to port 80

#define DHT1PIN 14
#define DHT2PIN 4

#define FANPWMPIN 

#define L1VCC 13
#define L2VCC 12
#define SENSORDELAY 500

#define DHTTYPE    DHT22

DHT dht1(DHT1PIN, DHTTYPE);
DHT dht2(DHT2PIN, DHTTYPE);


float minDistance = 15;
float onLightsOff = 60;
float onLightOn = 70;

int L1Threshold = 200;
int L2Threshold = 200;

bool L1IsOn;
bool L2IsOn;

int dist(int x, int y){
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
  
}
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(L1VCC,OUTPUT);
  pinMode(L2VCC,OUTPUT);
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
  server.begin(); //start the webserver
  Serial.println("Webserver started");
}

// the loop routine runs over and over again forever:
void loop() {
    server.handleClient();
    //check which lights ar on/off
    L1IsOn = readLightSensor(L1VCC,SENSORDELAY) > L1Threshold;   //bathroom
    L2IsOn = readLightSensor(L2VCC,SENSORDELAY) > L2Threshold;   //toilet
    
    if(L2IsOn)
    {
      //turn fan off
      
    } else if(readLightSensor(L2VCC,SENSORDELAY) > L2Threshold) //light 1 on
    {
      
    }
    Serial.print("Humidity1: ");
    Serial.print(dht1.readHumidity());
    Serial.print(" %, Temp1: ");
    Serial.print(dht1.readTemperature());
    Serial.println(" Celsius");
    Serial.print(readLightSensor(L1VCC,SENSORDELAY));
    Serial.println(" Light1\n");
    Serial.print("Humidity2: ");
    Serial.print(dht2.readHumidity());
    Serial.print(" %, Temp2: ");
    Serial.print(dht2.readTemperature());
    Serial.println(" Celsius");
    Serial.print(readLightSensor(L2VCC,SENSORDELAY));
    Serial.println(" Light2\n");
    Serial.println("----------------------------------------\n");
  delay(2000);        // delay in between reads for stability
}
