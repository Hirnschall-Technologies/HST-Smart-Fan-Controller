#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#include <ESP8266WiFiMulti.h>
#include <InfluxDbClient.h>

#include <ESP8266WebServer.h>

HTTPClient http;
WiFiClient client;
const char *ssid = "";
const char *password = "";
String homebridgeWebhook = "http://192.168.0.101:51828/?accessoryId=";
//---------------------------------------------------

// InfluxDB server url, e.g. http://192.168.1.48:8086 (don't use localhost, always server name or ip address)
#define INFLUXDB_URL "http://192.168.0.101:8086"
// InfluxDB database name
#define INFLUXDB_DB_NAME "smarthome"
// Single InfluxDB instance
InfluxDBClient clientDB(INFLUXDB_URL, INFLUXDB_DB_NAME);
// Define data point with measurement name 'device_status`
Point pointDeviceF1("fan_status");
Point pointDevicedht1("dht");
Point pointDevicedht2("dht");
Point pointDeviceO1("ocupancy");
Point pointDeviceO2("ocupancy");
//---------------------------------------------------
ESP8266WebServer server(80);  //listen to port 80

//pins
#define DHT1PIN 14
#define DHT2PIN 4
#define FANPWMPIN 5
#define L1VCC 13
#define L2VCC 12


#define DHTTYPE    DHT22

int minSensorDelayInMS = 500;  //will be at least TEMP_READ_INTERVALL_IN_MS in practice


float maxHumidityDeltaWhenOn=5;
float maxHumidityDeltaWhenOff=8;
float maxHumidity=97;
int requestedHumidity=70;

unsigned int dhtSensorReadIntervallInMS=2000;
unsigned int minLightOnTimeInMS=180000; //3min
unsigned int toiletFanOnTimeInMS=600000; //10min
unsigned int manualOnTimeInMS=900000; //15min
unsigned int manualOffTimeInMS=1200000; //20min


unsigned int sendDataToHomebridgeIntervallInMS=60000;
unsigned int sendDataToInfluxDBIntervallInMS=60000;

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
int previousLid = 0;  //toggle after L1/L2 is read

bool L1IsOn=false;
bool L2IsOn=false;

short currentFanStatus =0;

unsigned long currentMillis = 0;
unsigned long previousDhtMillis =0;
unsigned long previousHomebridgeSentMillis =0;
unsigned long previousInfluxDBSentMillis =0;

unsigned long previousL1Millis =0;
unsigned long previousL2Millis =0;
unsigned long fanOnAt =0;
unsigned long L2OnAt = 0;
unsigned long manualAt = 0;



void sendInfluxData(float t1, float t2, float h1, float h2);
void setRequestedHumidity(int hum);
float dist(float x, float y);
void readDHTSensors();
void readLightSensors();
void sendDataToHomebridge();
void sendDataToInfluxDB();
void sendFanStatus();
void setFanStatus();
void sendOcupancyStatus();
void sendTemperatureStatus();
void sendHumidityStatus();
void sendInfluxData();
void respondToHttp();

void setRequestedHumidity(int hum){
  if(hum<0 || hum>100)
    return;
    
  requestedHumidity = hum;
  //send hum status to homekit
  String tempstatus = currentFanStatus?"true":"false";
  String serverPath = homebridgeWebhook + "F1&state=" + tempstatus + "&speed=" + String(requestedHumidity);
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  http.end();
}

float dist(float x, float y){
  return x>y?x-y:y-x;
}

void readDHTSensors(){
  if(previousDhtMillis > currentMillis || currentMillis - previousDhtMillis >= dhtSensorReadIntervallInMS){
    hum1 = dht1.readHumidity(); //bathroom
    hum2 = dht2.readHumidity(); //wc
    temp1 = dht1.readTemperature(); //bathroom
    temp2 = dht2.readTemperature(); //wc
    previousDhtMillis = currentMillis;
  }
}

void readLightSensors(){
  //read light sensors without blocking http responses
  if(previousLid){  //read L1
    digitalWrite(L1VCC,HIGH);
    
    if(previousL1Millis > currentMillis){  //prevent sensor delays < minLightOnTimeInMS in case of overflow of currentMillis
      previousL1Millis = currentMillis;
    }else if(currentMillis - previousL1Millis > minLightOnTimeInMS){
      L1 = analogRead(A0);
      digitalWrite(L1VCC,LOW);
      previousL1Millis = currentMillis;
      previousLid = 0;
    }
  }else{
    digitalWrite(L2VCC,HIGH);
    
    if(previousL2Millis > currentMillis){ //prevent sensor delays < minLightOnTimeInMS in case of overflow of currentMillis
      previousL2Millis = currentMillis;
    }
    if(currentMillis - previousL2Millis > minLightOnTimeInMS){
      L2 = analogRead(A0);
      digitalWrite(L2VCC,LOW);
      previousL2Millis = currentMillis;
      previousLid = 1;
    }
  }
}

void sendDataToHomebridge(){
  if(previousHomebridgeSentMillis > currentMillis || currentMillis - previousHomebridgeSentMillis >= sendDataToHomebridgeIntervallInMS)
  {
    previousHomebridgeSentMillis = currentMillis;
    sendTemperatureStatus(temp1, 1);
    sendTemperatureStatus(temp2, 2);
    sendHumidityStatus(hum1, 1);
    sendHumidityStatus(hum2, 2);
  }
}
void sendDataToInfluxDB(){
  if(previousInfluxDBSentMillis > currentMillis || currentMillis - previousInfluxDBSentMillis >= sendDataToInfluxDBIntervallInMS)
  {
    previousInfluxDBSentMillis = currentMillis;
    sendInfluxData(temp1,temp2,hum1,hum2);
  }
}

void setFanState(int state){
  if(currentFanStatus == state)
    return;
  currentFanStatus = state;
  digitalWrite(FANPWMPIN, state?HIGH:LOW);
  sendFanStatus();
}

void sendFanStatus(){
  //send fan status to homekit
  String tempstatus = currentFanStatus?"true":"false";
  String serverPath = homebridgeWebhook + "F1&state=" + tempstatus;
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  http.end();
  //send influxdb data
  pointDeviceF1.clearFields();
  pointDeviceF1.addField("value", currentFanStatus*requestedHumidity);
}

void sendOcupancyStatus(int sensorId){
  //send fan status to homekit
  String tempstatus = "";
  if(sensorId == 1)
    tempstatus = L1IsOn?"true":"false";
  else if (sensorId == 2)
    tempstatus = L2IsOn?"true":"false";
    
  String tempid = String(sensorId);
  String serverPath = homebridgeWebhook + "O" + tempid + "&state=" + tempstatus;
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  http.end();

  //influx data
  pointDeviceO1.clearFields();
  pointDeviceO1.addField("value", L1IsOn?1:0);
  pointDeviceO2.clearFields();
  pointDeviceO2.addField("value", L2IsOn?1:0);
}

void sendTemperatureStatus(float data,int sendorId){
  //send fan status to homekit
  String tempid = String(sendorId);
  String serverPath = homebridgeWebhook + "T" + tempid + "&value=" + data;
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  http.end();
}
void sendHumidityStatus(float data,int sendorId){
  //send fan status to homekit
  String tempid = String(sendorId);
  String serverPath = homebridgeWebhook + "H" + tempid + "&value=" + data;
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  http.end();
}


void sendInfluxData(float t1,float t2,float h1,float h2){
  pointDeviceF1.clearFields();
  pointDeviceF1.addField("value", currentFanStatus*requestedHumidity);
  pointDeviceO1.clearFields();
  pointDeviceO1.addField("value", L1IsOn?1:0);
  pointDeviceO2.clearFields();
  pointDeviceO2.addField("value", L2IsOn?1:0);
  pointDevicedht1.clearFields();
  pointDevicedht1.addField("temperature", t1);
  pointDevicedht1.addField("humidity", h1);
  pointDevicedht2.clearFields();
  pointDevicedht2.addField("temperature", t2);
  pointDevicedht2.addField("humidity", h2);
}

void respondToHttp(){
  if(server.hasArg("state")){
    manualAt = millis();
    if(server.arg("state").toInt()>0){
      setFanState(1);
      server.send ( 200, "text/html",  String(currentFanStatus));
    }else{
      setFanState(0);
      server.send ( 200, "text/html",  String(currentFanStatus));
    }
  }else if(server.hasArg("requestHumidity")){
    server.send ( 200, "text/html",  String(currentFanStatus));
    setRequestedHumidity(server.arg("requestHumidity").toInt());
  }else if(server.hasArg("setDeltaWhenOn")){
    maxHumidityDeltaWhenOn = server.arg("setDeltaWhenOn").toFloat();
    server.send ( 200, "text/html",  String(maxHumidityDeltaWhenOn));
  }else if(server.hasArg("setDeltaWhenOff")){
    maxHumidityDeltaWhenOff = server.arg("setDeltaWhenOff").toFloat();
    server.send ( 200, "text/html",  String(maxHumidityDeltaWhenOff));
  }else if(server.hasArg("setMaxHumidity")){
    maxHumidity = server.arg("setMaxHumidity").toFloat();
    server.send ( 200, "text/html",  String(maxHumidity));
  }else if(server.hasArg("setReadIntervall")){
    dhtSensorReadIntervallInMS = server.arg("setReadIntervall").toInt();
    server.send ( 200, "text/html",  String(dhtSensorReadIntervallInMS));
  }else if(server.hasArg("setMinToiletTime")){
    minLightOnTimeInMS = server.arg("setMinToiletTime").toInt();
    server.send ( 200, "text/html",  String(minLightOnTimeInMS));
  }else if(server.hasArg("setFanOnTime")){
    toiletFanOnTimeInMS = server.arg("setFanOnTime").toInt();
    server.send ( 200, "text/html",  String(toiletFanOnTimeInMS));
  }else if(server.hasArg("setManualOnTime")){
    manualOnTimeInMS = server.arg("setManualOnTime").toInt();
    server.send ( 200, "text/html",  String(manualOnTimeInMS));
  }else if(server.hasArg("setManualOffTime")){
    manualOffTimeInMS = server.arg("setManualOffTime").toInt();
    server.send ( 200, "text/html",  String(manualOffTimeInMS));
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
  server.on ( "/", respondToHttp );
  //
  server.begin(); //start the webserver
  Serial.println("Webserver started");

  
  pointDeviceF1.addTag("device", "luefter-bad");
  pointDevicedht1.addTag("device", "dht-bad");
  pointDevicedht2.addTag("device", "dht-klo");
  pointDeviceO1.addTag("device", "besetzt-bad");
  pointDeviceO2.addTag("device", "besetzt-klo");

  setRequestedHumidity(requestedHumidity);
}



// the loop routine runs over and over again forever:
void loop() {
    server.handleClient();
    
    unsigned long currentMillis = millis();
    unsigned long manualTime = currentFanStatus?manualOnTimeInMS:manualOffTimeInMS;


    readLightSensors();  //read light sensors alternating with respect to minSensorDelayInMS
    readDHTSensors(); //read temperature and humidity

    //check if manual control is still active
    if(manualAt >= currentMillis || currentMillis - manualAt > manualTime)
      manualAt = 0;

    //turn fan on if humidity is high
    if(!manualAt && !currentFanStatus && (hum1>requestedHumidity || hum1> maxHumidity) && (dist(hum1,hum2) > maxHumidityDeltaWhenOff || hum1> maxHumidity)){
      setFanState(1);
    }

    //check if light state has changed to detect when to turn on toilet fan to remove smells
    if(!L2IsOn && L2 > L2Threshold)
    {
      //toilet light was turned on now
      L2IsOn = true;
      L2OnAt = currentMillis;
      sendOcupancyStatus(2);
    }else if(L2IsOn && L2 <= L2Threshold){
      //toilet light was turned off now
      L2IsOn = false;
      //check how long it was on for
      if(!manualAt && !currentFanStatus && (L2OnAt > currentMillis || currentMillis - L2OnAt > minLightOnTimeInMS)){
        //turn fan on
        fanOnAt = currentMillis;
        setFanState(1);
      }else if(L2OnAt > currentMillis || currentMillis - L2OnAt > minLightOnTimeInMS){
        fanOnAt = currentMillis;
      }
      sendOcupancyStatus(2);
    }
    //L1 changes state
    if(!L1IsOn && L1 > L1Threshold){
      L1IsOn = true;
      sendOcupancyStatus(1);
    }
    else if(L1IsOn && L1 <= L1Threshold){
      L1IsOn = false;
      sendOcupancyStatus(1);
    }

    //turn fan off:
    if(!manualAt && (hum1 <= requestedHumidity || dist(hum1,hum2)<maxHumidityDeltaWhenOn) && hum1 < maxHumidity && (fanOnAt > currentMillis || currentMillis - fanOnAt > toiletFanOnTimeInMS)){
      setFanState(0);
    }
    

    //send data to homebridge and influxdb for data logging
    sendDataToHomebridge();
    sendDataToInfluxDB();

}
