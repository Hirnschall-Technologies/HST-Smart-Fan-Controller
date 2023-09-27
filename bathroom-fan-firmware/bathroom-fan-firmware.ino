#include <DHT.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
HTTPClient http;
WiFiClient client;
const char *ssid = "DLan";
const char *password = "PW2015dIr615!";
String homebridgeWebhook = "http://192.168.0.101:51828/?accessoryId=";
//---------------------------------------------------
#include <ESP8266WiFiMulti.h>
#include <InfluxDbClient.h>
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
#include <ESP8266WebServer.h>
ESP8266WebServer server(80);  //listen to port 80

#define DHT1PIN 14
#define DHT2PIN 4

#define FANPWMPIN 5

#define L1VCC 13
#define L2VCC 12
#define SENSORDELAY 500

#define DHTTYPE    DHT22

#define MAX_ACCEPTABLE_HUMIDITY_DELTA_ON 10
#define MAX_ACCEPTABLE_HUMIDITY_DELTA_OFF 5
#define MAX_ACCEPTABLE_HUMIDITY 97
float requestedHumidity=70;

#define TEMP_READ_INTERVALL_IN_MS 10000
#define MIN_LIGHT_ON_TIME_TOILET 180000 //3min
#define TOILET_FAN_ON_TIME 600000 //10min
#define MANUAL_ON_TIME 900000 //15min
#define MANUAL_OFF_TIME 1200000 //20min


#define TEMP_SEND_INTERVALL_IN_MS 60000
#define TEMP_INFLUX_INTERVALL_IN_MS 60000

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
unsigned long previousSendMillis =0;
unsigned long previousInfluxMillis =0;
unsigned long fanOnAt =0;
unsigned long L2OnAt = 0;
unsigned long manualAt = 0;



void sendInfluxData(float t1,float t2,float h1,float h2, int force = 0);

void setRequestedHumidity(float hum){
  if(hum<0 || hum>100)
    return;
    
  requestedHumidity = hum;
  //send hum status to homekit
  Serial.print("sending requested humidity status... - ");
  String serverPath = homebridgeWebhook + "F1&speed=" + String(requestedHumidity);
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  Serial.println(httpResponseCode);
  http.end();
}

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

void setFanState(int state){
  currentFanStatus = state;
  digitalWrite(FANPWMPIN, state?HIGH:LOW);
  sendFanStatus();
}

void sendFanStatus(){
  //send fan status to homekit
  Serial.print("sending fan status... - ");
  String tempstatus = currentFanStatus?"true":"false";
  String serverPath = homebridgeWebhook + "F1&state=" + tempstatus;
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  Serial.println(httpResponseCode);
  http.end();

  sendInfluxData(1,1,1,1,1);
}

void sendOcupancyStatus(int sensorId){
  //send fan status to homekit
  Serial.print("sending ocupancy status... - ");
  String tempstatus = "";
  if(sensorId == 1)
    tempstatus = L1IsOn?"true":"false";
  else if (sensorId == 2)
    tempstatus = L2IsOn?"true":"false";
    
  String tempid = String(sensorId);
  String serverPath = homebridgeWebhook + "O" + tempid + "&state=" + tempstatus;
  Serial.println(serverPath);
  //Serial.println(serverPath);
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  Serial.println(httpResponseCode);
  http.end();

}

void sendTemperatureStatus(float data,int sendorId){
  //send fan status to homekit
  Serial.print("sending temperature status... - ");
  String tempid = String(sendorId);
  String serverPath = homebridgeWebhook + "T" + tempid + "&value=" + data;
  //Serial.println(serverPath);
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  Serial.println(httpResponseCode);
  http.end();
}
void sendHumidityStatus(float data,int sendorId){
  //send fan status to homekit
  Serial.print("sending temperature status... - ");
  String tempid = String(sendorId);
  String serverPath = homebridgeWebhook + "H" + tempid + "&value=" + data;
  //Serial.println(serverPath);
  http.begin(client, serverPath.c_str());
  int httpResponseCode = http.GET();
  Serial.println(httpResponseCode);
  http.end();
}


void sendInfluxData(float t1,float t2,float h1,float h2, int force){
   Serial.print("writing fan status to influxdb: ");
   Serial.println(currentFanStatus);
   pointDeviceF1.clearFields();
   pointDeviceF1.addField("value", currentFanStatus);
   if (!clientDB.writePoint(pointDeviceF1)) {
     Serial.print("write failed: ");
     Serial.println(clientDB.getLastErrorMessage());
   }

   if(!force){
     Serial.println("writing o1 status to influxdb.");
     pointDeviceO1.clearFields();
     pointDeviceO1.addField("value", L1IsOn?1:0);
     if (!clientDB.writePoint(pointDeviceO1)) {
       Serial.print("write failed: ");
       Serial.println(clientDB.getLastErrorMessage());
     }
     Serial.println("writing o2 status to influxdb.");
     pointDeviceO2.clearFields();
     pointDeviceO2.addField("value", L2IsOn?1:0);
     if (!clientDB.writePoint(pointDeviceO2)) {
       Serial.print("write failed: ");
       Serial.println(clientDB.getLastErrorMessage());
     }
     Serial.println("writing dht1 status to influxdb.");
     pointDevicedht1.clearFields();
     pointDevicedht1.addField("temperature", t1);
     pointDevicedht1.addField("humidity", h1);
     if (!clientDB.writePoint(pointDevicedht1)) {
       Serial.print("write failed: ");
       Serial.println(clientDB.getLastErrorMessage());
     }
     Serial.println("writing dht2 status to influxdb.");
     pointDevicedht2.clearFields();
     pointDevicedht2.addField("temperature", t2);
     pointDevicedht2.addField("humidity", h2);
     if (!clientDB.writePoint(pointDevicedht2)) {
       Serial.print("write failed: ");
       Serial.println(clientDB.getLastErrorMessage());
     }
   }
}


void getStatus(){
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
    setRequestedHumidity(server.arg("requestHumidity").toFloat());
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

        if(!L2IsOn && tempL2 > L2Threshold)
        {
          //toilet light was turned on now
          L2IsOn = true;
          L2OnAt = currentMillis;
          sendOcupancyStatus(2);
        }else if(L2IsOn && tempL2 <= L2Threshold){
          //toilet light was turned off now
          L2IsOn = false;
          sendOcupancyStatus(2);
        }
        //L1 changes state
        if(!L1IsOn && tempL1 > L1Threshold){
          L1IsOn = true;
          sendOcupancyStatus(1);
        }
        else if(L1IsOn && tempL1 <= L1Threshold){
          L1IsOn = false;
          sendOcupancyStatus(1);
        }
      

      L1IsOn = tempL1> L1Threshold;   //bathroom
      L2IsOn = tempL2 > L2Threshold;   //toilet
      L1 = tempL1;
      L2 = tempL2;

      if(previousSendMillis > currentMillis || currentMillis - previousSendMillis >= TEMP_SEND_INTERVALL_IN_MS)
        {
          previousSendMillis = currentMillis;
          sendOcupancyStatus(1);
          sendOcupancyStatus(2);
          sendTemperatureStatus(temp1, 1);
          sendTemperatureStatus(temp2, 2);
          sendHumidityStatus(hum1, 1);
          sendHumidityStatus(hum2, 2);
        }
      if(previousInfluxMillis > currentMillis || currentMillis - previousInfluxMillis >= TEMP_INFLUX_INTERVALL_IN_MS)
        {
          previousInfluxMillis = currentMillis;
          sendInfluxData(temp1,temp2,hum1,hum2,0);
        }
      
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
      if(hum1>requestedHumidity && (dist(hum1,hum2)>MAX_ACCEPTABLE_HUMIDITY_DELTA_ON || hum1> MAX_ACCEPTABLE_HUMIDITY)){
        //bath humidity is high. keep fan running
        //fanOnAt = currentMillis;  //if the humidity is low again we instantly turn the fan off. this line would keep the fan running for TOILET_FAN_ON_TIME after humidity goes back down
        setFanState(1);
        
      }
      
      if(!L2IsOn && tempL2 > L2Threshold)
        {
          //toilet light was turned on now
          L2IsOn = true;
          L2OnAt = currentMillis;
          sendOcupancyStatus(2);
        }else if(L2IsOn && tempL2 <= L2Threshold){
          //toilet light was turned off now
          L2IsOn = false;
          sendOcupancyStatus(2);
          //check how long it was on for
          if(L2OnAt > currentMillis || currentMillis - L2OnAt > MIN_LIGHT_ON_TIME_TOILET){
            //turn fan on
            fanOnAt = currentMillis;
            setFanState(1);
          }
        }

        //L1 changes state
        if(!L1IsOn && tempL1 > L1Threshold){
          L1IsOn = true;
          sendOcupancyStatus(1);
        }
        else if(L1IsOn && tempL1 <= L1Threshold){
          L1IsOn = false;
          sendOcupancyStatus(1);
        }


        //turn fan off:
        if(hum1 <= requestedHumidity || (dist(hum1,hum2)<MAX_ACCEPTABLE_HUMIDITY_DELTA_OFF) && hum1 < MAX_ACCEPTABLE_HUMIDITY && (fanOnAt > currentMillis || currentMillis - fanOnAt > TOILET_FAN_ON_TIME)){
          setFanState(0);
        }
      
      //check which lights are on/off
      L1IsOn = tempL1> L1Threshold;   //bathroom
      L2IsOn = tempL2 > L2Threshold;   //toilet
      L1 = tempL1;
      L2 = tempL2;


      Serial.print("Reading at" + String(currentMillis) +"ms: ");
      Serial.print("\tT1=");
      Serial.print(temp1);
      Serial.print("\tH1=");
      Serial.print(hum1);
      Serial.print("\tO1=");
      Serial.print(L1IsOn);
      Serial.print("\tT2=");
      Serial.print(temp2);
      Serial.print("\tH2=");
      Serial.print(hum2);
      Serial.print("\tO2=");
      Serial.println(L2IsOn);

      if(previousSendMillis > currentMillis || currentMillis - previousSendMillis >= TEMP_SEND_INTERVALL_IN_MS)
        {
          previousSendMillis = currentMillis;
          sendOcupancyStatus(1);
          sendOcupancyStatus(2);
          sendTemperatureStatus(temp1, 1);
          sendTemperatureStatus(temp2, 2);
          sendHumidityStatus(hum1, 1);
          sendHumidityStatus(hum2, 2);
        }
      if(previousInfluxMillis > currentMillis || currentMillis - previousInfluxMillis >= TEMP_INFLUX_INTERVALL_IN_MS)
        {
          previousInfluxMillis = currentMillis;
          sendInfluxData(temp1,temp2,hum1,hum2,0);
        }
      
      //check toilet light. if it was on for MIN_LIGHT_ON_TIME_TOILET turn the fan on for TOILET_FAN_ON_TIME
    }
}
