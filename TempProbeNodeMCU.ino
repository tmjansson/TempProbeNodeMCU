/*
 * Temperatute controller program for NodeMCU
 * 
 * Using OneWire and Dallasprobe
 * 
 * OneWire Temp Probe Digital Pin D2
 * LCD Display (D0,D1,D3,D4); //SCL(D0), SDA(D1), RST(RES), DC(DC)
 * Heating relay Digital Pin D5
 * Cooling relay Digital pin D6
 * 
 * Reads first six bytes from EEPROM as Unit ID
 * Default min and max temperatures are in global varaibles.
 * Protection delays are in the variables
 * 
 * MqTT io Key : 1f7cd97a8b974e40a843f14c5862f974
 * 
 * --------
 * Left to do:
 * - Integrate to configuration server (not built probobly some wesbervicerequest with GUI to set configuration)
 * - Change pins for heating / cooling relays
 * 
 */

#include <ESP8266WiFi.h>
const char* ssid = "OakLaneCraftBrewery";
const char* password = "kalleanka";


//MQTT Config
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "t_m_jansson"
#define AIO_KEY         "1f7cd97a8b974e40a843f14c5862f974"

WiFiClient   client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish templogger = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temploggers");

long mqttTimer = 30000; // Min delay between MQTT update
long lastMqttUpdate = 0;

// OLED Display
#include <OLED.h>
OLED OLED;

#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>  


// Variables
 char idOut[] = {'_', '_', '_', '_', '_', '_'}; 
 String ID;
int minTemp = 23; // min temp before heating turns on
int maxTemp = 26; // max temp before cooling turns on
long offCoolerTimeStamp;
long offHeaterTimeStamp;
String status = "All Off!     ";
char statusC[15] = "              ";
int heaterPinValue;
int coolerPinValue;
long timeStamp;

/***************** ONEWIRE and RELAY pins *********************/
#define ONEWIRE_PIN D2
#define HEATER_PIN D5
#define COOLER_PIN D6
#define MIN_OFF_COOLING_TIME 60000 // 1 mins * 60 sec * 1000 ms
#define MIN_OFF_HEATER_TIME 60000 // 1 mins * 60 sec * 1000 ms

/************ Global State (you don't need to change this!) ******************/
// 1wire
OneWire oneWire(ONEWIRE_PIN); 
DallasTemperature sensors(&oneWire); 
 boolean TempSensorAvailable = false;
DeviceAddress TempSensor;
float tempC;

/*************************** Sketch Code ************************************/
// Connect to MQTT
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

/* Begin the program */

void setup() {

/*Initalize OLED */
   OLED.LEDPIN_Init(D0,D1,D3,D4); //SCL, SDA, RST, DC
   OLED.LED_Init();
   OLED.LED_CLS();
  
/* Start Serial interface */
  Serial.begin(115200);
  delay(10);

/* Read Unit ID from EEPROM */
  for (int i=0;i<6;i++){
      idOut[i] = EEPROM.read(i);
      ID = ID + idOut[i];
  }
  
  Serial.println("");
  Serial.println("--------------------------------");
  Serial.println("Startup of unit....");
  Serial.println("--------------------------------");
  Serial.print("UnitID : ");
  Serial.println(ID);

  // Connect to WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  OLED.LED_P6x8Str(0,0,"Connecting to WiFi:");
  
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  OLED.LED_P6x8Str(0,0,"ConnectEd to WiFi!  ");

  // start sensors
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" OneWire device(s).");
   // report parasite power requirements
  Serial.print("Parasite power: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
   if (!sensors.getAddress(TempSensor, 0)) {
    Serial.println("No OneWire Device Found");
  } else {
    TempSensorAvailable = true;
    Serial.println("OneWire Device Found");
    sensors.setResolution(TempSensor, 12);
  }
  Serial.println("Reset status, turn both heating and cooling off.");

  /* CODE TO TURN BOTH COOLING AND HEATING PIN TO LOW!!!!!! */
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(COOLER_PIN, OUTPUT);
  digitalWrite(COOLER_PIN, HIGH);
  digitalWrite(HEATER_PIN,HIGH);
  offCoolerTimeStamp = millis();
  offHeaterTimeStamp = millis();
}

/* loop starts here */
void loop() {

  // get temp
  sensors.requestTemperatures();          // Get temperature
  tempC = sensors.getTempC(TempSensor);   // save temperature

  //Print temp
  Serial.print("Temp C: ");   
  Serial.print(tempC);        // Serial print the C temperature
  Serial.print(". Status is : ");   
  Serial.println(status);

  //Write temp to OLED
  OLED.LED_P6x8Str(0,0,"Temperature :        ");
  OLED.LED_P6x8Str(0,4,"Status :        ");
  OLED.LED_PrintValueF(0,2,tempC, 1);
  status.toCharArray(statusC, 15);
  OLED.LED_P6x8Str(0,6,statusC);
  
// Sending data to IO.ADAFRUIT.COM via MQTT
  if (millis() > (lastMqttUpdate + mqttTimer)){
    MQTT_connect();
    Serial.print(F("\Sending photocell val "));
    Serial.print(tempC);
    Serial.print("...");
    if (! templogger.publish(tempC)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    lastMqttUpdate = millis();
  } 
  // WRITE CODE TO READ AND UPDATE NEW TARGET TEMP

/* Logic for turing on and of heating / cooling */
/* If temp to low */
if (tempC < minTemp) {
  //Serial.println("Temp to low.");
  if (status != "Heating") {
    if (status == "Cooling"){
        Serial.println("Turning cooling off.");
        //CODE TO TURN COOLING PIN TO LOW!!!!!!
        digitalWrite(COOLER_PIN, HIGH);
        offCoolerTimeStamp = millis();
        }
    Serial.println("Turning heating on.");
    timeStamp = millis();
    //CODE TO TURN HEATING PIN TO HIGH!!!!!!
    if (timeStamp-offHeaterTimeStamp >= MIN_OFF_HEATER_TIME) {
      digitalWrite(HEATER_PIN,LOW);    
      status = "Heating";
      }
      else {
        Serial.print("Heating has been off to short time - posponing it for ");
        Serial.print((MIN_OFF_HEATER_TIME-(timeStamp-offHeaterTimeStamp))/1000);
        Serial.println(" seconds.");
        }
    }
  }
/*If temp too high */
if (tempC > maxTemp) {
  //Serial.println("Temp to high.");
  if (status != "Cooling") {
    if (status == "Heating") {
      Serial.println("Turning heating off.");
      //CODE TO TURN HEATING PIN TO LOW!!!!!!
      digitalWrite(HEATER_PIN,HIGH);
      offHeaterTimeStamp = millis();
      }
    Serial.println("Turning cooling on.");
    timeStamp=millis();
    //CODE TO TURN COOLING PIN TO HIGH!!!!!!
    if (timeStamp-offCoolerTimeStamp >= MIN_OFF_COOLING_TIME) {
      digitalWrite(COOLER_PIN,LOW);
      status = "Cooling";
      }
      else {
        Serial.print("Cooling has been off to short time - posponing it for ");
        Serial.print((MIN_OFF_COOLING_TIME-(timeStamp-offCoolerTimeStamp))/1000);
        Serial.println(" seconds.");
        }
    }
  }
/* If temp within normal range */
if (tempC >= minTemp && tempC <= maxTemp) {
  if (status == "Cooling") {
    Serial.println("Turning cooling off.");
    //CODE TO TURN COOLING PIN TO LOW!!!!!!
    digitalWrite(COOLER_PIN, HIGH);
    offCoolerTimeStamp = millis();
    }
  if (status == "Heating"){
    Serial.println("Turning heating off.");
    //CODE TO TURN HEATING PIN TO LOW!!!!!!
    digitalWrite(HEATER_PIN, HIGH);
    offHeaterTimeStamp = millis();
    }
    status = "All OFF!";
  }

  delay(2000);

/* End of program loop */
}
