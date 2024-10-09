#include <DHT.h> //Library for the temparature sensor
#include "LiquidCrystal_I2C.h"  //Library for the LCD display
#include <WiFi.h>
#include <ThingSpeak.h> //Library for web dashboard

#define WIFI_NETWORK "Dilee_ZMI" 
#define WIFI_PASSWORD "Dilee2000914"
#define WIFI_TIMEOUT_MS 200000

#define CHANNEL_IP 2295777
#define CHANNEL_API_KEY "3GTMQGV7A92R0KX4"

void connectsToWiFi()
{
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK,WIFI_PASSWORD);
  unsigned long startAttempting = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - startAttempting < WIFI_TIMEOUT_MS)
  {
    Serial.print(".");
    delay(100);
  }
  
  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Failed connecting to the internet.");
    exit(1);
  }
  else
  {
    Serial.print("Connected ");
    Serial.println(WiFi.localIP());
  }
}

/*common variables*/
float voltage, adcRaw;

/*constants/pins for temparature/humidity*/
int temp_sensor = 33;
float temp, humidity;

/*constants/pins for water level ultrasocic sensor*/
int trigPin = 18;
int echoPin = 19;

/*constants/pins for moisture sensor*/
int moisure_sensor = 32;
float moisture;

/*constants for Turbidity sensor*/
int turbidityLED = 13;
int turbidityLDR = 35;
int Turbidity_percentage = 0;

/*Output actuators*/
/*buzzer*/
int buzzer = 27;

/*LED*/
int redLED = 25;
int greenLED = 26;

/*pump*/
int pump = 4;

/*object initialization*/
LiquidCrystal_I2C display(0x27,16,2);
WiFiClient client;
DHT dht(temp_sensor, DHT11);

void setup() {

  Serial.begin(115200);
  connectsToWiFi();
  ThingSpeak.begin(client);
  dht.begin(); 

  /*set up the output pins*/
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(turbidityLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pump, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  display.init();
  display.backlight();
  display.clear();

  delay(2000);
}

void loop() {

  /*Temparature sensor*/
  temp = dht.readTemperature();
  humidity = dht.readHumidity(); 

  display.setCursor(0,0);
  display.print("Temp: ");
  display.print(temp);
  display.print("C");
  display.setCursor(0,1);
  display.print("Humid: ");
  display.print(humidity);
  display.print("%");
  delay(2000);
  display.clear();

  // Serial.print("Temp: ");
  // Serial.print(temp);
  // Serial.print("C ");
  // Serial.print("Humidity: ");
  // Serial.print(humidity);
  // Serial.println(" %");
  
  /*moisture sensor*/
  adcRaw = analogRead(moisure_sensor);
  moisture = map(adcRaw,0,4096,100,0);

  display.setCursor(0,0);
  display.print("Moisture: ");
  display.print(moisture);
  display.print(" %");
  delay(1000);
  

  // Serial.print("Moisture: ");
	// Serial.print(moisture);
	// Serial.println(" %");

  if(moisture<30)
  {
    digitalWrite(pump, HIGH);
    delay(2000);
    digitalWrite(pump, LOW);
  }
  
  display.clear();
  /*Turbidity sensor*/
  digitalWrite(turbidityLED, HIGH); 
  delay(1000);
  adcRaw = analogRead(turbidityLDR);
  Turbidity_percentage = map(adcRaw, 1350,2950 , 100, 0);
  digitalWrite(turbidityLED, LOW);

  display.setCursor(0,0);
  display.print("Turbidity: ");
  display.print(Turbidity_percentage);
  display.print(" %");
  delay(1000);
  display.clear();

  // Serial.print("Turbidity: ");
	// Serial.print(Turbidity_percentage);
	// Serial.println(" %");

  if(Turbidity_percentage<50)
  {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
  }
  else
  {
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
  }

  /*Water level sensor*/
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH);
  float distanceCM = (duration * 0.0343) / 2.0;
  float water_level = 7-distanceCM;

  display.setCursor(0,0);
  display.print("Water level: ");
  display.print(water_level);
  display.print(" cm");
  delay(1000);
  display.clear();

  // Serial.print("Distance: ");
  // Serial.print(distanceCM);
  // Serial.println(" cm");
  // delay(1000); 

  if(water_level<2)
  {
    tone(buzzer, 1000, 200);
    delay(200);
    tone(buzzer, 1000, 200);
  }
   
  /*Uploading to the web dashboard*/
  display.setCursor(0,0);
  display.print("Uploading...");

  ThingSpeak.setField(2,water_level);
  ThingSpeak.setField(4,temp);
  ThingSpeak.setField(3,Turbidity_percentage);
  ThingSpeak.setField(1,moisture);
  ThingSpeak.writeFields(CHANNEL_IP, CHANNEL_API_KEY);

  delay(15000);

  display.clear();
}

