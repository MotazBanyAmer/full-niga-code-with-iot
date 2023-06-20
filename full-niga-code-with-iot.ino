#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#include <WiFi.h>
#include <PubSubClient.h>

#define DHTPIN 27
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

#define SOILPIN 14
#define BUZZERPIN 18

#define button 4
// TODO define relays pins numbers
#define relay_heater 12 // todo chagne the number of pins
#define relay_fan 13
#define relay_humidifier 16
#define relay_light 17

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define lightOnSetpoint_hours 4
uint32_t lightOnSetpoint_seconds = 0;
uint32_t systemOnTime = 0;

// WiFi credentials
const char *ssid = "RoKiTech";
const char *password = "RokiTech!";

// Adafruit IO MQTT broker details
const char *server = "io.adafruit.com";
const int port = 1883;
const char *username = "bloomingmushroom";
const char *key = "aio_bulh54atpNUoSk3N6DEkx6lWuLgV";

// WiFi client and MQTT client objects
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
#define temp_low 20
#define temp_high 50
#define hum_low 20
#define hum_high 50

// #define state_nothing 0
// #define state_heating 1
// #define state_humidifying 2
// #define state_suction 3

uint8_t currentState = state_nothing;
/*
uint8_t getState(float temp, float humidity)
{

  // todo
  if (temp < temp_low)
  {
  }
  if (temp > temp_high)
  {
  }
  if (humidity < hum_low)
  {
  }
  if (humidity > hum_high)
  {
  }
}
*/
bool turnedOn = 0;
//! note: if the devices didnt turn on ON function/command, flip HIGH and LOW on code, or change on the relay terminal from NC to NO connector
void turn_light_On()
{
  digitalWrite(relay_light, HIGH);
}
void turn_light_Off()
{
  digitalWrite(relay_light, LOW);
}

void turn_humidifier_On()
{
  digitalWrite(relay_humidifier, HIGH);
}
void turn_humidifier_Off()
{
  digitalWrite(relay_humidifier, LOW);
}

void turn_heater_On()
{
  digitalWrite(relay_heater, HIGH);
}
void turn_heater_Off()
{
  digitalWrite(relay_heater, LOW);
}

void turn_fan_On()
{
  digitalWrite(relay_fan, HIGH);
}
void turn_fan_Off()
{
  digitalWrite(relay_fan, LOW);
}

void setup()
{
  lightOnSetpoint_seconds = lightOnSetpoint_hours * 60 * 60; // hrs*60m*60s

  lcd.begin();
  lcd.backlight();
  pinMode(button, INPUT_PULLUP);

  pinMode(relay_fan, OUTPUT);
  pinMode(relay_heater, OUTPUT);
  pinMode(relay_light, OUTPUT);
  pinMode(relay_humidifier, OUTPUT);

  turn_light_On();

  pinMode(BUZZERPIN, OUTPUT);
  // Start serial communication
  Serial.begin(115200);
  delay(2000);
  // Connect to WiFi
  Serial.println("Connecting To: ");
  Serial.println(ssid);
  Serial.println(password);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set up MQTT client
  mqttClient.setServer(server, port);
}

void loop()
{

  float value_humidity = dht.readHumidity();
  float value_temperature = dht.readTemperature();
  int sm = analogRead(SOILPIN);
  int smPercent = map(sm, 0, 4095, 0, 100);

  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.print(value_temperature);
  lcd.print((char)223);
  lcd.print("C ");

  lcd.setCursor(0, 1);
  lcd.print("Hum:");
  lcd.print(value_humidity);
  lcd.print("% ");

  lcd.setCursor(9, 0);
  lcd.print("Soil:");
  lcd.print(smPercent);
  lcd.print("% ");

  if (digitalRead(button) == 0)
  {
    delay(350); // debouncing
    turnedOn != turnedOn;
    systemOnTime = millis() / 1000;
  }
  // if system enable
  if (turnedOn == 1)
  {

    if (millis() / 1000 > (lightOnSetpoint_seconds + systemOnTime)) // time needed + start time when we pressed the button
    {
      turn_light_Off();
    }

    // procedure

    if (value_temperature < temp_low)
    {
      turn_heater_On();
      turn_fan_Off();
      delay(5000);
    }
    if (value_temperature > temp_high)
    {
      turn_heater_Off();
      turn_fan_On();
      delay(5000);
    }
    if (value_humidity < hum_low)
    {
      turn_humidifier_On();
      turn_fan_Off();
      delay(5000);
    }
    if (value_humidity > hum_high)
    {
      turn_humidifier_Off();
      turn_fan_On();
      delay(5000);
    }

    /*
    below is not used, delete if the above is enough

    // currentState = getState(value_temperature, value_humidity);

    // if (currentState == state_humidifying)
    // {
    todo humidifier on
    // }
    // if (currentState == state_suction)
    // {
    //   // fan on
    // }
    // if (currentState == state_heating)
    // {
    //   // heater on
    // }
    */
  }

  // Connect to MQTT broker
  if (!mqttClient.connected())
  {
    connectToMqtt();
  }
  mqttClient.publish("bloomingmushroom/feeds/temperature", String(value_temperature).c_str());
  mqttClient.publish("bloomingmushroom/feeds/humidity", String(value_humidity).c_str());
  delay(2500);
  mqttClient.publish("bloomingmushroom/feeds/light", String(lPercent).c_str());
  mqttClient.publish("bloomingmushroom/feeds/soil", String(smPercent).c_str());

  // Disconnect from MQTT broker
  mqttClient.disconnect();

  delay(5000);
}
// Helper function to connect to MQTT broker
void connectToMqtt()
{
  while (!mqttClient.connected())
  {
    if (mqttClient.connect("aqsa", username, key))
    {
      Serial.println("Connected to MQTT broker");
    }
    else
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds...");
      delay(5000);
    }
  }
}
