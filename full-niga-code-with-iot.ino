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

LiquidCrystal_I2C lcd(0x27, 16, 2);

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

#define state_nothing 0
#define state_heating 1
#define state_humidifying 2
#define state_suction 3

uint8_t currentState = state_nothing;

uint8_t getState(float temp, float humidity)
{
  {
    // todo
    if (t < temp_low)
    {
    }
    if (t > temp_high)
    {
    }
    if (t < hum_low)
    {
    }
    if (t > hum_high)
    {
    }
  }

  void setup()
  {

    lcd.begin();
    lcd.backlight();

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

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    int sm = analogRead(SOILPIN);
    int smPercent = map(sm, 0, 4095, 0, 100);

    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(t);
    lcd.print((char)223);
    lcd.print("C ");

    lcd.setCursor(0, 1);
    lcd.print("Hum:");
    lcd.print(h);
    lcd.print("% ");

    lcd.setCursor(9, 0);
    lcd.print("Soil:");
    lcd.print(smPercent);
    lcd.print("% ");
    // if system enable
    if (turnedOn == 1)
    {
      currentState = getState(t, h);
      if (currentState == state_nothing)
      {
        // stop all on
      }
      if (currentState == state_humidifying)
      {
        // todo  humidifier on
      }
      if (currentState == state_suction)
      {
        // fan on
      }
      if (currentState == state_heating)
      {
        // heater on
      }
    }
  }
  // Connect to MQTT broker
  if (!mqttClient.connected())
  {
    connectToMqtt();
  }
  mqttClient.publish("bloomingmushroom/feeds/temperature", String(t).c_str());
  mqttClient.publish("bloomingmushroom/feeds/humidity", String(h).c_str());
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
