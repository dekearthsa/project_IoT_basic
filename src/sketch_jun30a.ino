#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <dht.h>
#include <Adafruit_Sensor.h>

#define DHTTYPE DHT22 
#define DHTPIN 23

DHT dht(DHTPIN, DHTTYPE);

const char* ssid = "Bannok Kiang Chan3";
const char* password = "0818462519";

int push_button_state = 0;
int celsius;

// // init pin // //
int pin_out_led_red = 33;
int push_button_pin = 32;
int pin_out_led_blue = 2;
int pin_in_water = 34;

// test sub mosquitto_sub -h 192.168.1.43 -p 1883 -t sensor/temperature // 
// test pub mosquitto_pub -h 192.168.1.43 -p 1883 -t sensor/control -m '{"command": 0}' //
const char* mqtt_server = "192.168.1.43";
const char* mqtt_topic = "sensor/temperature";
const char* subscribe_topic = "sensor/control";

WiFiClient espClient;
PubSubClient client(espClient);

struct SensorData {
  float temp;
  float humid;
  int water;
};

void wifi_connection(){
  if (WiFi.status() != WL_CONNECTED){
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1500);
      digitalWrite(pin_out_led_blue, HIGH);
      delay(1500);
      Serial.print(".");
      digitalWrite(pin_out_led_blue, LOW);
    }
  }
}

void mqtt_reconn(){
  while (!client.connected()) {
    digitalWrite(pin_out_led_blue, HIGH);
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      digitalWrite(pin_out_led_blue, HIGH);
      client.subscribe(subscribe_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
      digitalWrite(pin_out_led_blue, LOW);
    }
  }
}

SensorData operate_fucntion(){
  SensorData setData;
  float humid = dht.readHumidity();
  float celsius = dht.readTemperature();

  // // deug logs // //
  // Serial.print("Water level: ");
  // Serial.println(analogRead(pin_in_water));
  // Serial.print("Celsius: ");
  // Serial.println(celsius);
  // Serial.print("Humid: ");
  // Serial.println(humid);

  setData.temp = celsius;
  setData.humid = humid;
  setData.water = analogRead(pin_in_water);
  return setData;
}

String createJson(float temp, int water, float humid){
  StaticJsonDocument<200> doc;
  String jsonString;

  doc["temp"] = temp;
  doc["water"] = water;
  doc["humid"] = humid;

  serializeJson(doc, jsonString);
  return jsonString;
}

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> command;
  String  commandOut;
  for (unsigned int i = 0; i < length; i++) {
    commandOut = commandOut + (char)payload[i];
  }
  DeserializationError error = deserializeJson(command, commandOut);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  int isC = command["command"];
  // // deug logs // //
  // Serial.print("Message arrived command: ");
  // Serial.println(isC);
  if (isC == 1){
    digitalWrite(pin_out_led_red, HIGH);
  }else if (isC == 0){
    digitalWrite(pin_out_led_red, LOW);
  }
}

void setup() {
  pinMode(pin_out_led_red,OUTPUT);
  pinMode(pin_out_led_blue, OUTPUT);
  pinMode(push_button_pin, INPUT);
  pinMode(pin_in_water,INPUT);
  Serial.begin(115200);
  wifi_connection();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dht.begin();
}

void loop() {
  wifi_connection();
  if (!client.connected()) {
    mqtt_reconn();
  }
  client.loop();
  SensorData dataout = operate_fucntion();
  push_button_state = digitalRead(push_button_pin);
  // // deug logs // //
  // Serial.print("btn: ");
  // Serial.print(push_button_state);
  // Serial.println("");
  if (push_button_state == 1){
    String jsonData = createJson(dataout.temp, dataout.water, dataout.humid);
    client.publish(mqtt_topic, jsonData.c_str());
  }
  delay(2000);
}
