#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "Bannok Kiang Chan3";
const char* password = "0818462519";

int push_button_state = 0;
int celsius = 0;

int pin_out_led_red = 33;
int push_button_pin = 32;
int pin_out_led_blue = 2;
int pin_in_temp = 35;
int pin_in_water = 34;

// test sub mosquitto_sub -h 192.168.1.43 -p 1883 -t sensor/temperature // 
// test pub mosquitto_pub -h 192.168.1.43 -p 1883 -t sensor/control -m '{"command": 0}' //
const char* mqtt_server = "192.168.1.43";
const char* mqtt_topic = "sensor/temperature";
const char* subscribe_topic = "sensor/control";

WiFiClient espClient;
PubSubClient client(espClient);

struct SensorData {
  int temp;
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
  celsius = map(((analogRead(pin_in_temp) - 20) * 3.04), 0, 1023, -40, 125);
  Serial.print("Water level: ");
  Serial.println(analogRead(pin_in_water));
  Serial.print("Celsius: ");
  Serial.println(celsius);
  setData.temp = celsius;
  setData.water = analogRead(pin_in_water);
  return setData;
}

String createJson(int temp, int water){
  StaticJsonDocument<200> doc;
  doc["temp"] = temp;
  doc["water"] = water;
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> command;
  String  commandOut;
  // Serial.print("Message arrived [");
  // Serial.print(topic);
  // Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    // Serial.print((char)payload[i]);
    commandOut = commandOut + (char)payload[i];
  }
  DeserializationError error = deserializeJson(command, commandOut);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  int isC = command["command"];
  Serial.print("Message arrived command: ");
  Serial.println(isC);
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
  pinMode(pin_in_temp,INPUT);
  Serial.begin(115200);
  wifi_connection();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  wifi_connection();
  if (!client.connected()) {
    mqtt_reconn();
  }
  client.loop();
  SensorData dataout = operate_fucntion();
  push_button_state = digitalRead(push_button_pin);
  Serial.print("btn: ");
  Serial.print(push_button_state);
  Serial.println("");
  if (push_button_state == 1){
    String jsonData = createJson(dataout.temp, dataout.water);
    client.publish(mqtt_topic, jsonData.c_str());
  }
  delay(2000);
}
