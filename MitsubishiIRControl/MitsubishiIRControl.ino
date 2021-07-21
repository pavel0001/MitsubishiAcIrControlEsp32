#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_MitsubishiHeavy.h>
#include <AsyncMqttClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define WIFI_SSID "BS_WiFi"
#define WIFI_PASSWORD "BSWIFI524"

#define MQTT_HOST IPAddress(5, 196, 95, 208)
#define MQTT_PORT 1883

#define MQTT_TOPIC_ROOT "esp32/myowndemo/+"
#define MQTT_TOPIC_CALLBACK "esp32/myowndemo/callback"
#define MQTT_TOPIC_SENSOR "esp32/myowndemo/sensor"
#define MQTT_TOPIC_JSON_CMD  "esp32/myowndemo/json"

Adafruit_BMP280 bme;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0; 
const long interval = 10000;

const uint16_t kIrLed = 23;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRMitsubishiHeavy88Ac ac(kIrLed);  // Set the GPIO used for sending messages.

void sendAcStateJson(){
    char tmpBuff[150];
  StaticJsonDocument<200> doc;

  doc["clean"] = ac.getClean() ? 1 : 0;
  doc["fan"] = ac.getFan();
  doc["mode"] = ac.getMode();
  doc["power"] = ac.getPower()? 1 : 0;
  doc["swingH"] = ac.getSwingHorizontal();
  doc["swingV"] = ac.getSwingVertical();
  doc["temp"] = ac.getTemp();
  doc["threeD"] = ac.get3D()? 1 : 0;
  doc["turbo"] = ac.getTurbo()? 1 : 0;
  serializeJson(doc, tmpBuff);
  Serial.println("Print result json converting");
  Serial.println(tmpBuff);
  uint16_t packetIdPub3 = mqttClient.publish(MQTT_TOPIC_CALLBACK, 1, true,tmpBuff);
}

void setAcState(int power, int temp, int mode, int fan, int swingV, int swingH, int turbo, int clean, int threeD){
ac.setPower(power == 1);
ac.setTemp(temp);
ac.setMode(mode);
ac.setFan(fan);
ac.setSwingVertical(0b100);
ac.setSwingHorizontal(0b1000);
ac.setTurbo(turbo == 1);
ac.setClean(clean == 1);
ac.set3D(threeD == 1);
printState();
ac.send();
}
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  StaticJsonDocument<300> doc;
  String messageTemp;
  for (int i = 0; i < len; i++) {
    messageTemp += (char)payload[i];
  }
  DeserializationError error = deserializeJson(doc, messageTemp);
  if (strcmp(topic, MQTT_TOPIC_JSON_CMD) == 0) {
    if(!error){
      int clean = doc["clean"];
      int fan = doc["fan"];
      int mode = doc["mode"];
      int power = doc["power"];
      int swingH = doc["swingH"];
      int swingV = doc["swingV"];
      int temp = doc["temp"];
      int threeD = doc["threeD"];
      int turbo = doc["turbo"];
      setAcState( power,  temp , mode,  fan,  swingV,  swingH,  turbo,  clean,  threeD);
      sendAcStateJson();
    }
    else{
      Serial.println("Error parsed json error");
    }
  } 
  Serial.println("Publish received.");
  Serial.print("  message: ");  //  "  сообщение: "
  Serial.println(messageTemp);
  Serial.print("  topic: ");  //  "  топик: "
  Serial.println(topic);
}
void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
            //  "Публикация подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
            //  "Отписка подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
}
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
             //  "Подписка подтверждена."
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
             //  "Отключились от MQTT."
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");  //  "Подключились к MQTT."
  Serial.print("Session present: ");     //  "Текущая сессия: "
  Serial.println(sessionPresent);
  // ESP32 подписывается на топик esp32/led
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC_JSON_CMD, 1);
  Serial.print("Subscribing at QoS 0, packetId: ");  //  "Подписка при QoS 0, ID пакета: "
  Serial.println(packetIdSub);
}
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");  //  "Подключились к WiFi"
      Serial.println("IP address: ");    //  "IP-адрес: "
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      //  "WiFi-связь потеряна"
      // делаем так, чтобы ESP32
      // не переподключалась к MQTT
      // во время переподключения к WiFi:
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void printState() {
  Serial.println("Mitsubishi Heavy A/C remote is in the following state:");
  Serial.printf("  %s\n", ac.toString().c_str());
  unsigned char* ir_code = ac.getRaw();
  Serial.print("IR Code: 0x");
  for (uint8_t i = 0; i < kMitsubishiHeavy88StateLength; i++)
    Serial.printf("%02X", ir_code[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
   if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  connectToWifi();
  WiFi.onEvent(WiFiEvent);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  ac.begin();
}

void loop() {
 unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

  char tmpBuff[100];
  StaticJsonDocument<200> doc;
  doc["temperature"] = bme.readTemperature();
  doc["pressure"] = bme.readPressure();
  doc["altitude"] = bme.readAltitude(1013.25);
  serializeJson(doc, tmpBuff);
  Serial.println("Print result json converting");
  Serial.println(tmpBuff);

  uint16_t packetIdPub2 = mqttClient.publish(MQTT_TOPIC_SENSOR, 1, true, tmpBuff);
  Serial.print("Publishing on topic esp32/myowndemo/temperature at QoS 2, packetId: ");
  Serial.println(packetIdPub2);
  delay(100);
  sendAcStateJson();
  }
}