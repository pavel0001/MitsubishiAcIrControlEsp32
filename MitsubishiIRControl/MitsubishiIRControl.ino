#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_MitsubishiHeavy.h>
#include <AsyncMqttClient.h>
#include <WiFi.h>

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
#define MQTT_TOPIC_TEMPERATURE "esp32/myowndemo/temperature"
#define MQTT_TOPIC_PRESSURE "esp32/myowndemo/pressure"
#define MQTT_TOPIC_ALTITUDE "esp32/myowndemo/altitude"

#define MQTT_TOPIC_AC "esp32/myowndemo/ac/#"
#define MQTT_TOPIC_AC_RUN "esp32/myowndemo/ac/run"
#define MQTT_TOPIC_AC_MODE "esp32/myowndemo/ac/mode"
#define MQTT_TOPIC_AC_TEMP_HEAT "esp32/myowndemo/ac/temp/heat"
#define MQTT_TOPIC_AC_TEMP_COOL "esp32/myowndemo/ac/temp/cool"

#define AC_COMMAND_ON "on"
#define AC_COMMAND_OFF "of"

#define AC_MODE_COOL "cool"
#define AC_MODE_HEAT "heat"
#define AC_MODE_AVTO "avto"
#define AC_MODE_DRY "dry"
#define AC_MODE_TURBO "turbo"

Adafruit_BMP280 bme;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0; 
const long interval = 10000;

const uint16_t kIrLed = 23;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRMitsubishiHeavy88Ac ac(kIrLed);  // Set the GPIO used for sending messages.



void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  if (strcmp(topic, MQTT_TOPIC_AC_RUN) == 0) {
    if(strcmp(messageTemp.c_str(), AC_COMMAND_ON) == 0){
      ac.setPower(true); 
    } else{
      ac.setPower(false); 
    }
  } else if (strcmp(topic, MQTT_TOPIC_AC_MODE) == 0){
    if(strcmp(messageTemp.c_str(), AC_MODE_COOL) == 0){
      ac.setMode(kMitsubishiHeavyCool);  
    }else if(strcmp(messageTemp.c_str(), AC_MODE_HEAT) == 0){
      ac.setMode(kMitsubishiHeavyHeat);  
    }else if(strcmp(messageTemp.c_str(), AC_MODE_AVTO) == 0){
      ac.setMode(kMitsubishiHeavyAuto);  
    }else if(strcmp(messageTemp.c_str(), AC_MODE_DRY) == 0){
      ac.setMode(kMitsubishiHeavyDry);  
    }else if(strcmp(messageTemp.c_str(), AC_MODE_TURBO) == 0){
      ac.setTurbo(true);
    }
  }else if (strcmp(topic, MQTT_TOPIC_AC_TEMP_HEAT) == 0 ){
        ac.setMode(kMitsubishiHeavyHeat);  
        ac.setTemp(messageTemp.toInt());  
    }else if (strcmp(topic, MQTT_TOPIC_AC_TEMP_COOL) == 0 ) {
        ac.setMode(kMitsubishiHeavyCool); 
        ac.setTemp(messageTemp.toInt());  
    }
      
  
  ac.send();
  ac.setTurbo(false);
  printState();
 
  Serial.println("Publish received.");
             //  "Опубликованные данные получены."
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
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC_AC, 1);
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
  ac.setPower(true); 
  ac.setSwingVertical(kMitsubishiHeavy88SwingVAuto);      // Swing vertically
  ac.setSwingHorizontal(kMitsubishiHeavy88SwingHAuto);
}

void loop() {
 unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    char tempArray[16];
    sprintf(tempArray, "%.2f", bme.readTemperature());

    char pressureArray[16];
    sprintf(pressureArray, "%.2f", bme.readPressure());

    char altitudeArray[16];
    sprintf(altitudeArray, "%.2f", bme.readAltitude(1013.25));

    Serial.print("Temperature: ");
    Serial.println(tempArray);
    Serial.print("Pressure: ");
    Serial.println(pressureArray);
    Serial.print("Altitude: ");
    Serial.println(altitudeArray);

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_TOPIC_TEMPERATURE, 2, true, tempArray);
    Serial.print("Publishing on topic esp32/myowndemo/temperature at QoS 2, packetId: ");
    Serial.println(packetIdPub2);
    delay(100);
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_TOPIC_PRESSURE, 2, true, pressureArray);
    Serial.print("Publishing on topic esp32/myowndemo/pressure at QoS 2, packetId: ");
    Serial.println(packetIdPub3);
    delay(100);
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_TOPIC_ALTITUDE, 2, true, altitudeArray);
    Serial.print("Publishing on topic esp32/myowndemo/altitude at QoS 2, packetId: ");
    Serial.println(packetIdPub4);
    delay(100);

    mqttClient.publish(MQTT_TOPIC_CALLBACK, 1, true, ac.toString().c_str());
  }
}