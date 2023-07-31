/*
  LILIGO T-Relay MQTT sushilnia NPG 2023 published to topic
  Byala Robots Club
  author: Anton Anchev
*/
#include <WiFi.h>
#include "DHT.h"
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

#define DHTPIN1 14  // пин 14 свързан към сензора
#define DHTPIN2 15  // пин 14 свързан към сензора
#define DHTTYPE DHT21
#define RELAY_PIN_1 21
#define RELAY_PIN_2 19
#define RELAY_PIN_3 18
#define RELAY_PIN_4 5

DHT dht14(DHTPIN1, DHTTYPE);
DHT dht15(DHTPIN2, DHTTYPE);

//--- NETWORK CREDENTIALS
#define WIFI_SSID "RobotikaGD"
#define WIFI_PASSWORD "12345678"

// --- Static IP address -----
IPAddress local_IP(192, 168, 0, 10);  // set your Static IP address
IPAddress gateway(192, 168, 0, 1);    // Set your Gateway IP address
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 0, 1);  // optional
IPAddress secondaryDNS(8, 8, 8, 8);    // optional

// --- Mosquitto MQTT Broker ------
#define MQTT_HOST IPAddress(192, 168, 0, 2)  // local MQTT broker
//#define MQTT_HOST "broker.emqx.io"                 // web broker EMQX
// #define MQTT_HOST "broker.hivemq.com"             // web broker HiveMQ
#define MQTT_PORT 1883

//------- MQTT Topics -----
#define MQTT_PUB_TEMP1 "atasav/temp1"
#define MQTT_PUB_TEMP2 "atasav/temp2"
#define MQTT_PUB_HUM1 "atasav/hum1"
#define MQTT_PUB_HUM2 "atasav/hum2"
#define MQTT_PUB_TSR "atasav/tsr"
#define MQTT_PUB_HSR "atasav/hsr"
#define MQTT_SUB_RELS "atasav/relay"
#define MQTT_SUB_SETTEMPMIN "atasav/settempmin"
#define MQTT_SUB_SETTEMPMAX "atasav/settempmax"
#define MQTT_SUB_SETHUMMIN "atasav/sethummin"
#define MQTT_SUB_SETHUMMAX "atasav/sethummax"

char temp1Str[10];
char temp2Str[10];
char hum1Str[10];
char hum2Str[10];
char tsrStr[10];
char hsrStr[10];

int h1, h2, HUM;
float t1, t2, TEMP;
int temp_val, t_max, t_min, hum_val, h_max, h_min;
int interval_value = 5000;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// ---------------- connection to WiFi ---------------
void connectToWifi() {
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ---------------- connect to MQTT ------------------
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

// --------------- WiFi event -----------------------
void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

// ------------- On MQTT Connect ---------------
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub1 = mqttClient.subscribe(MQTT_SUB_RELS, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub1);

  uint16_t packetIdSub2 = mqttClient.subscribe(MQTT_SUB_SETTEMPMIN, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub2);

  uint16_t packetIdSub3 = mqttClient.subscribe(MQTT_SUB_SETTEMPMAX, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub3);

  uint16_t packetIdSub4 = mqttClient.subscribe(MQTT_SUB_SETHUMMIN, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub4);

  uint16_t packetIdSub5 = mqttClient.subscribe(MQTT_SUB_SETHUMMAX, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub5);
}

// ------------ On MQTT Disconnect ---------------
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

// ----------- MQTT publish -----------------
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// ------------ On MQTT Subscribe -------------
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

// ------------ On MQTT Unsubcribe ------------
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// ------------ On MQTT Message -------------
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("\n Publish received.");
  Serial.print("topic: ");
  Serial.println(topic);
  String messageTemp;
  String mytopic(topic);

  for (int i = 0; i < len; i++) {
    messageTemp += (char)payload[i];
  }
  //  Serial.print("Message: ");
  //  Serial.println(messageTemp);
  relay_control(messageTemp);   // ------ функция за ръчно управление на релетата

  // --------- set new treshold for min temp -----------
  if (mytopic == MQTT_SUB_SETTEMPMIN) {
    int temp_val = messageTemp.toInt();
    t_min = temp_val;
    Serial.print("Min temp is: ");
    Serial.println(t_min);
  }

  // --------- set new treshold for max temp -----------
  if (mytopic == MQTT_SUB_SETTEMPMAX) {
    temp_val = messageTemp.toInt();
    t_max = temp_val;
    Serial.print("Max temp is: ");
    Serial.println(t_max);
  }

  // --------- set new treshold for min humidity -----------
  if (mytopic == MQTT_SUB_SETHUMMIN) {
    hum_val = messageTemp.toInt();
    h_min = hum_val;
    Serial.print("Min humidity is: ");
    Serial.println(h_min);
  }

  // --------- set new treshold for max humidity -----------
  if (mytopic == MQTT_SUB_SETHUMMAX) {
    hum_val = messageTemp.toInt();
    h_max = hum_val;
    Serial.print("Max humidity is: ");
    Serial.println(h_max);
  }
}

// --------- Relay control -------------
void relay_control(String messageTemp) {
  if (messageTemp == "ON1") {
    digitalWrite(RELAY_PIN_1, HIGH);
    Serial.println("RELAY - 1 is now ON!");
  } else if (messageTemp == "OFF1") {
    digitalWrite(RELAY_PIN_1, LOW);
    Serial.println("RELAY - 1 is now OFF");
  }

  if (messageTemp == "ON2") {
    digitalWrite(RELAY_PIN_2, HIGH);
    Serial.println("RELAY - 2 is now ON!");
  } else if (messageTemp == "OFF2") {
    digitalWrite(RELAY_PIN_2, LOW);
    Serial.println("RELAY - 2 is now OFF");
  }

  if (messageTemp == "ON3") {
    digitalWrite(RELAY_PIN_3, HIGH);
    Serial.println("RELAY - 3 is now ON!");
  } else if (messageTemp == "OFF3") {
    digitalWrite(RELAY_PIN_3, LOW);
    Serial.println("RELAY - 3 is now OFF");
  }

  if (messageTemp == "ON4") {
    digitalWrite(RELAY_PIN_4, HIGH);
    Serial.println("RELAY - 4 is now ON!");
  } else if (messageTemp == "OFF4") {
    digitalWrite(RELAY_PIN_4, LOW);
    Serial.println("RELAY - 4 is now OFF");
  }
}
// ============= SETUP =================
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  pinMode(RELAY_PIN_4, OUTPUT);
  delay(100);
  // ---- изключване на всички релета
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
  digitalWrite(RELAY_PIN_3, LOW);
  digitalWrite(RELAY_PIN_4, LOW);
  dht14.begin();
  dht15.begin();
  Serial.println();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

//============ LOOP =====================
void loop() {
  h1 = dht14.readHumidity();     // прочитане на данни за влажността
  t1 = dht14.readTemperature();  // прочитане на данни за температурата
  h2 = dht15.readHumidity();     // прочитане на данни за влажността
  t2 = dht15.readTemperature();  // прочитане на данни за температурата
  TEMP = (t1 + t2) / 2;          // средна стойност на температурата
  HUM = (h1 + h2) / 2;           // средна стойност на влажността

  //--- проверка дали прочетените стойности са числови стойности, ако не са значи има някакъв проблем
  if (isnan(t1) || isnan(h1) || isnan(t2) || isnan(h2)) {
    Serial.println("Error read sensor!");
  } else {
    //    Serial.print("Humidity: ");
    //    Serial.print(h1);
    //    Serial.print(" % \t");
    //    Serial.print("Temperature: ");
    //    Serial.print(t1);
    //    Serial.print(" *C / ");
    //    Serial.print("Humidity: ");
    //    Serial.print(h2);
    //    Serial.print(" % \t");
    //    Serial.print("Temperature: ");
    //    Serial.print(t2);
    //    Serial.println(" *C");
    //    Serial.print("Temperature: ");
    //    Serial.print(TEMP);
    //    Serial.print(" *C\t");
    //    Serial.print("Humidity: ");
    //    Serial.print(HUM);
    //    Serial.println(" % ");
  }
  Serial.print(t_min);
  Serial.print("\t");
  Serial.print(t_max);
  Serial.print("\t");
  Serial.print(h_min);
  Serial.print("\t");
  Serial.println(h_max);

  // ----- relay programming ------
  if (HUM < h_min) {
    digitalWrite(RELAY_PIN_1, HIGH);  // --- стартира овлажнител
  } else {
    digitalWrite(RELAY_PIN_1, LOW);
  }
  if (HUM > h_max) {
    digitalWrite(RELAY_PIN_2, HIGH);  // --- стартира обезвлажнител
  } else {
    digitalWrite(RELAY_PIN_2, LOW);
  }
  if (TEMP > t_max) {
    digitalWrite(RELAY_PIN_3, HIGH);  // --- стартира компресора
  } else {
    digitalWrite(RELAY_PIN_3, LOW);
  }
  if (TEMP < t_min) {
    digitalWrite(RELAY_PIN_4, HIGH);  // --- стартира нагревател
  } else {
    digitalWrite(RELAY_PIN_4, LOW);  // ---  спира нагревател
  }
  // ----- convert data to string
  dtostrf(t1, 6, 2, temp1Str);
  dtostrf(t2, 6, 2, temp2Str);
  //  sprintf(temp2Str, " % d", t2);
  sprintf(hum1Str, " % d", h1);
  sprintf(hum2Str, " % d", h2);
  dtostrf(TEMP, 6, 2, tsrStr);
  //  sprintf(tsrStr, " % d", TEMP);
  sprintf(hsrStr, " % d", HUM);

  // ---- Publish to MQTT topics ------
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP1, 1, true, temp1Str);
  //  Serial.printf("Publishing on topic % s at QoS 1, packetId: % i", MQTT_PUB_TEMP1, packetIdPub1);
  //  Serial.println(temp1Str);
  delay(200);
  uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_TEMP2, 1, true, temp2Str);
  //  Serial.printf("Publishing on topic % s at QoS 1, packetId: % i", MQTT_PUB_TEMP2, packetIdPub2);
  //  Serial.println(temp2Str);
  delay(200);
  uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_HUM1, 1, true, hum1Str);
  //  Serial.printf("Publishing on topic % s at QoS 1, packetId: % i", MQTT_PUB_HUM1, packetIdPub3);
  //  Serial.println(hum1Str);
  delay(200);
  uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_HUM2, 1, true, hum2Str);
  //  Serial.printf("Publishing on topic % s at QoS 1, packetId: % i", MQTT_PUB_HUM2, packetIdPub4);
  //  Serial.println(hum2Str);
  delay(200);
  uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_TSR, 1, true, tsrStr);
  //  Serial.printf("Publishing on topic % s at QoS 1, packetId: % i", MQTT_PUB_TSR, packetIdPub5);
  //  Serial.println(tsrStr);
  delay(200);
  uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_HSR, 1, true, hsrStr);
  //  Serial.printf("Publishing on topic % s at QoS 1, packetId: % i", MQTT_PUB_HSR, packetIdPub6);
  //  Serial.println(hsrStr);
  delay(interval_value);
}
