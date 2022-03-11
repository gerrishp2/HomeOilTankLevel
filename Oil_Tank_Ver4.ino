// Oil Tank Level Monitor 1/22/2022 Phil Gerrish

#include <WiFi.h>
#include <AsyncMqttClient.h>

// Motor A
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 
const int SwUp = 15;
const int SwDown = 19;
const int SwLevel = 21;

#define ONBOARD_LED  2
#define WIFI_SSID "Your-SSD"
#define WIFI_PASSWORD "Your-Password"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(XXX, XX, XX, XX)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// MQTT Topics
#define MQTT_PUB_OIL "esp32/OilTank"

volatile int count = 0;
#define PULSE_PIN  34
#define OSCPIN 17

portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings


void IRAM_ATTR isr() {
  portENTER_CRITICAL(&synch);
  digitalWrite(OSCPIN, 1);
  digitalWrite(OSCPIN, 0);
  count++;
  portEXIT_CRITICAL(&synch);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;}}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);}}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);}


int progMode = 1;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(SwUp, INPUT);
  pinMode(SwDown, INPUT);
  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(PULSE_PIN, isr, RISING);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  Serial.begin(115200);

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("mqttuser", "mqttpassword");
  connectToWifi();
}

void loop() {

int SwUp_state = digitalRead(SwUp);
int SwDown_state = digitalRead(SwDown);
unsigned long currentMillis = millis();

//Subtract counts from Home to Top Level
// Devide by Level Range total
float countLevel = ((count - 16646)/71283.0)*100.0;

countLevel = (100.0 - countLevel);

int SwLevel_state = digitalRead(SwLevel);

   //if ( SwDown_state == LOW ) {Serial.println(count);}

  // Home Mode Turn on Motor
  if ( progMode == 1 and SwUp_state == LOW){
    ledcWrite(pwmChannel, dutyCycle);
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
  }

  // Home Mode Sw Found Turn Motor Off
  if ( progMode == 1 and SwUp_state == HIGH){
    ledcWrite(pwmChannel, dutyCycle);
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      count = 0;
      progMode = 2;}

        // Home Mode Sw Found Turn Motor Off
  if ( progMode == 2 and SwDown_state == LOW){
        if( SwLevel_state == LOW){
                digitalWrite(motor1Pin1, HIGH);
                digitalWrite(motor1Pin2, LOW);}
          else{
               digitalWrite(motor1Pin1, LOW);
                digitalWrite(motor1Pin2, LOW);}
         }

    // Home Mode Sw Found Turn Motor Off
  if (SwLevel_state == HIGH){
    digitalWrite(ONBOARD_LED,HIGH);}
  else
  {digitalWrite(ONBOARD_LED,LOW);}

      //Level Missed Bottom Sw Found Turn Motor OFF
  if (SwDown_state == HIGH and progMode == 2){
                digitalWrite(motor1Pin1, LOW);
                digitalWrite(motor1Pin2, LOW);}

    if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    
    // Publish an MQTT message
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_OIL, 1, true, String(countLevel).c_str());   
    Serial.println(countLevel);                        
  }
  
  dutyCycle = 250;
}
