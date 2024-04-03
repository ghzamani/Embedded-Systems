#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#include "WiFi.h"
#include "AsyncUDP.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "time.h"

//////WIFI settings
//#define WLAN_SSID "Galaxy A51E4D2"
//#define WLAN_PASS "kmhn4235"
//#define WLAN_SSID "Parisa's Galaxy S21 FE 5G"
//#define WLAN_PASS "hjni2911"
#define WLAN_SSID "Alaie"
#define WLAN_PASS "B@118m@48Al767883"

#define AIO_SERVER      "45.149.77.235"
#define AIO_SERVERPORT  1883           
#define AIO_USERNAME    "97522175"
#define AIO_KEY         "0VSscu3h"

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish car_state = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/car_state");
Adafruit_MQTT_Publish alarm_state = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/alarm_state");
Adafruit_MQTT_Publish buzzer_state = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/buzzer_state");
// get on/off from server
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/ONOFF_FEED");

//// wire connected to the car
#define car_wire 32
int car_wire_value;

// Define NTP Client to get time
const char* ntpServer = "asia.pool.ntp.org";
const long  gmtOffset_sec = 12600;
const int   daylightOffset_sec = 3600;

//gyroscope sensor
Adafruit_MPU6050 mpu;
//get acceleration
double acceleration;
int buzzer = 23;
double buzzer_threshold = 10.5;

// relay
const int relay = 26;

bool send_sms_alarm = true;
bool send_sms_buzzer = true;

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


void check_car_signal(){
//  Serial2.println("AT+CCLK?");
//  updateSerial();

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    if(car_wire_value == 0){
      send_sms_alarm = false;
      car_state.publish("Unknown Time, Car is OFF");
      alarm_state.publish("Unknown Time, Alarm is ON");
    }
    else{
      send_sms_alarm = true;
      car_state.publish("Unknown Time, Car is ON");
      alarm_state.publish("Unknown Time, Alarm is OFF");
    }
    return;
  }
  
  char timeWeekDay[10];
  char month[10];
  char day[3];
  char year[5];
  char hour[3];
  char minute[3];
  char second[3];
  
  strftime(timeWeekDay,10, "%A", &timeinfo);
  strftime(month,10, "%B", &timeinfo);
  strftime(day,3, "%d", &timeinfo);
  strftime(year,5, "%Y", &timeinfo);
  strftime(hour,3, "%H", &timeinfo);
  strftime(minute,3, "%M", &timeinfo);
  strftime(second,3, "%S", &timeinfo);

  char str[80];
  strcpy(str, timeWeekDay);
  strcat(str, ", ");
  strcat(str, day);
  strcat(str, " ");
  strcat(str, month);
  strcat(str, " ");
  strcat(str, year);
  strcat(str, ", ");
  strcat(str, hour);
  strcat(str, ":");
  strcat(str, minute);
  strcat(str, ":");
  strcat(str, second);
  Serial.println(str);
  Serial.println("****************************");

  char str2[80];
  strcpy(str2, str);
  
  if(car_wire_value == 0){
    send_sms_alarm = false;
    car_state.publish(strcat(str, " Car is OFF"));
    alarm_state.publish(strcat(str2, " Alarm is ON"));
  }
  else{
    send_sms_alarm = true;
    car_state.publish(strcat(str, " Car is ON"));
    alarm_state.publish(strcat(str2, " Alarm is OFF"));
  }
}

void setup_mpu(int a, int g, int bw){
  // Try to initialize MPU6050!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //SET SENSITIVITY OF MPU
  //A smaller range determines a better sensitive outcome of reading which are obtained
  switch(a){
    case 2:
      mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
      break;
    case 4:
      mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
      break;
    case 8:
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      break;
    case 16:
      mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
      break;
  }  
  Serial.print("Accelerometer range set to: ");
  Serial.println(a);

  switch(g){
    case 250:
      mpu.setGyroRange(MPU6050_RANGE_250_DEG);
      break;
    case 500:
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      break;
    case 1000:
      mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
      break;
    case 2000:
      mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
      break;
  }  
  Serial.print("Gyro range set to: ");
  Serial.println(g);

  switch(bw){
    case 260:
      mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
      break;
    case 184:
      mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
      break;
    case 94:
      mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
      break;
    case 44:
      mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
      break;
    case 21:
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      break;
    case 10:
      mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
      break;
    case 5:
      mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
      break;      
  }
  Serial.print("Filter bandwidth set to: ");
  Serial.println(bw);
}

double get_mpu(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  return sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));  
}

void buzzer_sound(double threshold){
  if(acceleration > threshold){
    send_sms_buzzer = false;
    buzzer_state.publish("Buzzer is beeping!");
    for (int i = 0; i < 80; i++) {  // make a sound
      digitalWrite(buzzer, HIGH); // send high signal to buzzer 
      delay(1); // delay 1ms
      digitalWrite(buzzer, LOW); // send low signal to buzzer
      delay(1);
    }
  }
  else {
    buzzer_state.publish("Buzzer is OFF");
    send_sms_buzzer = true;
    }
}

//void relay_switch(){
//  // Normally Open configuration, send LOW signal to let current flow
//  // (if you're usong Normally Closed configuration send HIGH signal)
//  digitalWrite(relay, LOW);
//  Serial.println("Current Flowing");
//  delay(5000); 
//  
//  // Normally Open configuration, send HIGH signal stop current flow
//  // (if you're usong Normally Closed configuration send LOW signal)
//  digitalWrite(relay, HIGH);
//  Serial.println("Current not Flowing");
//  delay(5000);
//}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}

void send_SMS(String msg)
{
  Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  Serial2.println("AT+CMGS=\"+989305863861\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  Serial2.print(msg); //text content
  updateSerial();
  Serial.println();
  Serial.println("Message Sent");
  Serial2.write(26);
}

int receiveCommand() {
  int clientSt = -1;
  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char*)onoffbutton.lastread);

      char relay_val[7];
      strcpy(relay_val, (char*)onoffbutton.lastread);
      Serial.print("receiveCommand: "); 
      Serial.println(relay_val[6]);
      return (int)relay_val[6];
    }
  }
  return clientSt;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  Serial2.begin(9600);
  delay(3000);
//  Serial.println("Initializing modem...");

  ///WIFI setup
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
//    Serial.print(".");
  }

  
  pinMode(car_wire, INPUT);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW); /// the first time led must be off
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Setup sensor MPU6050
  setup_mpu(2, 500, 5);
  pinMode(buzzer, OUTPUT); // set buzzer pin as output

  mqtt.subscribe(&onoffbutton);
}

void loop() {
  // put your main code here, to run repeatedly:
    
  //// connect to server
  MQTT_connect();

  car_wire_value = analogRead(car_wire);
  Serial.print("car voltage: ");
  Serial.println(car_wire_value);

  //get relay_value from server
  int val = receiveCommand();
  if(val != -1){
    Serial.print("server relay value");
    Serial.println(val);
    if(val-48 == 0){
      digitalWrite(relay, LOW);
    }
    if(val-48 == 1){
      digitalWrite(relay, HIGH);
    }
  }
  
  // send sms to user showing alarm is working
  if(send_sms_alarm && car_wire_value == 0){
//    send_SMS("Car Alarm is working");
    Serial.println("SMS:Car Alarm is working");
  }
  
  //// send signal, indicating whether the car is on/off 
  //// and send the time to server
  check_car_signal();

  // gyroscope value
  if(car_wire_value == 0){
    acceleration = get_mpu();
    Serial.print("Acceleration: ");
    Serial.println(acceleration);

    // send sms to user showing buzzer is beeping
    if(send_sms_buzzer && acceleration > buzzer_threshold){
//      send_SMS("Buzzer is beeping");
        Serial.println("SMS:Buzzer is beeping");
    }
    
    //set threshold for making sound
    buzzer_sound(buzzer_threshold);  
  }
  else buzzer_state.publish("Buzzer is OFF");

//  relay_switch();
//  updateSerial();
  
  if (!mqtt.ping()) {
    mqtt.disconnect();
  }

  delay(2000);
  
}
