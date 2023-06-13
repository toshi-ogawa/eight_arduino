#include "AlcoholSensor.h"
#include <ArduinoMqttClient.h>
#include "WiFiNINA_Generic.h"
#include "arduino_secrets.h"
#include <ArduinoJson.h>


///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

//const char broker[] = "192.168.1.88"; //Cable LAN
//const char broker[] = "192.168.0.136";  //Potato6,Tomato7
//const char broker[] = "192.168.2.131"; //Ex406Aroom 5G
//const char broker[] = "192.168.43.10"; //mobile
const char broker[] = "192.168.43.113"; //mobile
int        port     = 1883;
const char topic[]  = "cptdata";

//set interval for sending messages (milliseconds)
const long interval = 200;
const long interval_2 = 200;
unsigned long previousMillis= 0;

int count = 0;

//---------------------------------
//関数
//---------------------------------
void updateData();
void checkStimu();


//---------------------------------
//ピン定義
//---------------------------------
#define LED 13
//---------------------------------
//定数パラメータ
//---------------------------------
//制御周期
#define WAIT 0  //待機状態
#define ESTIMATE 2 //検知周期を計算する

//センサ
#define THRESHOLD 0.004

//---------------------------------
//ARXモデル
//---------------------------------
AlcoholSensor sensor_1 = AlcoholSensor();
AlcoholSensor sensor_2 = AlcoholSensor();
AlcoholSensor sensor_3 = AlcoholSensor();
AlcoholSensor sensor_4 = AlcoholSensor();
AlcoholSensor sensor_5 = AlcoholSensor();
AlcoholSensor sensor_6 = AlcoholSensor();
AlcoholSensor sensor_7 = AlcoholSensor();
AlcoholSensor sensor_8 = AlcoholSensor();


//---------------------------------
//ADコンバータ
//---------------------------------
#define SELPIN 10 //Selection Pin 
#define DATAOUT 11//MOSI 
#define DATAIN  12//MISO 
#define SPICLOCK  13//Clock 
int readvalue; 


//---------------------------------
//変数
//---------------------------------
//ロボット内部状態
int robotState = WAIT;
int stimu_1 = 0, stimu_2 = 0, stimu_3 = 0, stimu_4 = 0,stimu_5 = 0,stimu_6 = 0,stimu_7 = 0,stimu_8 = 0;
int Gsens_1 =0, Gsens_2=0, Gsens_3=0, Gsens_4=0, Gsens_5=0, Gsens_6=0, Gsens_7=0, Gsens_8=0;

int id=-1;
int i = 0;

void SendData();



void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    Serial.println(ssid);
    Serial.println(pass);
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

//    while (1);
delay(1000);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();


//Alcohol sensor
  pinMode(LED, OUTPUT);

//ARXモデルの係数入力  

  sensor_1.arx.set_constant(0.0035,0.0008,-0.9015,-0.0833);
  sensor_2.arx.set_constant(0.0005,0.0074,-0.9014,-0.083);
  sensor_3.arx.set_constant(0.0054,0.0010,-0.9014,-0.0834);
  sensor_4.arx.set_constant(0.0027,0.0078,-0.9014,-0.0831);
  sensor_5.arx.set_constant(0.005,0.0024,-0.9015,-0.0831);
  sensor_6.arx.set_constant(-0.0024,0.0066,-0.9016,-0.0833);
  sensor_7.arx.set_constant(0.0026,0.007,-0.9015,-0.0832);
  sensor_8.arx.set_constant(0.0046,-0.0003,-0.9015,-0.0834);
  sensorCalibration();

  sensor_1.setThreshold(THRESHOLD);
  sensor_2.setThreshold(THRESHOLD);
  sensor_3.setThreshold(THRESHOLD);
  sensor_4.setThreshold(THRESHOLD);
  sensor_5.setThreshold(THRESHOLD);
  sensor_6.setThreshold(THRESHOLD);
  sensor_7.setThreshold(THRESHOLD);
  sensor_8.setThreshold(THRESHOLD);

  //ADコンバータ
   //set pin modes 
 pinMode(SELPIN, OUTPUT); 
 pinMode(DATAOUT, OUTPUT); 
 pinMode(DATAIN, INPUT); 
 pinMode(SPICLOCK, OUTPUT); 
 //disable device to start with 
 digitalWrite(SELPIN,HIGH); 
 digitalWrite(DATAOUT,LOW); 
 digitalWrite(SPICLOCK,LOW); 

}

//ADコンバータ関数設定
int read_adc(int channel){
  int adcvalue = 0;
  byte commandbits = 0b11000000; //command bits - start, mode, chn (3), dont care (3)

  //allow channel selection
  commandbits|=((channel-1)<<3);//３ビットだけ左にずらす

  digitalWrite(SELPIN,LOW); //Select adc
  // setup bits to be written
  for (int i=7; i>=3; i--){
    digitalWrite(DATAOUT,commandbits&1<<i);
    //cycle clock
    digitalWrite(SPICLOCK,HIGH);
    digitalWrite(SPICLOCK,LOW);    
  }

  digitalWrite(SPICLOCK,HIGH);    //ignores 2 null bits
  digitalWrite(SPICLOCK,LOW);
  digitalWrite(SPICLOCK,HIGH);  
  digitalWrite(SPICLOCK,LOW);

  //read bits from adc
  for (int i=11; i>=0; i--){
    adcvalue+=digitalRead(DATAIN)<<i;
    //cycle clock
    digitalWrite(SPICLOCK,HIGH);
    digitalWrite(SPICLOCK,LOW);
  }
  digitalWrite(SELPIN, HIGH); //turn off device
  return adcvalue;
}


void loop() {
  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();

    //シリアルデータの受信と処理
  int inByte = 0;

  robotState = ESTIMATE;
 digitalWrite(LED, LOW);
  

  Gsens_1 = read_adc(1); 
  Gsens_2 = read_adc(2); 
  Gsens_3 = read_adc(3); 
  Gsens_4 = read_adc(4); 
  Gsens_5 = read_adc(5); 
  Gsens_6 = read_adc(6); 
  Gsens_7 = read_adc(7); 
  Gsens_8 = read_adc(8); 
  
 updateData();  
 
 if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;
   
    if (robotState == ESTIMATE) { 
      id = id +1;
      SendData();
       digitalWrite(LED, HIGH);
      }else{
        id  =0 ;
      }
    
  }

}


void SendData(){

    
    DynamicJsonDocument doc(2048);

    doc["id"] = id;
    JsonObject sensor_array_data_0 = doc["data"].createNestedObject();

    // float Data1 = sensor_1.getARXData();
    // char formattedData1[8];
    // dtostrf(Data1, 6, 3, formattedData1);
    // sensor_array_data_0["s1"] = atof(formattedData1);

    // float Data2 = sensor_2.getARXData();
    // char formattedData2[8];
    // dtostrf(Data2, 6, 3, formattedData2);
    // sensor_array_data_0["s2"] = atof(formattedData2);

    // float Data3 = sensor_3.getARXData();
    // char formattedData3[8];
    // dtostrf(Data3, 6, 3, formattedData3);
    // sensor_array_data_0["s3"] = atof(formattedData3);

    // float Data4 = sensor_4.getARXData();
    // char formattedData4[8];
    // dtostrf(Data4, 6, 3, formattedData4);
    // sensor_array_data_0["s4"] = atof(formattedData4);

    // float Data5 = sensor_5.getARXData();
    // char formattedData5[8];
    // dtostrf(Data5, 6, 3, formattedData5);
    // sensor_array_data_0["s5"] = atof(formattedData5);

    // float Data6 = sensor_6.getARXData();
    // char formattedData6[8];
    // dtostrf(Data6, 6, 3, formattedData6);
    // sensor_array_data_0["s6"] = atof(formattedData6);

    // float Data7 = sensor_7.getARXData();
    // char formattedData7[8];
    // dtostrf(Data7, 6, 3, formattedData7);
    // sensor_array_data_0["s7"] = atof(formattedData7);

    // float Data8 = sensor_8.getARXData();
    // char formattedData8[8];
    // dtostrf(Data8, 6, 3, formattedData8);
    // sensor_array_data_0["s8"] = atof(formattedData8);

    float Data1 = sensor_1.getRawData();
    char formattedData1[8];
    dtostrf(Data1, 6, 3, formattedData1);
    sensor_array_data_0["s1"] = atof(formattedData1);

    float Data2 = sensor_2.getRawData();
    char formattedData2[8];
    dtostrf(Data2, 6, 3, formattedData2);
    sensor_array_data_0["s2"] = atof(formattedData2);

    float Data3 = sensor_3.getRawData();
    char formattedData3[8];
    dtostrf(Data3, 6, 3, formattedData3);
    sensor_array_data_0["s3"] = atof(formattedData3);

    float Data4 = sensor_4.getRawData();
    char formattedData4[8];
    dtostrf(Data4, 6, 3, formattedData4);
    sensor_array_data_0["s4"] = atof(formattedData4);

    float Data5 = sensor_5.getRawData();
    char formattedData5[8];
    dtostrf(Data5, 6, 3, formattedData5);
    sensor_array_data_0["s5"] = atof(formattedData5);

    float Data6 = sensor_6.getRawData();
    char formattedData6[8];
    dtostrf(Data6, 6, 3, formattedData6);
    sensor_array_data_0["s6"] = atof(formattedData6);

    float Data7 = sensor_7.getRawData();
    char formattedData7[8];
    dtostrf(Data7, 6, 3, formattedData7);
    sensor_array_data_0["s7"] = atof(formattedData7);

    float Data8 = sensor_8.getRawData();
    char formattedData8[8];
    dtostrf(Data8, 6, 3, formattedData8);
    sensor_array_data_0["s8"] = atof(formattedData8);


    String output;
    serializeJson(doc, output);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print(output);
    mqttClient.endMessage();

}

//------------------------------------------------------------
//関数
//------------------------------------------------------------

void updateData() {
  int i;

  sensor_1.addData( 5.0 * Gsens_1 / 1023);
  sensor_2.addData( 5.0 * Gsens_2 / 1023);
  sensor_3.addData( 5.0 * Gsens_3 / 1023);
  sensor_4.addData( 5.0 * Gsens_4 / 1023);
  sensor_5.addData( 5.0 * Gsens_5 / 1023);
  sensor_6.addData( 5.0 * Gsens_6 / 1023);
  sensor_7.addData( 5.0 * Gsens_7 / 1023);
  sensor_8.addData( 5.0 * Gsens_8 / 1023);
}



void sensorCalibration() {
  int i = 0;
  double sum_1 = 0, sum_2 = 0, sum_3 = 0 ,sum_4 = 0,sum_5 = 0, sum_6 = 0, sum_7 = 0 ,sum_8 = 0;
  for (i = 0; i < 100; i++) {

    sum_1 += read_adc(1) * 5.0 / 1023;
    sum_2 += read_adc(2) * 5.0 / 1023;
    sum_3 += read_adc(3) * 5.0 / 1023;
    sum_4 += read_adc(4) * 5.0 / 1023;
    sum_5 += read_adc(5) * 5.0 / 1023;
    sum_6 += read_adc(6) * 5.0 / 1023;
    sum_7 += read_adc(7) * 5.0 / 1023;
    sum_8 += read_adc(8) * 5.0 / 1023;
    delay(10);
  }

  
  sensor_1.setReferenceValue(sum_1 / 100);
  sensor_2.setReferenceValue(sum_2 / 100);
  sensor_3.setReferenceValue(sum_3 / 100);
  sensor_4.setReferenceValue(sum_4 / 100);
  sensor_5.setReferenceValue(sum_5 / 100);
  sensor_6.setReferenceValue(sum_6 / 100);
  sensor_7.setReferenceValue(sum_7 / 100);
  sensor_8.setReferenceValue(sum_8 / 100);
}

void checkStimu() {

  stimu_1 = 0;
  stimu_2 = 0;
  stimu_3 = 0;
  stimu_4 = 0;
  stimu_5 = 0;
  stimu_6 = 0;
  stimu_7 = 0;
  stimu_8 = 0;

   if (sensor_1.detected()) {
    stimu_1 = 1;
  }
  if (sensor_2.detected()) {
    stimu_2 = 1;
  }
  if (sensor_3.detected()) {
    stimu_3 = 1;
  }
  if (sensor_4.detected()) {
    stimu_4 = 1;
  }
  if (sensor_5.detected()) {
    stimu_5 = 1;
  }
  if (sensor_6.detected()) {
    stimu_6 = 1;
  }
  if (sensor_7.detected()) {
    stimu_7 = 1;
  }
  if (sensor_8.detected()) {
    stimu_8 = 1;
  }
}
