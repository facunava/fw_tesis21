//CambiÃƒÂ© los pines de las luces amarilla y verde (que estaban invertidos)
//AgreguÃƒÂ© "titulos" a cada bloque del cÃƒÂ³digo
//AgreguÃƒÂ© la segunda salida de I2C, lo que implicÃƒÂ³ modificaciones en la
//secciÃƒÂ³n de "GLOBAL CONSTANTS DEFINITION", y en la secciÃƒÂ³n de "MAIN SETUP".
//Me basÃƒÂ© en el siguiente post: https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
//Lo ÃƒÂºnico que restarÃƒÂ­a es asignar correctamente el pinout del segundo bus I2C
//para que coincida con lo que tenemos en el PCB y definir que la lectura de sensores
//se haga por el otro bus I2C.



#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SHT31.h"
#include "PCF8574.h"
#include <ArduinoJson.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <FastLED.h>

#ifdef DEBUG
   #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT macro, print
   #define DPRINTLN(...)  Serial.println(__VA_ARGS__)   //DPRINTLN, macro println
#else
   #define DPRINT(...)
   #define DPRINTLN(...)   
#endif

//===========================================================
//=============GLOBAL CONSTANTS DEFINITION===================
//===========================================================

const int buzzer  = 23;     //Buzzer Pin
const int red     = 16;     //RED Pin
const int yellow  = 17;     // YELLOW Pin
const int green   = 5;      // GREEN Pin

float temp1 = 0;
float hum1 = 0;
float temp2 = 0;
float hum2 = 0;

#define LED_PIN     13
#define NUM_LEDS    2
#define BRIGHTNESS  64
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define UPDATES_PER_SECOND 100

#define SHT31_ADDRESS     0x44
#define PCA9548A_ADDRESS  0x70
#define PCF8574_ADDRESS   0x20
#define SECONDS_BETWEEN_MEASUREMENTS 15 //Establece cada cuanto se hacen las mediciones
#define temperature_topic_01 "sensor/01/temperature"
#define humidity_topic_01    "sensor/01/humidity"
#define temperature_topic_02 "sensor/02/temperature"
#define humidity_topic_02    "sensor/02/humidity"

// SmartSaffron_inputs
#define I2C_SCL1 18
#define I2C_SDA1 19

// SmartSaffron_outputs
#define I2C_SDA2 25
#define I2C_SCL2 26


float temperature_threshold = 0;

//parametros de la red local y mqtt server
const char* ssid = "RPi_HomeAssistant";
const char* password = "SmartSaffron";
const char* mqtt_server = "10.0.0.10";
const uint16_t mqtt_port = 1883;
const char* CLIENT_NAME = "SmartSaffron_esp32";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

QWIICMUX myMux;

SHT31 sht1, sht2;

PCF8574 PCF_01(0x20, &Wire1);

//===========================================================
//===================WIFI CONNECTION=========================
//===========================================================
void setup_Wifi(){
  DPRINTLN("\t\t ### Configuracion WiFI ###");
  DPRINT("Conectando WiFi a " + String(ssid) + " ");

  WiFi.begin(ssid, password);
  
  while(WiFi.status() != WL_CONNECTED) { delay(500); DPRINT("."); }
  DPRINTLN("");
  DPRINTLN("Wifi conectado a " + String(ssid));
  DPRINT("Direccion IP : ");
  DPRINTLN(WiFi.localIP());
}


//===========================================================
//================MQTT OUTPUTS MESSAGES======================
//===========================================================
void callback(char* topic, byte* message, unsigned int length) {
  DPRINT("Mensaje recibido desde topico : ");
  DPRINT(topic);
  DPRINT(". Mensaje: ");

  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    DPRINT((char)message[i]);
    messageTemp += (char)message[i];
  }
  DPRINTLN("");
  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  // Buzzer control
  // char command[] = String(topic);

  // switch (command)
  // {
  //   case 'CMD/buzzer0':
  //     if(messageTemp == "ON")
  //       digitalWrite(buzzer, HIGH);
  //     else if(messageTemp == "OFF")
  //       digitalWrite(buzzer, LOW);

  //     break;
  // }
  if (String(topic) == "CMD/state"){
    temperature_threshold = messageTemp.toFloat();
    DPRINTLN("Valor umbral inicial: " + String(temperature_threshold));
  }

  if (String(topic) == "CMD/temperature_threshold"){
    temperature_threshold = messageTemp.toFloat();
    DPRINTLN("Valor umbral actual: " + String(temperature_threshold));
  }

  if (String(topic) == "CMD/buzzer0") 
  {
    //===========BUZZER CONTROL===========
    if(messageTemp == "ON"){
      digitalWrite(buzzer, HIGH);
      /*PCF_01.write(0, HIGH);
      PCF_01.write(1, HIGH);
      PCF_01.write(2, HIGH);
      PCF_01.write(3, HIGH);
      PCF_01.write(4, HIGH);
      PCF_01.write(5, HIGH);
      PCF_01.write(6, HIGH);
      PCF_01.write(7, HIGH);      
      digitalWrite(red, HIGH);
      digitalWrite(yellow, HIGH);
      digitalWrite(green, HIGH);*/

    }
    else if(messageTemp == "OFF"){
      digitalWrite(buzzer, LOW);
      /*PCF_01.write(0, LOW);
      PCF_01.write(1, LOW);
      PCF_01.write(2, LOW);
      PCF_01.write(3, LOW);
      PCF_01.write(4, LOW);
      PCF_01.write(5, LOW);
      PCF_01.write(6, LOW);
      PCF_01.write(7, LOW);
      digitalWrite(red, LOW);
      digitalWrite(yellow, LOW);
      digitalWrite(green, LOW);*/
    }
  }
  //===========EV CONTROL===========
  else if (String(topic) == "CMD/ev1") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(0, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(0, LOW);
    }
  }

  else if (String(topic) == "CMD/ev2") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(1, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(1, LOW);
    }
  }

  else if (String(topic) == "CMD/ev3") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(2, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(2, LOW);
    }
  }

  else if (String(topic) == "CMD/ev4") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(3, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(3, LOW);
    }
  }

  else if (String(topic) == "CMD/ev5") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(4, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(4, LOW);
    }
  }

  else if (String(topic) == "CMD/ev6") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(5, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(5, LOW);
    }
  }

  else if (String(topic) == "CMD/ev7") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(6, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(6, LOW);
    }
  }

  else if (String(topic) == "CMD/ev8") 
  {
    if(messageTemp == "ON"){
      PCF_01.write(7, HIGH);
    }
    else if(messageTemp == "OFF"){
      PCF_01.write(7, LOW);
    }
  }

  //===========Red light control===========
  else if (String(topic) == "CMD/red") 
  {
    if(messageTemp == "ON"){
      digitalWrite(red, HIGH);
    }
    else if(messageTemp == "OFF"){
      digitalWrite(red, LOW);
    }
  }

  //===========Yellow light control===========
  else if (String(topic) == "CMD/yellow") 
  {
    if(messageTemp == "ON"){
      digitalWrite(yellow, HIGH);
    }
    else if(messageTemp == "OFF"){
      digitalWrite(yellow, LOW);
    }
  }

  //===========Green light control===========
  else if (String(topic) == "CMD/green") 
  {
    if(messageTemp == "ON"){
      digitalWrite(green, HIGH);
    }
    else if(messageTemp == "OFF"){
      digitalWrite(green, LOW);
    }
  }
}


//===========================================================
//================MQTT SERVER CONNECTION=====================
//===========================================================
//conexion a MQTT server
void setup_mqttBroker() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setKeepAlive(3600); // se deconecta en 1 hs, 
  mqttClient.setCallback(callback);

  DPRINTLN("\t\t ### Configuracion MQTT ###");
  DPRINT("Conectando a broker en " + String(mqtt_server) + " ");

  while(!mqttClient.connect(CLIENT_NAME)) { delay(500); DPRINT("."); }
  DPRINTLN("");
  DPRINTLN("Conectado a broker en " + String(mqtt_server));

  mqttClient.subscribe("CMD/#");

}


//===========================================================
//======================MUX CONFIG===========================
//===========================================================

void PCA9548A(uint8_t bus)
{
  Wire.beginTransmission(PCA9548A_ADDRESS);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

//===========================================================
//======================I2C CONFIG===========================
//===========================================================
void setup_i2c() {
  //DUAL I2C INTIALIZATION
  // I2CBME1.begin(I2C_SDA1, I2C_SCL1, 100000);
  // I2CBME2.begin(I2C_SDA2, I2C_SCL2, 100000);
  Wire.begin(I2C_SDA1, I2C_SCL1, 100000);
  Wire1.begin(I2C_SDA2, I2C_SCL2, 100000);


  if (myMux.begin(PCA9548A_ADDRESS, Wire) == false)
  {
    DPRINTLN("Multiplexor no detectado ... loop infinito.");
    while (1);
  }
  DPRINTLN("Multiplexor detectado");

  PCA9548A(4);
  sht1.begin(SHT31_ADDRESS, &Wire);

  PCA9548A(5);
  sht2.begin(SHT31_ADDRESS, &Wire);

  bool status1;
  bool status2;

  // PCA9548A(4);
  // status1 = sht1.begin(SHT31_ADDRESS, &Wire);  
  // //Mensaje de alerta si no se detectan sensores conectados, feel free to comment them out 
  // if (!status1) {
  //   Serial.println("Could not find a valid SHR31 sensor (INPUT1), check wiring!");
  //   //while (1);
  // }

  // status2 = sht2.begin(SHT31_ADDRESS, &Wire);  
  // //Mensaje de alerta si no se detectan sensores conectados, feel free to comment them out 
  // if (!status2) {
  //   Serial.println("Could not find a valid SHR31 sensor (INPUT2), check wiring!");
  //   //while (1);
  // }

  PCF_01.begin(PCF8574_ADDRESS);
  PCF_01.write(0, LOW);
  PCF_01.write(1, LOW);
  PCF_01.write(2, LOW);
  PCF_01.write(3, LOW);
  PCF_01.write(4, LOW);
  PCF_01.write(5, LOW);
  PCF_01.write(6, LOW);
  PCF_01.write(7, LOW);

}


//===========================================================
//======================MAIN SETUP===========================
//===========================================================

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

void ChangePalettePeriodically();
void FillLEDsFromPaletteColors( uint8_t colorIndex);
void SetupPurpleAndGreenPalette();
void SetupBlackAndWhiteStripedPalette();
void SetupTotallyRandomPalette();

void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(green, OUTPUT);

  Serial.begin(9600);
  setup_Wifi();
  setup_mqttBroker();
  setup_i2c();


  // bool status1;
  // bool status2;


  // status1 = sht1.begin(0x76, &I2CBME1);  
  // //Mensaje de alerta si no se detectan sensores conectados, feel free to comment them out 
  // if (!status1) {
  //   Serial.println("Could not find a valid SHR31 sensor (INPUT1), check wiring!");
  //   while (1);
  // }

  // status2 = sht2.begin(0x76, &I2CBME2); 
  // //Mensaje de alerta si no se detectan sensores conectados, feel free to comment them out 
  // if (!status2) {
  //   Serial.println("Could not find a valid SHR31 sensor (INPUT2), check wiring!");
  //   while (1);
  // }

  delay( 2000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
  
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
}


//===========================================================
//=====================DATA PUBLISH==========================
//===========================================================
//const char* MQTT_SENSOR_TOPIC = "test";

void publishData(float p_temperature, float p_humidity, char* mqtt_topic) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = p_temperature;
  root["humidity"] = p_humidity;
  root.prettyPrintTo(Serial);
  Serial.println("");
  /*
     {
        "temperature": 23.20 ,
        "humidity": 43.70
     }
  */
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  mqttClient.publish(mqtt_topic, data, true);
  yield();
}

unsigned long lastTime = 0;
bool firstTime = true;
char msg[50]="";


void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}

//===========================================================
//=======================MAIN LOOP===========================
//===========================================================

void loop() 
{ 

  while(temperature_threshold == 0)
  {
    currentPalette = RainbowColors_p;         
    currentBlending = LINEARBLEND;

    static uint8_t startIndex = 0;
    startIndex = startIndex + 1; /* motion speed */
    
    FillLEDsFromPaletteColors( startIndex);
    
    FastLED.show();
    FastLED.delay(1000 / UPDATES_PER_SECOND);

    mqttClient.loop();
  }

  // Alerta Verde
  if (temperature_threshold > 0) {
    digitalWrite(green, HIGH);
  } 

  else if (temperature_threshold == 0)
  {
    digitalWrite(green, LOW);
  }

    // Alerta Amarilla
  if ((temp1 >= (temperature_threshold-1) && temp1 < (temperature_threshold)) || (temp2 >= (temperature_threshold-1) && temp2 < (temperature_threshold)) ) {
    digitalWrite(yellow, HIGH);
  } 
  else
  {
    digitalWrite(yellow, LOW);
  }


  // Alerta Roja
  if ((temp1 >= temperature_threshold) || (temp2 >= temperature_threshold)) {
    digitalWrite(red, HIGH);
    digitalWrite(buzzer, HIGH);
  } 
  else
  {
    digitalWrite(red, LOW);
    digitalWrite(buzzer, LOW);  
  }

  if ( firstTime || (millis() - lastTime > SECONDS_BETWEEN_MEASUREMENTS*1000) ) 
  {
    firstTime = false;
    lastTime = millis();
  
    if (!mqttClient.connected()) 
    {
      setup_mqttBroker();
    }

    //mqttClient.loop();

    PCA9548A(4);
    sht1.read();

    temp1 = sht1.getTemperature();
    hum1  = sht1.getHumidity();

    DPRINT("Temperatura1: " + String(temp1) + "\n");
    DPRINT("Humedad1: " + String(hum1) + "\n");
    publishData(temp1, hum1, "sensor1");

    PCA9548A(5);
    sht2.read();

    temp2 = sht2.getTemperature();
    hum2  = sht2.getHumidity();

    DPRINT("Temperatura2: " + String(temp2) + "\n");
    DPRINT("Humedad2: " + String(hum2) + "\n");
    publishData(temp2, hum2, "sensor2");


    /*Lights control
    if temp1 >= 23 {
    digitalWrite(red, HIGH);
    digitalWrite(yellow, LOW);
    }
    if temp1 < 23 && temp1 > 23 - 1.5 {
    digitalWrite(red, LOW);
    digitalWrite(yellow, HIGH);
    }
    digitalWrite(green, HIGH);*/
  }

  mqttClient.loop();

}
