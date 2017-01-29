/*
   MQTT RGBW Light for Home-Assistant Using PCA9685 PWM Controller - NodeMCU (ESP8266)
   https://home-assistant.io/components/light.mqtt/
   Libraries :
    - ESP8266 core for Arduino : https://github.com/esp8266/Arduino
    - PubSubClient : https://github.com/knolleary/pubsubclient
    - Adafruit PWM Servo Driver - PCA9685 : https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
   Sources :
    - https://github.com/mertenats/Open-Home-Automation/tree/master/ha_mqtt_rgb_light (Based On - Thanks mertenats!)
    - File > Examples > ES8266WiFi > WiFiClient
    - File > Examples > PubSubClient > mqtt_auth
    - File > Examples > PubSubClient > mqtt_esp8266
    - File > Examples > Adafruit PWM Server Driver Library > pwmtest
    - http://forum.arduino.cc/index.php?topic=272862.0
    - http://blog.saikoled.com/post/44677718712/how-to-convert-from-hsi-to-rgb-white
   Configuration (HA) : 
    light:
      platform: mqtt
      name: 'Kitchen RGBW light'
      state_topic: 'kitchen/rgbw1/light/status'
      command_topic: 'kitchen/rgbw1/light/switch'
      brightness_state_topic: 'kitchen/rgbw1/brightness/status'
      brightness_command_topic: 'kitchen/rgbw1/brightness/set'
      rgb_state_topic: 'kitchen/rgbw1/rgb/status'
      rgb_command_topic: 'kitchen/rgbw1/rgb/set'
      brightness_scale: 4095
      optimistic: false
    binary_sensor:
      platform: mqtt
      state_topic: "kitchen/pir/status"
      name: "Kitchen Motion"
      qos: 0
      payload_on: "ON"
      payload_off: "OFF"
      sendor_class: motion

   Unreasonably Mundane - v0.21 - 2017/01
   ToDo:
      - Major code cleanup
      - Add local operation mode(s) if MQTT fails
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "math.h"
#define DEG_TO_RAD(X) (M_PI*(X)/180)
#define min(a,b) ((a)<(b)?(a):(b))

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MQTT_VERSION MQTT_VERSION_3_1_1


// Wifi: SSID and password
const char* WIFI_SSID = "[redacted]";
const char* WIFI_PASSWORD = "[redacted]";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "kitchen_rgbw_light";
const PROGMEM char* MQTT_SERVER_IP = "[redacted]";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "[redacted]";
const PROGMEM char* MQTT_PASSWORD = "[redacted]";

// MQTT: topics
// state
const PROGMEM char* MQTT_LIGHT_STATE_TOPIC = "kitchen/rgbw1/light/status";
const PROGMEM char* MQTT_LIGHT_COMMAND_TOPIC = "kitchen/rgbw1/light/switch";

// brightness
const PROGMEM char* MQTT_LIGHT_BRIGHTNESS_STATE_TOPIC = "kitchen/rgbw1/brightness/status";
const PROGMEM char* MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC = "kitchen/rgbw1/brightness/set";

// colors (rgb)
const PROGMEM char* MQTT_LIGHT_RGB_STATE_TOPIC = "kitchen/rgbw1/rgb/status";
const PROGMEM char* MQTT_LIGHT_RGB_COMMAND_TOPIC = "kitchen/rgbw1/rgb/set";

// payloads by default (on/off)
const PROGMEM char* LIGHT_ON = "ON";
const PROGMEM char* LIGHT_OFF = "OFF";

const PROGMEM char* MQTT_PIR_STATE_TOPIC = "kitchen/pir/status";

// variables used to store the state, the brightness and the color of the light
boolean m_rgb_state = false;
uint16_t m_rgb_brightness = 4095;
uint8_t m_rgb_red = 255;
uint8_t m_rgb_green = 255;
uint8_t m_rgb_blue = 255;
float m_hsi_hue = 0; //current hue (0-360)
float m_hsi_saturation = 0; //current saturation (0-1)
float m_hsi_intensity = 0; //current intensity (0-1)
float t_hsi_hue = 0; //target hue (0-360)
float t_hsi_saturation = 0; //target saturation (0-1)
float t_hsi_intensity = 0; //target intensity (0-1)
float t_hsi_hue_increment = 0;
float t_hsi_saturation_increment = 0;
float t_hsi_intensity_increment = 0;
uint16_t transitionSteps = 4096; //number of steps when transitioning Hue/Saturation/Intensity
boolean pwmInvert = true; //true for common anode (common +), false for common cathode (common -)

// PCA9685 pins used for the rgb led (PWM)
const PROGMEM uint8_t RGB_LIGHT_RED_PIN = 0;
const PROGMEM uint8_t RGB_LIGHT_GREEN_PIN = 1;
const PROGMEM uint8_t RGB_LIGHT_BLUE_PIN = 2;
const PROGMEM uint8_t RGB_LIGHT_WHITE_PIN = 3;

//i2c pins for PCA9685
const PROGMEM uint8_t i2cSDA = 2;
const PROGMEM uint8_t i2cSCL = 14;

// buffer used to send/receive data with MQTT
const uint8_t MSG_BUFFER_SIZE = 20;
char m_msg_buffer[MSG_BUFFER_SIZE]; 

int pirPin = 13; // Input for HC-S501
int pirValue = LOW; // Place to store read PIR Value

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// function called to adapt the brightness and the color of the led
void setColor(uint16_t p_red, uint16_t p_green, uint16_t p_blue) {
  float HSItemp[3];
  // if setting 0,0,0 intensity should be 0
  if(p_red == 0 && p_green == 0 && p_blue == 0) {
    t_hsi_intensity = 0;
  }
  rgb2hsi(p_red,p_green,p_blue,HSItemp);
  t_hsi_hue = HSItemp[0];
  t_hsi_saturation = HSItemp[1];
  if(t_hsi_hue != m_hsi_hue) {
    float huediff = t_hsi_hue - m_hsi_hue;
    if(abs(huediff) > 180){
      if(t_hsi_hue > m_hsi_hue){
        huediff = 360 - t_hsi_hue + m_hsi_hue;
        t_hsi_hue_increment = (huediff / (float)transitionSteps) * -1;
      } else {
        huediff = 360 - m_hsi_hue + t_hsi_hue;
        t_hsi_hue_increment = huediff / (float)transitionSteps;
      }
    } else {
      t_hsi_hue_increment = huediff / (float)transitionSteps;
    }
  }
  if(t_hsi_saturation != m_hsi_saturation) {
    t_hsi_saturation_increment = (t_hsi_saturation - m_hsi_saturation) / (float)transitionSteps;
  }
  if(t_hsi_intensity != m_hsi_intensity) {
    t_hsi_intensity_increment = (t_hsi_intensity - m_hsi_intensity) / (float)transitionSteps;
  }
}

void activateColor(float H, float S, float I) {
  int RGBWtemp[4];
  uint16_t p_red,p_green,p_blue,p_white;
  H = H>=0?(H<=360?H:(H-360)):(H+360); //handle H<0 or H>360 by adding or subtracting 360 so H can travel around the Hue circle in either direction
  hsi2rgbw(H,S,I,RGBWtemp);
  p_red = RGBWtemp[0];
  p_green = RGBWtemp[1];
  p_blue = RGBWtemp[2];
  p_white = RGBWtemp[3];
  p_red = map(p_red, 0, 255, 0, 4095);
  p_green = map(p_green, 0, 255, 0, 4095);
  p_blue = map(p_blue, 0, 255, 0, 4095);
  p_white = map(p_white, 0, 255, 0, 4095);
  pwm.setPin(RGB_LIGHT_RED_PIN, p_red, pwmInvert );
  pwm.setPin(RGB_LIGHT_GREEN_PIN, p_green, pwmInvert );
  pwm.setPin(RGB_LIGHT_BLUE_PIN, p_blue, pwmInvert );
  pwm.setPin(RGB_LIGHT_WHITE_PIN, p_white, pwmInvert);
  m_hsi_hue = H;
  m_hsi_saturation = S;
  m_hsi_intensity = I;
}

// function called to publish the state of the led (on/off)
void publishRGBState() {
  if (m_rgb_state) {
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_ON, true);
  } else {
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_OFF, true);
  }
}

// function called to publish the brightness of the led (0-100)
void publishRGBBrightness() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", m_rgb_brightness);
  client.publish(MQTT_LIGHT_BRIGHTNESS_STATE_TOPIC, m_msg_buffer, true);
}

// function called to publish the colors of the led (xx(x),xx(x),xx(x))
void publishRGBColor() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d,%d,%d", m_rgb_red, m_rgb_green, m_rgb_blue);
  client.publish(MQTT_LIGHT_RGB_STATE_TOPIC, m_msg_buffer, true);
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  // handle message topic
  if (String(MQTT_LIGHT_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(LIGHT_ON))) {
      if (m_rgb_state != true) {
        m_rgb_state = true;
        if(t_hsi_intensity==0) {
          t_hsi_intensity=1;
        }
        setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
        publishRGBState();
      }
    } else if (payload.equals(String(LIGHT_OFF))) {
      if (m_rgb_state != false) {
        m_rgb_state = false;
        t_hsi_intensity = 0;
        setColor(0, 0, 0);
        publishRGBState();
      }
    }
  } else if (String(MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC).equals(p_topic)) {
    uint16_t brightness = payload.toInt();
    if (brightness < 0 || brightness > 4095) {
      // do nothing...
      return;
    } else {
      m_rgb_brightness = brightness;
      t_hsi_intensity = (float)brightness / 4095;
      if(t_hsi_intensity != m_hsi_intensity) {
        t_hsi_intensity_increment = (t_hsi_intensity - m_hsi_intensity) / (float)transitionSteps;
      }
      publishRGBBrightness();
    }
  } else if (String(MQTT_LIGHT_RGB_COMMAND_TOPIC).equals(p_topic)) {
    // get the position of the first and second commas
    uint8_t firstIndex = payload.indexOf(',');
    uint8_t lastIndex = payload.lastIndexOf(',');
    
    uint8_t rgb_red = payload.substring(0, firstIndex).toInt();
    if (rgb_red < 0 || rgb_red > 255) {
      return;
    } else {
      m_rgb_red = rgb_red;
    }
    
    uint8_t rgb_green = payload.substring(firstIndex + 1, lastIndex).toInt();
    if (rgb_green < 0 || rgb_green > 255) {
      return;
    } else {
      m_rgb_green = rgb_green;
    }
    
    uint8_t rgb_blue = payload.substring(lastIndex + 1).toInt();
    if (rgb_blue < 0 || rgb_blue > 255) {
      return;
    } else {
      m_rgb_blue = rgb_blue;
    }
    
    setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
    publishRGBColor();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      
      // Once connected, publish an announcement...
      // publish the initial values
      publishRGBState();
      publishRGBBrightness();
      publishRGBColor();

      // ... and resubscribe
      client.subscribe(MQTT_LIGHT_COMMAND_TOPIC);
      client.subscribe(MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC);
      client.subscribe(MQTT_LIGHT_RGB_COMMAND_TOPIC);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  // init the serial
  Serial.begin(9600);

#ifdef ESP8266
  Wire.pins(i2cSDA, i2cSCL);   // ESP8266 can use any two pins, such as SDA to #2 and SCL to #14
#endif
  
  Serial.println("16 channel PWM test!");

  pwm.begin();
  pwm.setPWMFreq(400);  // This is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
#ifdef TWBR    
  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // upgrade to 400KHz!
#endif


  activateColor(0, 0, 0);

  // init the WiFi connection
  Serial.println();
  Serial.println();
  Serial.print("INFO: Connecting to ");
  WiFi.mode(WIFI_STA);
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("INFO: WiFi connected");
  Serial.print("INFO: IP address: ");
  Serial.println(WiFi.localIP());

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);

  //set PIR pin to input
  pinMode(pirPin, INPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if(t_hsi_hue != m_hsi_hue || t_hsi_saturation != m_hsi_saturation || t_hsi_intensity != m_hsi_intensity) {
    if(t_hsi_hue != m_hsi_hue) {
      float huediff = t_hsi_hue - m_hsi_hue;
      if(abs(huediff) < abs(t_hsi_hue_increment)) { //check if hue difference is less than increment, if so set hue directly to target
        m_hsi_hue = t_hsi_hue;
        t_hsi_hue_increment = 0;
      }
      if (m_hsi_intensity == 0) { //if light is off set hue directly (don't transition)
        m_hsi_hue = t_hsi_hue;
        t_hsi_hue_increment = 0;
      }
      if (t_hsi_intensity == 0) { //if light is going off keep hue the sam
        t_hsi_hue = m_hsi_hue;
        t_hsi_hue_increment = 0;
      }
    } else {
      t_hsi_hue_increment = 0; //if we're not transitioning hue clean increment
    }
    if(t_hsi_saturation != m_hsi_saturation) {
      float satdiff = t_hsi_saturation - m_hsi_saturation;
      if(abs(satdiff) < abs(t_hsi_saturation_increment)) {
        m_hsi_saturation = t_hsi_saturation;
        t_hsi_saturation_increment = 0;
      }
      if (m_hsi_intensity == 0) {
        m_hsi_saturation = t_hsi_saturation;
        t_hsi_saturation_increment = 0;
      }
      if (t_hsi_intensity == 0) {
        t_hsi_saturation = m_hsi_saturation;
        t_hsi_saturation_increment = 0;
      }
    } else {
      t_hsi_saturation_increment = 0;
    }
    if(t_hsi_intensity != m_hsi_intensity) {
      float intdiff = t_hsi_intensity - m_hsi_intensity;
      if(abs(intdiff) < abs(t_hsi_intensity_increment)) {
        m_hsi_intensity = t_hsi_intensity;
        t_hsi_intensity_increment = 0;
      }
      if (m_hsi_intensity == 0 && t_hsi_intensity == 0) { // if we've just finished shutting off set hue and saturation to 0
        m_hsi_saturation = 0;
        t_hsi_saturation = 0;
        m_hsi_hue = 0;
        t_hsi_hue = 0;
      }
    } else {
      t_hsi_intensity_increment = 0;
    }
    activateColor(m_hsi_hue + t_hsi_hue_increment,m_hsi_saturation + t_hsi_saturation_increment,m_hsi_intensity + t_hsi_intensity_increment);
  }
  if(digitalRead(pirPin) != pirValue) {
    pirValue = digitalRead(pirPin);
    if(pirValue == HIGH) {
      client.publish(MQTT_PIR_STATE_TOPIC, "ON", true);
    } else {
      client.publish(MQTT_PIR_STATE_TOPIC, "OFF", true);
    }
  }
}

void hsi2rgbw(float H, float S, float I, int* rgbw) {
  int r, g, b, w;
  float cos_h, cos_1047_h;
  H = floatmod(H,360); // cycle H around to 0-360 degrees
  H = 3.14159*H/(float)180; // Convert to radians.
  S = S>0?(S<1?S:1):0; // clamp S and I to interval [0,1]
  I = I>0?(I<1?I:1):0;
  
  if(H < 2.09439) {
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667-H);
    r = S*255*I/3*(1+cos_h/cos_1047_h);
    g = S*255*I/3*(1+(1-cos_h/cos_1047_h));
    b = 0;
    w = 255*(1-S)*I;
  } else if(H < 4.188787) {
    H = H - 2.09439;
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667-H);
    g = S*255*I/3*(1+cos_h/cos_1047_h);
    b = S*255*I/3*(1+(1-cos_h/cos_1047_h));
    r = 0;
    w = 255*(1-S)*I;
  } else {
    H = H - 4.188787;
    cos_h = cos(H);
    cos_1047_h = cos(1.047196667-H);
    b = S*255*I/3*(1+cos_h/cos_1047_h);
    r = S*255*I/3*(1+(1-cos_h/cos_1047_h));
    g = 0;
    w = 255*(1-S)*I;
  }
  
  rgbw[0]=r;
  rgbw[1]=g;
  rgbw[2]=b;
  rgbw[3]=w;
}

void rgb2hsi(float r,float g, float b, float* hsi) {
  float H,S,I;
  float SMinTemp, SMin;
  I = (r/255)+(g/255)+(b/255);
  I = I>0?(I<1?I:1):0; //clamp I to 0-1
  if(I==0) {
    S = 0;
  } else {
    SMinTemp = min(r,g);
    SMin = min(SMinTemp,b);
    S = 1-(SMin/255*3/I);
    S = S>0?(S<1?S:1):0; //clamp S to 0-1
  }

  if(r!=g || g!=b) {
    H=acos((r-0.5*g-0.5*b)/sqrt(sq(r)+sq(g)+sq(b)-r*g-r*b-g*b))*60;
    if(b>g) {
      H=360-H;
    }
  } else {
    H=0;
  }
  hsi[0] = H;
  hsi[1] = S;
  hsi[2] = I;
}

double floatmod(double a, double b) //fmod doesn't compile
{
    return (a - b * floor(a / b));
}
