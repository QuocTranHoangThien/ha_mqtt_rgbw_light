# ha_mqtt_rgbw_light
   MQTT RGBW Light for Home-Assistant Using PCA9685 PWM Controller - NodeMCU (ESP8266)<br>
   https://home-assistant.io/components/light.mqtt/<br>
   Libraries :<br>
    - ESP8266 core for Arduino : https://github.com/esp8266/Arduino<br>
    - PubSubClient : https://github.com/knolleary/pubsubclient<br>
    - Adafruit PWM Servo Driver - PCA9685 : https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library<br>
   Sources :<br>
    - https://github.com/mertenats/Open-Home-Automation/tree/master/ha_mqtt_rgb_light (Based On - Thanks mertenats!)<br>
    - File > Examples > ES8266WiFi > WiFiClient<br>
    - File > Examples > PubSubClient > mqtt_auth<br>
    - File > Examples > PubSubClient > mqtt_esp8266<br>
    - File > Examples > Adafruit PWM Server Driver Library > pwmtest<br>
    - http://forum.arduino.cc/index.php?topic=272862.0<br>
    - http://blog.saikoled.com/post/44677718712/how-to-convert-from-hsi-to-rgb-white<br>
   Configuration (HA) : <br>
    light:<br>
      platform: mqtt<br>
      name: 'Kitchen RGBW light'<br>
      state_topic: 'kitchen/rgbw1/light/status'<br>
      command_topic: 'kitchen/rgbw1/light/switch'<br>
      brightness_state_topic: 'kitchen/rgbw1/brightness/status'<br>
      brightness_command_topic: 'kitchen/rgbw1/brightness/set'<br>
      rgb_state_topic: 'kitchen/rgbw1/rgb/status'<br>
      rgb_command_topic: 'kitchen/rgbw1/rgb/set'<br>
      brightness_scale: 4095<br>
      optimistic: false<br>
      <br>
   Unreasonably Mundane - v0.1 - 2017/01<br>
   ToDo:<br>
      - Major code cleanup<br>
      - Add Motion Sensor as seperate MQTT sensor<br>
      - Shortest transition path for Hue<br>
