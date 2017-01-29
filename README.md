##MQTT RGBW Light for Home-Assistant Using PCA9685 PWM Controller - NodeMCU (ESP8266)
   https://home-assistant.io/components/light.mqtt/

##Libraries :
- ESP8266 core for Arduino : https://github.com/esp8266/Arduino
- PubSubClient : https://github.com/knolleary/pubsubclient
- Adafruit PWM Servo Driver - PCA9685 : https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library

##Sources :
- https://github.com/mertenats/Open-Home-Automation/tree/master/ha_mqtt_rgb_light (Based On - Thanks mertenats!)
- File > Examples > ES8266WiFi > WiFiClient
- File > Examples > PubSubClient > mqtt_auth
- File > Examples > PubSubClient > mqtt_esp8266
- File > Examples > Adafruit PWM Server Driver Library > pwmtest
- http://forum.arduino.cc/index.php?topic=272862.0
- http://blog.saikoled.com/post/44677718712/how-to-convert-from-hsi-to-rgb-white

## Configuration (HA)
configuration.yaml :
```yaml
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
```

##Unreasonably Mundane - v0.21 - 2017/01
##ToDo:
- Major code cleanup
- Add local operation mode(s) if MQTT fails
- ~~Add Motion Sensor as seperate MQTT sensor~~
- ~~Shortest transition path for Hue~~
