// copyright 2021 BotanicFields, Inc.
// M5Atom, DFPlayerMini, AdcButton, and ENV unit
//
//    Please install library before compiling:
//    Adafruit BMP280: https://github.com/adafruit/Adafruit_BMP280_Library

#include <M5Atom.h>
#include "DHT12.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "BF_DfplayerMini.h"
#include "BF_AdcButton.h"

// for ENV unit
DHT12 dht12;
Adafruit_BMP280 bme;

// for DFplayer-Mini
DfplayerMini Dfpm;
int dfpm_select(0);

void setup()
{
  const bool serial_enable(true);
  const bool i2c_enable(true);
  const bool display_enable(true);
  M5.begin(serial_enable, !i2c_enable, display_enable);
  delay(3000);
  Serial.println("M5Stack begin");

  // Port I2C
  const int wire_scl(21);       // GPIO21
  const int wire_sda(25);       // GPIO25
  const int wire_freq(100000);  // 100kHz
  Wire.begin(wire_sda, wire_scl, wire_freq);
  Serial.println("Wire begin");

  // for ENV unit
  while (!bme.begin(0x76))
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  Serial.println("BME begin");

  // Port Analog
  const int analog_port_button_pin(33);  // GPIO33
  const int ambient_light_pin(34);       // GPIO34

  // for ADC Button and Ambient Light
  AdcButtonBegin(analog_port_button_pin);
  ambient_light_sensor.Begin(ambient_light_pin);
  Serial.println("ADC begin");

  // Port UART
  const int serial2_rx(22);  // GPIO22
  const int serial2_tx(19);  // GPIO19
  Serial2.begin(9600, SERIAL_8N1, serial2_rx, serial2_tx);

  // for DFPlayer-Mini
  const bool feedback_enable(true);
  Dfpm.Begin(Serial2, !feedback_enable);
  delay(3000);
  Dfpm.Reset();
}

void loop()
{
  M5.update();
  AdcButtonUpdate();
  Dfpm.Update();

  if (btn_right.wasReleased()) Dfpm.Next();
  if (btn_up.wasReleased())    Dfpm.IncreaseVolume();
  if (btn_down.wasReleased())  Dfpm.DecreaseVolume();
  if (btn_left.wasReleased())  Dfpm.Previous();
  if (btn_select.wasReleased())  // Playing will be stopped by selecting device
    switch (dfpm_select) {
      case  0: Dfpm.SelectDevice(0x01);  dfpm_select = 1;  break;
      case  1: Dfpm.SelectDevice(0x02);  dfpm_select = 0;  break;
      default: dfpm_select = 0;  break;
    }

  static unsigned int ambient_last_ms(0);
  if (millis() - ambient_last_ms > 10000) {  // every 10sec
    float tmp = dht12.readTemperature();
    float hum = dht12.readHumidity();
    float pressure = bme.readPressure();
    int ambient_light = ambient_light_sensor.Read();
    Serial.printf("Temperature: %2.2f*C  Humidity: %0.2f%%  Pressure: %0.2fPa  Ambient light: %d\n", tmp, hum, pressure, ambient_light);
    ambient_last_ms = millis();
  }
}
