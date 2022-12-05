#include "HomeSpan.h"
#include "extras/Pixel.h" // include the HomeSpan Pixel class
#include "DEV_Identify.h"
#include "DEV_Sensors.h"

//#include "getSensordata.h"

// LED Definitions:
/*#define NEOPIN 23
#define NUMPIXELS 1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

#define NEOPIXEL_RGB_PIN 23
#define NEOPIXEL_RGBW_PIN 32
#define DOTSTAR_DATA_PIN 33
#define DOTSTAR_CLOCK_PIN 27
#define DEVICE_SUFFIX ""
*/

/*
#ifdef ESP32
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#else
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#endif
*/
#include <Wire.h>

uint32_t timer = 0; // keep track of time since last update
/*
AsyncWebServer server(8080);
uint16_t predict;
uint8_t statu;
int32_t resistance;
uint16_t tvoc;

void readAllBytes()
{
  Wire.requestFrom(Co2iaqaddress, 9);

  predict = (Wire.read() << 8 | Wire.read());
  statu = Wire.read();
  resistance = (Wire.read() & 0x00) | (Wire.read() << 16) | (Wire.read() << 8 | Wire.read());
  tvoc = (Wire.read() << 8 | Wire.read());
}
void checkStatus()
{
  if (statu == 0x10)
  {
    LOG1("Warming up...");
  }
  else if (statu == 0x00)
  {
    LOG1("Ready");
  }
  else if (statu == 0x01)
  {
    LOG1("Busy");
  }
  else if (statu == 0x80)
  {
    LOG1("Error");
  }
  else
    LOG1("No Status, check module");
}

String read_co2()
{

  Wire.requestFrom(Co2iaqaddress, 9);

  predict = (Wire.read() << 8 | Wire.read());
  statu = Wire.read();
  resistance = (Wire.read() & 0x00) | (Wire.read() << 16) | (Wire.read() << 8 | Wire.read());
  tvoc = (Wire.read() << 8 | Wire.read());
  float co2 = predict;
  if (isnan(co2))
  {
    LOG1("Failed To Read data!");
    return "";
  }
  else
  {
    //    Serial.println("co2");
    //   Serial.println(co2);
    return String(co2);
  }
}

String read_voc()
{
  Wire.requestFrom(Co2iaqaddress, 9);

  predict = (Wire.read() << 8 | Wire.read());
  statu = Wire.read();
  resistance = (Wire.read() & 0x00) | (Wire.read() << 16) | (Wire.read() << 8 | Wire.read());
  tvoc = (Wire.read() << 8 | Wire.read());
  float voc = tvoc;
  if (isnan(voc))
  {
    LOG1("Failed to read from sensor!");
    return "";
  }
  else
  {

    // Serial.println(voc);
    return String(voc);
  }
}

String read_temperature()
{
  float temp = getTemp();
  return String(temp);
}

String read_humidity()
{
  float hum = getHumidity();
  if (isnan(hum))
  {
    Serial.println("Failed to read from BME280 sensor!");
    return "";
  }
  else
  {
    // Serial.println(hum);
    return String(hum);
  }
}
*/

/*
struct NeoPixel_RGB : Service::LightBulb

{ // Addressable single-wire RGB LED Strand (e.g. NeoPixel)

  Characteristic::On power{0, false};
  Characteristic::Hue H{0, true};
  Characteristic::Saturation S{0, true};
  Characteristic::Brightness V{100, true};
  Pixel *pixel;
  uint8_t nPixels;

  NeoPixel_RGB(uint8_t pin, uint8_t nPixels) : Service::LightBulb()
  {

    V.setRange(5, 100, 1);   // sets the range of the Brightness to be from a min of 5%, to a max of 100%, in steps of 1%
    pixel = new Pixel(pin);  // creates Pixel LED on specified pin
    this->nPixels = nPixels; // save number of Pixels in this LED Strand
    update();                // manually call update() to set pixel with restored initial values
  }

  boolean update() override
  {

    int p = power.getNewVal();

    float h = H.getNewVal<float>(); // range = [0,360]
    float s = S.getNewVal<float>(); // range = [0,100]
    float v = V.getNewVal<float>(); // range = [0,100]

    Pixel::Color color;

    Serial.println("h");
    Serial.println(h);
    Serial.println("p");
    Serial.println(p);
    Serial.println("s");
    Serial.println(s);
    Serial.println("v");
    Serial.println(v);

    pixel->set(color.HSV(h * p, s * p, v * p), nPixels); // sets all nPixels to the same HSV color

    return (true);
  }
};
*/
char sNumber[18] = "1d:1d:1d:1d:1d:1d"; // global

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  // Things Server:
  /*
    if (!SPIFFS.begin())
    {
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html"); });
    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/plain", String(getTemp()).c_str()); });
    server.on("/co2", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/plain", read_co2().c_str()); });
    server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/plain", read_humidity().c_str()); });
    server.on("/voc", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text/plain", String(tvoc).c_str()); });

    WiFi.begin();
    server.begin();
*/
  // Things homespan

  for (int i = 0; i < 17; ++i)
  {
    sNumber[i] = WiFi.macAddress()[i];
  }
  sNumber[17] = '\0';

  homeSpan.begin(Category::Sensors, "Airquality Sensor");

  new SpanAccessory();
  new DEV_Identify("Luftqualität", "Luftqualität ", sNumber, "HS Sensors", "0.1a", 10);
  new Service::HAPProtocolInformation();
  new Characteristic::Version("1.1.0");

  new SpanAccessory();
  new DEV_Identify("Temperature Sensor", "Luftqualität, Temperatur", sNumber, "Temperatur Sensor", "0.1a", 0);
  new DEV_TempSensor();

  new SpanAccessory();
  new DEV_Identify("VoC Sensor", "Luftqualität, VoC", sNumber, "VoC Sensor", "0.1a", 0);
  new DEV_VoCSensor();

  new SpanAccessory();
  new DEV_Identify("Co2 Sensor", "Luftqualität, Co2", sNumber, "Co2 Sensor", "0.1a", 0);
  new DEV_Co2Sensor();

  new SpanAccessory();
  new DEV_Identify("Humidity Sensor", "Luftqualität, Luftfeuchte", sNumber, "Luftfeuchte Sensor", "0.1a", 0);
  new DEV_HumiditySensor();

  /*
    new SpanAccessory(); // create Bridge
    new Service::AccessoryInformation();
    new Characteristic::Name("Pixel LEDS" DEVICE_SUFFIX);
    new Characteristic::Manufacturer("HomeSpan");
    new Characteristic::SerialNumber("123-ABC");
    new Characteristic::Model("Neo/Dot Pixels");
    new Characteristic::FirmwareRevision("1.0");
    new Characteristic::Identify();
  new Service::HAPProtocolInformation();
  new Characteristic::Version("1.1.0");

  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Name("Neo RGB");
  new Characteristic::Manufacturer("HomeSpan");
  new Characteristic::SerialNumber("123-ABC");
  new Characteristic::Model("8-LED Strand");
  new Characteristic::FirmwareRevision("1.0");
  new Characteristic::Identify();

  new NeoPixel_RGB(NEOPIXEL_RGB_PIN, 1); // create 1-LED NeoPixel RGB Strand with full color control
  */

} // end of setup()

//////////////////////////////////////

void loop()
{
  if (millis() - timer > 5000)
  { // only sample every 5 seconds
    timer = millis();
    /*  readAllBytes();
      checkStatus();
    */
  }

  homeSpan.poll();

} // end of loop()

//////////////////////////////////////
