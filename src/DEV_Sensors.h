
////////////////////////////////////
/*
 * Temp 		40
 * CO2 		  5A
 */
////////////////////////////////////

#include <Wire.h>
#include "iAQCoreI2C.h"
#include <Adafruit_NeoPixel.h>
#define Co2iaqaddress 0x5A
#define TmpIaqaddress 0x40
iAQCoreI2C iaq;
#include "SparkFun_Si7021_Breakout_Library.h"
#include "MegunoLink.h" // for plotting
#include "Filter.h"
// Create a new exponential filter with a weight of 5 and an initial value of 0.
long FilterWeight = 5;
ExponentialFilter<long> ADCFilter(FilterWeight, 130);

// LED Definitions:
#define NEOPIN 23
#define NUMPIXELS 1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);

// DisplayDefinitions:
/*
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
*/

double getTemp()
{
  Weather sensor;
  if (!sensor.begin())
  {
    sensor.begin();
  }

  float temp;
  temp = sensor.getTemp();
  temp = temp - 4;
  return temp;
  /*
    double t;

    t = temperature.readTemperatureC();
    delay(200);
    return (t);
  */
}

double getHumidity()
{
  Weather sensor;
  if (!sensor.begin())
  {
    sensor.begin();
  }
  float humidity;
  humidity = sensor.getRH();
  return humidity;
}

double getCo2()
{

  ////Wire.begin();
  Wire.requestFrom(Co2iaqaddress, 9);
  double peakLevelCo2;
  double co2 = (Wire.read() << 8 | Wire.read());
  double statu = Wire.read();
  double resistance = (Wire.read() & 0x00) | (Wire.read() << 16) | (Wire.read() << 8 | Wire.read());
  double VoCMeasure = (Wire.read() << 8 | Wire.read());

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

  LOG1("Co2 PPM:");
  LOG1(co2);

  return (co2);
}

void setAirQualityLED(int airQualityNumber)
{
  if (airQualityNumber == 1)
  {
    // gute Luft
    pixels.setPixelColor(0, pixels.Color(7, 18, 0));
  }
  else if (airQualityNumber == 2)
  {
    // luft Mittelmäßig
    pixels.setPixelColor(0, pixels.Color(17, 18, 0));
  }
  else if (airQualityNumber == 3)
  {
    // luft Mittelmäßig
    pixels.setPixelColor(0, pixels.Color(18, 8, 0));
  }
  else if (airQualityNumber == 4)
  {
    // luft Mittelmäßig
    pixels.setPixelColor(0, pixels.Color(18, 1, 0));
  }
  else if (airQualityNumber == 5)
  {
    // luft schlecht
    pixels.setPixelColor(0, pixels.Color(18, 0, 8));
  }

  pixels.show(); // Send the updated pixel colors to the hardware.
}

double getAirQualityNumber(double VoCMeasure)
{
  double AirQual;
  if (VoCMeasure < 130)
  {
    // gute Luft
    AirQual = 1;
  }
  else if (VoCMeasure >= 130 && VoCMeasure < 230)
  {
    // luft Mittelmäßig
    AirQual = 2;
  }
  else if (VoCMeasure >= 230 && VoCMeasure < 300)
  {
    // luft Mittelmäßig
    AirQual = 3;
  }
  else if (VoCMeasure >= 300 && VoCMeasure < 400)
  {
    // luft Mittelmäßig
    AirQual = 4;
  }
  else if (VoCMeasure >= 400)
  {
    // luft schlecht
    AirQual = 5;
  }
  return AirQual;
}

double getVoc()
{

  // Wire.begin();
  Wire.requestFrom(Co2iaqaddress, 9);

  double predict = (Wire.read() << 8 | Wire.read());
  double statu = Wire.read();
  double resistance = (Wire.read() & 0x00) | (Wire.read() << 16) | (Wire.read() << 8 | Wire.read());
  double tvoc = (Wire.read() << 8 | Wire.read());

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

  LOG1("TVoC:");
  LOG1(tvoc);
  return (tvoc);
}

struct DEV_TempSensor : Service::TemperatureSensor
{ // A standalone Temperature sensor

  SpanCharacteristic *temp; // reference to the Current Temperature Characteristic
  int addr;                 // I2C address of temperature sensor
  uint32_t timer = 0;       // keep track of time since last update

  DEV_TempSensor() : Service::TemperatureSensor()
  { // constructor() method

    this->addr = addr; // I2C address of temperature sensor

    // Wire.begin(); // start I2C in Controller Mode

    double tempC = getTemp();
    // Serial.print("Temp");
    // Serial.println(tempC);

    temp = new Characteristic::CurrentTemperature(tempC); // instantiate the Current Temperature Characteristic
    temp->setRange(-50, 100);                             // expand the range from the HAP default of 0-100 to -50 to 100 to allow for negative temperatures

    //   Serial.printf("Configuring Temperature Sensor ADT7410 version 0x%02X with address 0x%02X: %0.2f C.\n", id, addr, tempC);        // initialization message

  } // end constructor

  void loop()
  {

    char c[64];

    if (millis() - timer > 5000)
    { // only sample every 5 seconds
      timer = millis();

      double tempC = getTemp();
      // Serial.println("temp");
      // Serial.println(tempC);
      if (abs(temp->getVal<double>() - tempC) > 0.1)
      {                      // only update temperature if change is more than 0.1C
        temp->setVal(tempC); // set the new temperature; this generates an Event Notification and also resets the elapsed time
        sprintf(c, "ADT7410-%02X Temperature Update: %g\n", addr, tempC);
        LOG1(c);
      }
    }

  } // loop
};

//////////////////////////////////

struct DEV_VoCSensor : Service::AirQualitySensor
{ // A standalone VoC sensor

  SpanCharacteristic *VoC;        // reference to the Current VoC Characteristic
  SpanCharacteristic *airQuality; // reference to the Air Quality Characteristic, which is an integer from 0 to 5

  int addr;           // I2C address of temperature sensor
  uint32_t timer = 0; // keep track of time since last update

  DEV_VoCSensor() : Service::AirQualitySensor()
  { // constructor() method

    this->addr = addr; // I2C address of temperature sensor

    // Wire.begin(); // start I2C in Controller Mode

    /*
        pixels.begin();
        pixels.setPixelColor(0, 0, 0, 0);
        pixels.show();
    */

    double VoCMeasure = 50;

    // Serial.print("VOC");
    // Serial.println(VoCMeasure);

    airQuality = new Characteristic::AirQuality(1); // instantiate the Air Quality Characteristic and set initial value to 1
    VoC = new Characteristic::VOCDensity(VoCMeasure);
    // VoC->setRange(-50, 60000);

  } // end constructor

  void loop()
  {

    char c[64];

    if (millis() - timer > 5000)
    { // only sample every 5 seconds
      timer = millis();

      double VoCMeasure;
      // Serial.println(VoCMeasure);

      int RawValue = getVoc();
      ADCFilter.Filter(RawValue);
      VoCMeasure = ADCFilter.Current();
      setAirQualityLED(getAirQualityNumber(VoCMeasure));

      Serial.println(RawValue);
      Serial.println(VoCMeasure);

      if (abs(VoC->getVal<double>() - VoCMeasure) > 0.1)
      {                          // only update VoC if change is more than 1
        VoC->setVal(VoCMeasure); // set the new VoC; this generates an Event Notification and also resets the elapsed time
        sprintf(c, "ADT7410-%02X VoC Update: %g\n", addr, VoCMeasure);
        LOG1(c);
      }

      if (abs(airQuality->getVal<double>() - getAirQualityNumber(VoCMeasure)) > 0.1)
      {                                                      // only update airQuality if change is more than 1
        airQuality->setVal(getAirQualityNumber(VoCMeasure)); // set the new airQuality; this generates an Event Notification and also resets the elapsed time
        sprintf(c, "ADT7410-%02X airQuality Update: %g\n", addr, getAirQualityNumber(VoCMeasure));
        LOG1(c);
      }
    }

  } // loop
};

//////////////////////////////////

struct DEV_Co2Sensor : Service::CarbonDioxideSensor
{ // A standalone Co2 sensor

  SpanCharacteristic *Co2Level;    // reference to the Current Temperature Characteristic
  SpanCharacteristic *Co2Detected; // reference to the Current Temperature Characteristic
  SpanCharacteristic *Co2PeakLevel;
  int addr;           // I2C address of temperature sensor
  uint32_t timer = 0; // keep track of time since last update

  DEV_Co2Sensor() : Service::CarbonDioxideSensor()
  { // constructor() method

    this->addr = addr; // I2C address of temperature sensor

    // Wire.begin(); // start I2C in Controller Mode

    double Co2 = 200;
    double peakLevelCo2 = 200;
    // Serial.print("CO2");
    // Serial.println(Co2);

    Co2Detected = new Characteristic::CarbonDioxideDetected();
    Co2Level = new Characteristic::CarbonDioxideLevel(Co2);
    Co2PeakLevel = new Characteristic::CarbonDioxidePeakLevel(peakLevelCo2);

  } // end constructor

  void loop()
  {

    char c[64];
    // Serial.print("sensor has valid value: "); LOG1(iaq.hasValue() ? "true" : "false");

    if (millis() - timer > 5000)
    { // only sample every 5 seconds
      timer = millis();

      double peakLevelCo2;
      double co2 = getCo2();

      // Only get realistic Co2 Values ( exclue warmup etc..)
      if (co2 > 3500 || co2 < 300)
      {
        co2 = 450;
      }

      // Update Peak Level if changed to previouse
      if ((abs(Co2PeakLevel->getVal<double>())) < co2)
      {                            // only update co2_peak if change is more than 1
        Co2PeakLevel->setVal(co2); // set the new co2_peak level; this generates an Event Notification and also resets the elapsed time
        sprintf(c, "ADT7410-%02X Co2PeakLevel Update: %g\n", addr, co2);
        LOG1(c);
      }

      if (abs(Co2Level->getVal<double>() - co2) > 1)
      {                        // only update co2 if change is more than 1
        Co2Level->setVal(co2); // set the new co2 level; this generates an Event Notification and also resets the elapsed time
        sprintf(c, "ADT7410-%02X Co2 Update: %g\n", addr, co2);
        LOG1(c);
      }
    }

  } // loop
};

//////////////////////////////////

/*struct HumiditySensor : SpanService { HumiditySensor() : SpanService{"82","HumiditySensor"}{
    REQ(CurrentRelativeHumidity);
    OPT(Name);
    OPT(StatusActive);
    OPT(StatusFault);
    OPT(StatusTampered);
    OPT(StatusLowBattery);   */

struct DEV_HumiditySensor : Service::HumiditySensor
{ // A standalone Co2 sensor

  SpanCharacteristic *currentHumidity; // reference to the Current Humidity
  int addr;                            // I2C address of temperature sensor
  uint32_t timer = 0;                  // keep track of time since last update

  DEV_HumiditySensor() : Service::HumiditySensor()
  { // constructor() method

    this->addr = addr; // I2C address of temperature sensor

    // Wire.begin(); // start I2C in Controller Mode

    double hum = 60;

    // Serial.print("CO2");
    // Serial.println(Co2);

    currentHumidity = new Characteristic::CurrentRelativeHumidity(hum);

  } // end constructor

  void loop()
  {

    char c[64];
    // Serial.print("sensor has valid value: "); LOG1(iaq.hasValue() ? "true" : "false");

    if (millis() - timer > 5000)
    { // only sample every 5 seconds
      timer = millis();
      double humidity = getHumidity();

      // Update Peak Level if changed to previouse
      if (abs(currentHumidity->getVal<double>() - humidity) > 1)
      {                                    // only update humidity if change is more than 1
        currentHumidity->setVal(humidity); // set the new humidity level; this generates an Event Notification and also resets the elapsed time
        sprintf(c, "ADT7410-%02X currentHumidity Update: %g\n", addr, humidity);
        LOG1(c);
      }
    }

  } // loop
};
