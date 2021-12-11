#include <Arduino.h>
#include <uptime_formatter.h>
#include <string.h>
#include <Adafruit_BMP280.h>
#include <WiFiManager.h>

#include "connect_mqtt.h"

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setupBMP(void);
float get_Temperatur(void);
float get_Pressure(void);

String ProjectHostname = "SolarStation1";

typedef struct data
{ //Data for RTC Memory
  float temperatur;
  float pressure;
  unsigned int counter = 0;
};

data RTCdata;
float BatteryVoltage;
float VoltageFactor = 0.005089894;
double Sleeptime = 0;

bool upgradeMode = false;

WiFiManager wifiManager;

void setup()
{
  ESP.wdtDisable();
  Serial.begin(115200);
  Serial.println("");
  Wire.begin(D2, D3);

  setupBMP();
  ESP.wdtFeed();

  Serial.println(ESP.getResetInfo());
  Serial.println(ESP.getResetReason());

  if (ESP.getResetReason() == "Hardware Watchdog")
  {
    Serial.print("Hardware Watchdog, max Sleeptime ");
    Serial.println(ESP.deepSleepMax() / 60000000);
    delay(3000);
    ESP.deepSleep(ESP.deepSleepMax());
    delay(5000);
  }

  //Messure Battery Voltage
  pinMode(A0, INPUT);
  BatteryVoltage = VoltageFactor * analogRead(A0);
  Serial.print("Battery messurment: ");
  Serial.print(BatteryVoltage);
  Serial.println("V");
  Serial.print("Battery messurment raw: ");
  Serial.println(analogRead(A0));
  ESP.wdtFeed();

  if (BatteryVoltage < 3 && BatteryVoltage > 0.1)
  {
    Serial.println("Battery Voltage to low, max Sleeptime");
    delay(3000);
    ESP.deepSleep(ESP.deepSleepMax());
    delay(5000);
  }
  else
  {
    //Calculate sleeptime based on Battery Voltage
    if (BatteryVoltage > 4.2)
    {
      Sleeptime = (5 * 60 * 1000000);
    }
    else if (BatteryVoltage < 0.1)
    {
      Serial.println("Programmingmode dedected");
      delay(5000);
      Sleeptime = 5000000;
    }
    else
    {
      Sleeptime = ((((4.2 - BatteryVoltage) * (4.2 - BatteryVoltage) * 100) + 5) * 60 * 1000000);
    }

    Serial.print("Sleeptime: ");
    Serial.print(Sleeptime / 60000000);
    Serial.println(" minutes");
    ESP.wdtFeed();

    sensors_event_t temp_event, pressure_event;
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);
    float temperatur = temp_event.temperature;
    float pressure = pressure_event.pressure;
    Serial.print("Temperatur: ");
    Serial.print(temperatur);
    Serial.println(" °C");
    Serial.print("Luftdruck: ");
    Serial.print(pressure);
    Serial.println(" mBar");

    ESP.rtcUserMemoryRead(65, (uint32_t *)&RTCdata, (sizeof(data)));
    float diff = float(abs(temperatur * 100 - RTCdata.temperatur * 100)) / 100;
    if (RTCdata.counter > 100)
    {
      RTCdata.counter = 0;
    }
    ESP.wdtFeed();

    Serial.print("old Temperatur: ");
    Serial.print(RTCdata.temperatur);
    Serial.println("°C");
    Serial.print("new Temperatur: ");
    Serial.print(temperatur);
    Serial.println("°C");
    Serial.print("diff: ");
    Serial.print(diff);
    Serial.println("°C");
    Serial.print("Reboots since last send: ");
    Serial.println(RTCdata.counter);
    ESP.wdtFeed();

    if (diff < 0.1 && RTCdata.counter < 10)
    {
      Serial.println("Diff to small no wifi needed");
      RTCdata.counter++;
      ESP.rtcUserMemoryWrite(65, (uint32_t *)&RTCdata, (sizeof(data)));
    }
    else
    {
      // Manage all Wifi-stuff
      WiFi.hostname(ProjectHostname + "_" + String(ESP.getChipId(), HEX));
      wifiManager.setTimeout(60);
      ESP.wdtFeed();

      ESP.wdtEnable(1000);
      if (!wifiManager.autoConnect(ProjectHostname.c_str()))
      {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.deepSleep(ESP.deepSleepMax());
        delay(5000);
      }
      ESP.wdtDisable();
      ESP.wdtFeed();
      /*
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
      WiFi.hostname(ProjectHostname + "_" + String(ESP.getChipId(), HEX));
      connect_wifi();
      
*/
      init_mqtt();
      String Status_topic = "home/" + ProjectHostname + "/esp_" + String(ESP.getChipId(), HEX) + "/status";
      String Data_topic = "home/" + ProjectHostname + "/esp_" + String(ESP.getChipId(), HEX) + "/data";
      String Status_message = "{\"BatteryVoltage\":\"" + String(BatteryVoltage) + "\",\"Sleeptime\":\"" + String(Sleeptime / 60000000) + "\",\"noWiFicounter\":\"" + String(RTCdata.counter) + "\",\"executingTime\":\"" + String(millis()) + "\",\"esp_id\":\"" + String(ESP.getChipId(), HEX) + "\",\"IP\":\"" + WiFi.localIP().toString() + "\"}";
      String Data_message = "{\"Temperatur\":\"" + String(temperatur) + "\",\"TemperaturDifference\":\"" + String(diff) + "\",\"Pressure\":\"" + String(pressure) + "\"}";
      ESP.wdtFeed();

      RTCdata.temperatur = temperatur;
      RTCdata.pressure = pressure;
      RTCdata.counter = 0;
      ESP.rtcUserMemoryWrite(65, (uint32_t *)&RTCdata, (sizeof(data)));
      ESP.wdtFeed();

      send_mqtt(Data_topic, Data_message, true);
      send_mqtt(Status_topic, Status_message, true);
      ESP.wdtFeed();

      Serial.println(Data_topic);
      Serial.println(Data_message);
      Serial.println(Status_topic);
      Serial.println(Status_message);
      ESP.wdtFeed();

      disconnect_mqtt();
      WiFi.disconnect();
    }
  }

  Serial.println("Go to DeepSleep");
  delay(300);
  ESP.deepSleep(Sleeptime);
  delay(5000);
  ESP.deepSleep(ESP.deepSleepMax());
  delay(5000);
}

void loop()
{
  //loop
  delay(5000);
  ESP.deepSleep(ESP.deepSleepMax());
  delay(5000);
}

void setupBMP()
{
  if (!bmp.begin())
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    Serial.print("Go to Deepsleep");
    delay(5000);
    ESP.deepSleep(ESP.deepSleepMax());
    delay(5000);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
}