#include <OneWire.h>
#include <spark-dallas-temperature.h>

#ifndef TEMPERATURE_PIN
#define TEMPERATURE_PIN D6
#endif

#ifndef RESOLUTION
#define RESOLUTION 11
#endif

OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature sensors(&oneWire);

void setup() {
  Particle.publish("photon-silvia", "Starting");

  sensors.begin();
  sensors.setResolution(RESOLUTION);
  if (sensors.isParasitePowerMode()) {
    Particle.publish("photon-silvia-parasite", "using parasite power mode");
  } else {
    Particle.publish("photon-silvia-powered", "using regular power mode");
  }
}

void loop() {
  Particle.publish("temp-request", "Requesting temp");
  sensors.requestTemperatures();
  Particle.publish("temp-requested", "Requested temp");
  float temperature = sensors.getTempCByIndex(0);
  String temperatureStr = String(temperature);
  Particle.publish("current temp", temperatureStr);
  delay(8000);
}
