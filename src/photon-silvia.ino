#include <OneWire.h>
#include <spark-dallas-temperature.h>
#include <pid.h>

#ifndef TEMPERATURE_PIN
#define TEMPERATURE_PIN D6
#endif

#ifndef RELAY_PIN
#define RELAY_PIN D7
#endif

#ifndef BREW_SETPOINT
#define BREW_SETPOINT 85
#endif

#ifndef STEAM_SETPOINT
#define STEAM_SETPOINT 125
#endif

#ifndef RESOLUTION
#define RESOLUTION 11 // 375 ms read
#endif

// temperature sensor defs
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress sensorAddress;

// PID params
double kp = 60;
double ki = 0;
double kd = 0;

// input, output, and setpoint
double setpoint, input, output;

// PID library
PID myPID(&input, &output, &setpoint, kp, ki, kd, PID::DIRECT);

// "window" size. Basically the duration of our slow PWMesque control.
int windowSize = 3000;
unsigned long windowStartTime;

void setup() {
  Particle.publish("photon-silvia", "Starting");

  // init sensors and report our power mode. Vdd (using the 3rd pin) power mode is recommended for high temperatures
  // like in the boiler
  sensors.begin();
  sensors.setResolution(RESOLUTION);
  if (sensors.isParasitePowerMode()) {
    Particle.publish("photon-silvia-parasite", "using parasite power mode");
  } else {
    Particle.publish("photon-silvia-powered", "using Vdd power mode");
  }


  // we also expose the vars for monitoring
  Particle.variable("currentTemp", input);
  Particle.variable("boilerPower", output);
  Particle.variable("windowSize", windowSize);

  // get the sensor's address
  sensors.getAddress(sensorAddress, 0);

  // async temp measurement so it doesn't mess with PID
  sensors.setWaitForConversion(false);


  // FIRST RUN. We want to start with the real temperature
  sensors.requestTemperatures(); // Send the command to get first temperatures
  updateInputTemperature(true); // update, waiting for it to measure/convert


  // TODO make this setpoint var based on input from steam switch
  setpoint = BREW_SETPOINT;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, windowSize);

  //turn the PID on
  myPID.SetMode(PID::AUTOMATIC);

  // start our "PWM" window
  windowStartTime = millis();

  // init the pin
  pinMode(RELAY_PIN, OUTPUT);
}

  // String temperatureStr = String(input);
  // Particle.publish("current temp", temperatureStr);

void loop() {
  // because the DS18B20 takes a while to read, we only refresh the input whenever available
  updateInputTemperature(false); // non-blocking update, if one's available
  myPID.Compute();               // update the PID params

  /************************************************
   * turn the output pin on/off based on pid analog output
   ************************************************/
  if (millis() - windowStartTime > windowSize) {
    // time to shift the Relay Window
    windowStartTime += windowSize;
  }

  if (output > millis() - windowStartTime) {
    digitalWrite(RELAY_PIN,HIGH);
  } else {
    digitalWrite(RELAY_PIN,LOW);
  }

}

void updateInputTemperature(bool wait) {
  while (true) {
    bool avail = sensors.isConversionAvailable(sensorAddress);
    if (avail) {
      input = sensors.getTempC(sensorAddress); // record in global var
      sensors.requestTemperatures(); // start the next temperature measurement
      return;
    }

    // keep looping if we need to wait
    if (wait) {
      delay(1);
    } else {
      break;
    }

  }
}
