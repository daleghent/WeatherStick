/*
 * Original Author: Kai Wicker <kai@photonenfangen.de>
 * Website: http://photonenfangen.de/instruments/ascom-temperature-sensor/
 * 
 * Further modifications by: Dale Ghent <daleg@elemental.org>
 * As a part of the Weather Stick project based on Kai Wicker's ArduinoTempHumV2 plans.
 * https://github.com/daleghent/WeatherStick
 * 
 * Dependencies:
 *    - Adafruit Unified Sensor
 *    - Adafruit BME280 Library
 *    - DHT sensor library
 */
#include <DHT.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/*
 * A modified BME280 library is used to control the BMP280.
 * Changes to the Adafruit BME280 library are:
 * 
 * Adafruit_BME280.h:17: Default I2C address from 0x77 to 0x76
 * Adafruit_BME280.cpp:139: Chip identifier from 0x60 to 0x58
 * 
 * The I2C pins of the Nano are A4 (SCL) and A5(SDA)
 */

 /*
  * Version of this code
  */
#define VERSION 4.0

/*
 * Pin 2 is used to wake the processer from sleep. Serial
 * Rx traffic is connected to this pin through a 470 ohm
 * resistor. When it goes high, traffic is received and wakes
 * the CPU from sleep. The serial traffic is not buffered, so
 * each command must be preceeded by a # command to wake the CPU.
 */
#define INT_PIN 2

/*
 * Definitions for the DHT22
 */
#define DHT_PIN 7
#define LED_PIN 13
#define DHTTYPE DHT22

/*
 * Standard air pressure at sea level (SI unit)
 */
#define SEALEVELPRESSURE_HPA (1013.25)

/*
 * Prototypes
 */
DHT dht(DHT_PIN, DHTTYPE);

/*
 * Global variables
 */
float humidity, temperature, bmptemperature, pressure;
Adafruit_BME280 bme;

/********
 * Main *
 ********/

void setup() {
  pinMode(INT_PIN, INPUT);        // Pullup for INT_PIN
  digitalWrite(INT_PIN, HIGH);    // Arduino sleeps untils serial get actice.
  pinMode(LED_PIN, OUTPUT);       // LED for activity status
  dht.begin();

  // Default settings

  // If we cannot initialize the BME/BMP sensor, complain and enter an infinite loop to prev
  bool status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  messung();                      // just to have valid data ready
  delay(100);                   // to ensure not sleep to early

  Serial.begin(57600);
  Serial.flush();
  Serial.println("##");
}

void loop() {
  String cmd;

  digitalWrite(LED_PIN, LOW);   // Turn off the L LED
  enter_sleep();                // and sleep
  digitalWrite(LED_PIN, HIGH);  // We woke up: switch on the L LED

  messung();                    // Collect data from the sensors

  // For 5 sections (5000ms) listen and process any incoming commands, then
  // go back to sleep.
  for (int i = 0; i <= 5000; i++) {
    delay(1);
    while (Serial.available() > 0) {
      cmd = Serial.readStringUntil('#');

      if (cmd == "TEM") {
        Serial.print(temperature); Serial.println('#');
      }
      if (cmd == "HUM") {
        Serial.print(humidity); Serial.println('#');
      }
      if (cmd == "DEW") {
        Serial.print(dewPoint(temperature, humidity)); Serial.println('#');
      }
      if (cmd == "PRE") {
        Serial.print(pressure); Serial.println('#');
      }
      if (cmd == "VER") {
        Serial.print(VERSION); Serial.println('#');
      }
      if (cmd == "DBG") {
        printDBGValues();
      }
    }
  }
}

void INT_PINisr(void) {
  /* ISR for INT_PIN                      */
  /* Detach Interrupt, one time is enough for now. Lets   */
  /* the serial communication run without any interrupts.   */
  detachInterrupt(0);
}

void enter_sleep(void) {
  attachInterrupt(0, INT_PINisr, LOW);
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_mode();
  sleep_disable();
}

float dewPoint(double celsius, double humidity) {
  /*  calculate dew point, not the best method, but simple  */
  /*  and good enough for our tasks             */
  double  a     = 17.271;
  double  b     = 237.7;
  double  temp  = (a * celsius) / (b + celsius) + log(humidity / 100);
  return (b * temp) / (a - temp);
}

void messung() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  pressure = bme.readPressure() / 100.0F;  /* hPa */
}

void printDBGValues() {
  Serial.print("Pressure (BMP280) = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude (BMP280) = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Temperature (BMP280) = ");
  Serial.print(bme.readTemperature());
  Serial.println(" C");
  
  Serial.print("Temperature (DHT22) = ");
  Serial.print(dht.readTemperature());
  Serial.println(" C");
  
  Serial.print("Humidity (DHT22) = ");
  Serial.print(dht.readHumidity());
  Serial.println("%");

  Serial.print("Dewpoint (using BMP280) = ");
  Serial.print(dewPoint(bme.readTemperature(), dht.readHumidity()));
  Serial.println(" C");

  Serial.print("Dewpoint (using DHT22) = ");
  Serial.print(dewPoint(dht.readTemperature(), dht.readHumidity()));
  Serial.println(" C");
  
  Serial.println();
}
