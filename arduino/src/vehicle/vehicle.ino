#include "simpleHC12.h"
#include <SoftwareSerial.h>
#include <SimpleRelay.h>
#include <CountDown.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

const unsigned int hc12RxPin = 3;
const unsigned int hc12TxPin = 2;
const unsigned int hc12SetPin = 1;

const unsigned long baudRate = 9600;
const unsigned messageLength = 20;
boolean useChecksum = true;
char data[messageLength + 1];
char message[messageLength];

simpleHC12 HC12(hc12TxPin, hc12RxPin, hc12SetPin, baudRate, messageLength, useChecksum);

const unsigned int relayPin = 12;
SimpleRelay relay = SimpleRelay(relayPin, true);

const unsigned int engineShutoffSec = 5;
CountDown countdown(CountDown::SECONDS);

const unsigned int crashAlertSec = 5;
CountDown crashCountdown(CountDown::SECONDS);
const unsigned int locationCheckSec = 20;
CountDown locationCheckCountdown(CountDown::SECONDS);

TinyGPSPlus tinyGps;
SoftwareSerial gps(4, 5);
double LOCKED_LAT = 0.0, LOCKED_LON = 0.0;

Adafruit_MPU6050 mpu;

bool locked = false;
bool notifiedAccident = false;

SoftwareSerial sim(6, 7);
String riderNumber = "+639168947532";
String emergencyNumber = "+639162008019";
// ! String riderNumber = "+639275734769";
// ! String emergencyNumber = "+639275734769";

void setup()
{
  while (!Serial)
  {
  }

  Serial.begin(115200);

  HC12.begin();
  relay.off();
  delay(2000);

  gps.begin(9600);
  delay(2000);
  // ! gps.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  gps.println(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);
  // ! gps.println(PMTK_Q_RELEASE);

  sim.begin(9600);

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  noTone(10);

  HC12.iHC12.listen();
}

void loop()
{
  HC12.read();

  if (HC12.dataIsReady())
  {
    if (HC12.checksumOk())
    {
      strncpy(data, HC12.getRcvData(), messageLength);
      data[messageLength] = '\0';
      trimWhitespace(data);

      // ! Serial.println(data);

      if (strcmp(data, "riderReady") == 0)
      {
        relay.on();
        countdown.stop();
        crashCountdown.stop();
        notifiedAccident = false;
      }
      else if (strcmp(data, "reset") == 0)
      {
        notifiedAccident = false;
      }
      else if (strcmp(data, "lock") == 0)
      {
        gps.listen();
        delay(3000);

      readLockGps:
        while (gps.available() > 0)
        {
          if (tinyGps.encode(gps.read()))
          {
            if (tinyGps.location.isValid())
            {
              gps.stopListening();
              HC12.iHC12.listen();
              goto lockProcedure;
            }
          }
        }
        goto readLockGps;

      lockProcedure:
        LOCKED_LAT = tinyGps.location.lat();
        LOCKED_LON = tinyGps.location.lng();

        // ! Serial.print(LOCKED_LAT);
        // ! Serial.print(", ");
        // ! Serial.println(LOCKED_LON);

        relay.off();
        countdown.stop();
        crashCountdown.stop();
        locationCheckCountdown.start(locationCheckSec);
        locked = true;
      }
      else if (strcmp(data, "unlock") == 0)
      {
        locationCheckCountdown.stop();
        locked = false;
      }
      else
      {
        countdown.start(engineShutoffSec);
      }
    }

    HC12.setReadyToReceive();
  }

  if (locked)
  {
    if (locationCheckCountdown.remaining() < 1)
    {
      gps.listen();
      delay(3000);

    readCurrentLockedGps:
      while (gps.available() > 0)
      {
        if (tinyGps.encode(gps.read()))
        {
          if (tinyGps.location.isValid())
          {
            gps.stopListening();
            HC12.iHC12.listen();
            goto lockedProcedure;
          }
        }
      }
      goto readCurrentLockedGps;

    lockedProcedure:
      unsigned long kmFromCurrentToLockedLocation =
          (unsigned long)TinyGPSPlus::distanceBetween(
              tinyGps.location.lat(),
              tinyGps.location.lng(),
              LOCKED_LAT,
              LOCKED_LON) /
          1000;

      // ! Serial.print(kmFromCurrentToLockedLocation);
      // ! Serial.print(": ");
      Serial.print(LOCKED_LAT);
      Serial.print(", ");
      Serial.println(LOCKED_LON);

      if (kmFromCurrentToLockedLocation > 0.1)
      {
        // todo https://maps.google.com/?q=
        String vehicleTheftMessage = "Vehicle Theft! see Google Map coordinates: " + String(tinyGps.location.lat(), 3) + "," + String(tinyGps.location.lng(), 3);

        sendGSMMessage(vehicleTheftMessage, riderNumber);
        delay(2000);
      }

      locationCheckCountdown.stop();
      locationCheckCountdown.start(1);
    }

    // ! Serial.println("ping");
    sendHC12("ping");
    delay(1000);
  }
  else
  {
    float xOrientation = getXOrientation();

    if (!crashCountdown.isRunning() && isAccident(xOrientation))
    {
      crashCountdown.start(crashAlertSec);
    }

    if (crashCountdown.isRunning() && strcmp(data, "riderReady") == 0)
    {
      if (crashCountdown.remaining() > 1)
      {
        beep();
      }
      else
      {
        xOrientation = getXOrientation();

        Serial.println(xOrientation);

        if (!notifiedAccident && isAccident(xOrientation))
        {
          gps.listen();
          delay(3000);

        readAccidentGps:
          while (gps.available() > 0)
          {
            if (tinyGps.encode(gps.read()))
            {
              if (tinyGps.location.isValid())
              {
                goto sendAccidentAlert;
              }
            }
          }
          goto readAccidentGps;

        sendAccidentAlert:
          // todo https://maps.google.com/?q=
          String accidentMessage = "Accident! see Google Map coordinates: " + String(tinyGps.location.lat(), 3) + "," + String(tinyGps.location.lng(), 3);

          sendGSMMessage(accidentMessage, emergencyNumber);
          delay(2000);

          notifiedAccident = true;
          gps.stopListening();
          HC12.iHC12.listen();
        }

        relay.off();
        crashCountdown.stop();
      }
    }

    if (countdown.isRunning() && relay.isRelayOn())
    {
      int remainingTime = countdown.remaining();
      String elapsed = String(remainingTime);

      while (elapsed.length() < 2)
      {
        elapsed = "0" + elapsed;
      }

      beep();

      if (countdown.remaining() < 1)
      {
        relay.off();
      }
    }
  }
}

bool isAccident(float xOrientation)
{
  return xOrientation > 8 || xOrientation < -8;
}

void trimWhitespace(char *str)
{
  char *start = str;
  while (isspace(*start))
  {
    start++;
  }

  char *end = start + strlen(start) - 1;
  while (end > start && isspace(*end))
  {
    end--;
  }

  *(end + 1) = '\0';

  if (start != str)
  {
    memmove(str, start, end - start + 2);
  }
}

void beep()
{
  const int buzzerPin = 10;
  tone(buzzerPin, 1000);
  delay(200);
  noTone(buzzerPin);
  delay(200);
  tone(buzzerPin, 1000);
  delay(200);
  noTone(buzzerPin);
}

float getXOrientation()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  return a.acceleration.x;
}

void sendHC12(String mess)
{
  mess.toCharArray(message, messageLength);

  if (HC12.isReadyToSend())
  {
    HC12.print(message);
  }
}

void sendGSMMessage(String message, String number)
{
  // ! Serial.print(number);
  // ! Serial.print(": ");
  // ! Serial.println(message);

  sim.println("AT+CMGF=1");
  delay(200);
  sim.println("AT+CMGS=\"" + number + "\"\r");
  delay(200);
  sim.println(message);
  delay(200);
  sim.println((char)26);
}