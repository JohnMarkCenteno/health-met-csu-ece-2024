#include "simpleHC12.h"
#include <SoftwareSerial.h>
#include <CountDown.h>
#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

const unsigned int forceSensorAPin = A0;
const unsigned int forceSensorBPin = A1;
const unsigned int alcoholSensorPin = A2;

const unsigned int ledPin = 9;
const unsigned int buttonPin = 8;

const unsigned int hc12RxPin = 3;
const unsigned int hc12TxPin = 2;
const unsigned int hc12SetPin = 1;

const unsigned long baudRate = 9600;
const unsigned messageLength = 20;
boolean useChecksum = true;
char data[messageLength + 1];
char message[messageLength];
char lastMessage[messageLength];

simpleHC12 HC12(hc12TxPin, hc12RxPin, hc12SetPin, baudRate, messageLength, useChecksum);

// ? Alcohol Sensor smoothing parameters
const int numSamples = 10;
int alcoholReadings[numSamples];
int readIndex = 0;
int total = 0;
int averageAsVal = 0;
const int drunkThreshold = 450;

const unsigned int pingTimeoutSec = 15;
CountDown pingCountdown(CountDown::SECONDS);

bool locked = false;
bool notifiedDrunk = false;

TinyGPSPlus tinyGps;
SoftwareSerial gps(4, 5);

SoftwareSerial sim(6, 7);
String riderNumber = "+639168947532";
String emergencyNumber = "+639162008019";
// ! String riderNumber = "+639275734769";
// ! String emergencyNumber = "+639275734769";

void setup()
{
  Serial.begin(9600);
  HC12.begin();
  delay(2000);

  gps.begin(9600);
  delay(2000);
  // ! gps.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  gps.println(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);
  // ! gps.println(PMTK_Q_RELEASE);

  sim.begin(9600);

  pinMode(forceSensorAPin, INPUT);
  pinMode(forceSensorBPin, INPUT);
  pinMode(alcoholSensorPin, INPUT);

  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  for (int i = 0; i < numSamples; i++)
  {
    alcoholReadings[i] = 0;
  }

  HC12.iHC12.listen();
}

void loop()
{
  if (isButtonPressed())
  {
    if (locked)
    {
      locked = false;

      for (int i = 0; i < 5; i++)
      {
        sendHC12("unlock");
        delay(1000);
      }

      pingCountdown.stop();
      digitalWrite(ledPin, LOW);
    }
    else
    {
      if (isHelmetOn())
      {
        sendHC12("reset");
        notifiedDrunk = false;
        delay(1500);
      }
      else
      {
        locked = true;
        sendHC12("lock");
        delay(500);

        HC12.iHC12.listen();
        pingCountdown.start(pingTimeoutSec);
        digitalWrite(ledPin, HIGH);
      }
    }
  }

  if (locked)
  {
    HC12.read();

    if (HC12.dataIsReady())
    {
      if (HC12.checksumOk())
      {
        strncpy(data, HC12.getRcvData(), messageLength);
        data[messageLength] = '\0';
        trimWhitespace(data);

        if (strcmp(data, "ping") == 0)
        {
          pingCountdown.restart();
          digitalWrite(ledPin, HIGH);
        }
      }

      HC12.setReadyToReceive();
    }

    if (pingCountdown.remaining() < 1)
    {
      gps.listen();
      delay(3000);

    readHelmetTheftGps:
      while (gps.available() > 0)
      {
        if (tinyGps.encode(gps.read()))
        {
          if (tinyGps.location.isValid())
          {
            goto sendHelmetTheftAlert;
          }
        }
      }
      goto readHelmetTheftGps;

    sendHelmetTheftAlert:
      // todo https://maps.google.com/?q=
      String helmetTheftMessage = "Helmet Theft! see Google Map coordinates: " + String(tinyGps.location.lat(), 3) + "," + String(tinyGps.location.lng(), 3);

      sendGSMMessage(helmetTheftMessage, riderNumber);
      delay(2000);

      pingCountdown.start(pingTimeoutSec);
      gps.stopListening();
      HC12.iHC12.listen();
    }
    blinkLed();
  }
  else
  {
    if (isHelmetOn())
    {
      if (isDrunk())
      {
        sendHC12("riderDrunk");

        if (!notifiedDrunk)
        {
          gps.listen();
          delay(3000);

        readRiderDrunkGps:
          while (gps.available() > 0)
          {
            if (tinyGps.encode(gps.read()))
            {
              if (tinyGps.location.isValid())
              {
                goto sendRiderDrunkAlert;
              }
            }
          }
          goto readRiderDrunkGps;

        sendRiderDrunkAlert:
          // todo https://maps.google.com/?q=
          String drunkAlertMessage = "Rider is drunk! see Google Map coordinates: " + String(tinyGps.location.lat(), 3) + "," + String(tinyGps.location.lng(), 3);

          sendGSMMessage(drunkAlertMessage, emergencyNumber);
          delay(2000);

          notifiedDrunk = true;
        }
      }
      else
      {
        sendHC12("riderReady");
      }
    }
    else
    {
      sendHC12("riderNotReady");
    }
  }

  delay(250);
}

bool isButtonPressed()
{
  return digitalRead(buttonPin) == LOW;
}

bool isHelmetOn()
{
  int fsAVal = analogRead(forceSensorAPin);
  int fsBVal = analogRead(forceSensorBPin);

  return fsAVal > 30 && fsBVal > 30;
}

bool isDrunk()
{
  int asVal = analogRead(alcoholSensorPin);
  total = total - alcoholReadings[readIndex];

  alcoholReadings[readIndex] = asVal;
  total = total + alcoholReadings[readIndex];

  readIndex = (readIndex + 1) % numSamples;
  averageAsVal = total / numSamples;

  return averageAsVal > drunkThreshold;
}

void sendHC12(String mess)
{
  mess.toCharArray(message, messageLength);

  if (HC12.isReadyToSend() && strcmp(message, lastMessage) != 0)
  {
    mess.toCharArray(lastMessage, messageLength);
    HC12.print(message);
  }
}

void sendGSMMessage(String message, String number)
{
  Serial.print(number);
  Serial.print(": ");
  Serial.println(message);

  sim.println("AT+CMGF=1");
  delay(200);
  sim.println("AT+CMGS=\"" + number + "\"\r");
  delay(200);
  sim.println(message);
  delay(200);
  sim.println((char)26);
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

void blinkLed()
{
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);
}
