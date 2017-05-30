#include "ProximitySensor.h"
#include <Arduino.h>

#define uS_TO_M SOUND_SPEED/2000000.0

unsigned long lastPoll;
volatile unsigned long lastDistanceSet;
volatile float distance;

/**
    Interrupt pour mettre à jour la distance
**/
void interrupt(void)
{
    unsigned long result = micros() - lastPoll;
    distance = result * uS_TO_M;
    lastDistanceSet = micros();
}


// Object starts here
ProximitySensor::ProximitySensor(char trigger, char echo, long wait)
{
    this->triggerPin = trigger;
    this->echoPin = echo;
    this->wait = wait;
    distance = 0;
    lastPoll = 0;
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT_PULLUP);
    attachInterrupt(echo-2, interrupt, FALLING);
}

/**
    Appel non-bloquant.
    @return La distance relevée par le capteur ultrason ou -1 si OoR
**/
float ProximitySensor::getDistance()
{
    return micros() - lastDistanceSet < this->wait*1000 + 1000 ? distance : distance;
}

/**
    Envoie une demande de ping au capteur à ultrasons.
    Appel non-bloquant.
**/
void ProximitySensor::poll(void)
{
    digitalWrite(this->triggerPin, 1);
    delayMicroseconds(DELAY_uS);
    lastPoll = micros();
    digitalWrite(this->triggerPin, 0);
}

