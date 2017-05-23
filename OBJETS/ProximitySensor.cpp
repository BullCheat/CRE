#include "ProximitySensor.h"
#include <Arduino.h>

#define mS_TO_CM 1/((2.0*SOUND_SPEED)*100)

unsigned long lastPoll;
volatile unsigned long lastDistanceSet;
volatile unsigned long distance;

/**
    Interrupt pour mettre à jour la distance
**/
void interrupt(void) {
    unsigned long result = micros() - lastPoll;
    distance = result * mS_TO_CM;
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
long ProximitySensor::getDistance() {
    return micros() - lastDistanceSet < this->wait*1000 + 1000 ? distance : -1;
}

/**
    Envoie une demande de ping au capteur à ultrasons.
    Appel non-bloquant.
**/
void ProximitySensor::poll(void) {
    digitalWrite(this->triggerPin, 1);
    delayMicroseconds(DELAY_uS);
    lastPoll = micros();
    digitalWrite(this->triggerPin, 0);
}

