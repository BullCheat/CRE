#include "ProximitySensor.h"
#include <Arduino.h>

unsigned long lastPoll;
volatile unsigned long lastDistanceSet;
volatile unsigned long distance;

void interrupt(void) {
    unsigned long result = micros() - lastPoll;
    distance = result / (2.0 * SOUND_SPEED) * 100;
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


long ProximitySensor::getDistance() {
    return micros() - lastDistanceSet < this->wait*1000 + 1000 ? distance : -1;
}



void ProximitySensor::poll(void) {
    digitalWrite(this->triggerPin, 1);
    delayMicroseconds(DELAY_uS);
    lastPoll = micros();
    digitalWrite(this->triggerPin, 0);
}

