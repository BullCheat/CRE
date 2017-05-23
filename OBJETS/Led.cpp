/*! ===========================================================================================================================
  Classe: Led.cpp                        Fichier d'impémentation
  Editor: Lycée Marcel Callo
  date:02-10-2016
//============================================================================================================================*/

#include <Led.h>
#include <Arduino.h>

/*! ===========================================================================================================================
//                         Directives
// ===========================================================================================================================*/

Led::Led(char broche, bool reversed) {
    this->state = LOW;
    this->pin = broche;
    this->reversed = reversed;
    pinMode(broche, OUTPUT);
    this->refresh();
}

void Led::setPin(char broche) {
    this->pin = broche;
    pinMode(getPin(), OUTPUT);
    this->refresh();
}

char Led::getPin() {
    return this->pin;
}
bool Led::getState() {
    return this->state;
}

void Led::setState(bool state) {
    this->state = state;
    this->refresh();
}

void Led::refresh() {
    digitalWrite(getPin(), (this->getState()) ^ this->isReversed());
}

/* ATTENTION : Appel bloquant */
void Led::blink(int millisecs) {
    this->getState() ? off() : on();
    delay(millisecs);
    this->getState() ? off() : on();
}

void Led::setReversed(bool reversed) {
    this->reversed = reversed;
    this->refresh();
}

bool Led::isReversed() {
    return this->reversed;
}

void Led::on() {
    this->setState(1);
}
void Led::off() {
    this->setState(0);
}


/*! ===========================================================================================================================
//                         Impémentation des méthodes
// ===========================================================================================================================*/
