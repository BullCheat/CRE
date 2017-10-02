#include <Motor.h>
#include <Arduino.h>


Motor::Motor(char ena, char in1, char in2)
{
    this->ena = ena;
    this->in1 = in1;
    this->in2 = in2;
    this->speed = 0;
    this->power = false;
    this->reverse = false;
    pinMode(ena, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    this->update();
}

/**
    Avance à la vitesse 0 ≤ speed < 256
**/
void Motor::forward(int speed)
{
    this->power = true;
    this->reverse = speed < 0;
    this->speed = speed < 0 ? -speed : speed;
    if (this->speed > 255) this->speed = 255;
    this->update();
}

/**
    Passe en roue-libre
**/
void Motor::free()
{
    this->speed = this->reverse = this->power = 0;
    this->update();
}

int Motor::getSpeed()
{
    int i = this->speed;
    if (i < 0) i+= 256;
    return i;
}

/**
    Recule à la vitesse 0 ≤ speed < 256
**/

void Motor::backward(int speed)
{
    this->forward(-speed);
}

void Motor::stop()
{
    this->forward(0);
}

/**
    Le moteur est-il en roue-libre ?
**/
bool Motor::isFree()
{
    return !this->power;
}

/**
    Met à jour la « transmission » vers le moteur
**/
void Motor::update()
{
    digitalWrite(ena, this->power);
    analogWrite(in1, !this->reverse ? speed : 0);
    analogWrite(in2,  this->reverse ? speed : 0);
}

void Motor::step(int step)
{
    if (this->reverse) return;
    if (speed + step < 0) step = speed;
    this->forward(speed + step);
    update();
}
