/*! ===========================================================================================================================
  Project: CRE (Callo Run Energy)
  Target used = Arduino Uno R3
  Software version : v6
  Editor: Alexandre DRÉAN
//============================================================================================================================*/



/*! ===========================================================================================================================
//                         Directives
// ===========================================================================================================================*/


#include <Arduino.h>
#include <Led.h>
#include <Motor.h>
#include <ProximitySensor.h>
#include <FlexiTimer2.h>
using namespace std;

// Leds TODO FIXME
/// À configurer
#define GREEN_PIN  12 // Led verte
#define RED_PIN    13 // Led rouge
#define BLUE_PIN   11 // Led bleue
#define YELLOW_PIN 10 // Led jaune

#define SPEED 115200 // Vitesse UART
#define BLUETOOTH_BUFFER 16 // Taille maximale des messages bluetooth

// Motor
/// À configurer
#define ENA_PIN 4 // Pin ENA driver moteur
#define IN1_PIN 5 // Pin IN1 driver moteur
#define IN2_PIN 6 // Pin IN2 driver moteur

#define PROXIMITY_INTERVAL 10 // Intervalle de vérification de la proximité (en ms)
#define PRX_TRG_PIN 7 // Pin TRIG HC-SR04
#define PRX_ECH_PIN 2 // Pin ECHO HC-SR04

#define F_U_PIN A0 // Pin de la carte fréquence/tension
#define TEETH_COUNT 5 // Nombre de dents sur la roue permettant de mesurer la vitesse
#define DISTANCE_PER_ROTATION 0.062*3.141591 // Développé de la roue
#define U_ORIGIN 0 // b de ax+b du calcul de la tension
#define U_PER_HZ 13.2f // Nombre d'unités de tension arduino par hz de la carte F/U

#define PIN_ISR_DISTANCE 3 // Pin auquel est branché le capteur fourche pour mesurer la distance parcourue

/*! ===========================================================================================================================
//                         Objets et variables globales
// ===========================================================================================================================*/

Led greenLed    = Led(GREEN_PIN , false);
Led redLed      = Led(RED_PIN   , false);
Led blueLed     = Led(BLUE_PIN  , false);
Led yellowLed   = Led(YELLOW_PIN, false);
volatile unsigned int dst; /// FIXME int ou long ?
char bBuffer[BLUETOOTH_BUFFER]; // buffer bluetooth

Motor motor = Motor(
                    ENA_PIN,
                    IN1_PIN,
                    IN2_PIN
                    );
ProximitySensor proximitySensor = ProximitySensor(PRX_TRG_PIN, PRX_ECH_PIN, PROXIMITY_INTERVAL);

/* ISR */

/**
    ISR incrémentant la distance
**/
void _isrDistance(void) {
    dst++;
}

/**
    ISR envoyant un ultrason pour la mesure de proximité
**/
void _pollProximitySensor(void) { // ISR pour envoyer un ultrason
    proximitySensor.poll();
}

/* Fonctions */

/**
    @param ascii Chaîne ascii contenant le nombre à parse
    @param offset Où commencer dans la chaîne ascii ?
    @default ascii 0
    @param end Fin du nombre dans la chaîne ascii
    @default end BLUETOOTH_BUFFER
**/
long int parseInt(char ascii[], char offset, char end) {
    long int x = 0;
    char m = 1;
    for (int i = offset; i < end; i++) {
        char c = ascii[i];
        if (c >= 0x30 && c <= 0x39) {
            x*=10;
            x += c - 0x30;
        } else if (c == '+') {
        } else if (c == '-') {
            m = -m;
        }

    }
    return x * m;
}

/**
    @return distance parcourue depuis le début en mètre
**/
float getDistance(void) {
    return dst / (float) TEETH_COUNT * DISTANCE_PER_ROTATION;
}
/**
    @return vitesse en m/s
**/
float getInstantSpeed(void) {
    int poll = analogRead(F_U_PIN); // On lit la valeur
    poll -= U_ORIGIN; // On retire le "b" de ax+b
    float freq = poll / (float) U_PER_HZ; // On divise pour obtenir la fréquence
    freq /= (float) TEETH_COUNT; // On divise par le nombre de dents pour avoir la fréquence réelle
    float speed = freq * (float) DISTANCE_PER_ROTATION; // On multiplie pour obtenir la distance roulée en 1s
    return speed;
}

/* Programme principal */

/**
    Setup UART
**/
void setup(void)
 {
    Serial.begin(SPEED);
    pinMode(F_U_PIN, INPUT); // On met la carte fréquence/tension en lecture
    FlexiTimer2::set(PROXIMITY_INTERVAL, 0.001, _pollProximitySensor);
    FlexiTimer2::start(); // Utilise INT0 - Désactivé pour pouvoir calculer la distance
    pinMode(PIN_ISR_DISTANCE, INPUT_PULLUP);
    attachInterrupt(PIN_ISR_DISTANCE-2, _isrDistance, RISING);
 }



///
/// Réception des commandes ICI
///
void handleSerial(void) {
    if (bBuffer[0] == 'T') {
        int speed = parseInt(bBuffer, 1, BLUETOOTH_BUFFER);
        motor.forward(speed);
    } else if (bBuffer[0] == 'F') {
        motor.free();
    } else { // Ping ou erreur
        Serial.print(bBuffer[0]);
    }
}


/// Gestion des commandes, ne pas éditer
void serialEvent(void) {
    static char bLenTmp[3];
    static char bLen = 0;
    static char bState = 0;
    static char bIndex = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (bState == 0 || bState == 1) { // On est en cours de lecture du début de la trame

            if (bState == 1 && c == '\r') { // Chiffres fini
                bLen = 0;
                for (int i = bIndex - 1; i >= 0; i--) {
                    bLen *= 10;
                    bLen += bLenTmp[i];
                }
                memset(&bLenTmp[0], 0, 3);
                Serial.print(bLen);
            } else if (bState == 1 && c == '\n') { // On a fini l'en-tête, on passe en mode lecture des données dans le buffer
                bState = 2; // État à 2
                bIndex = 0; // Indexes à 0
                memset(&bBuffer[0], 0, BLUETOOTH_BUFFER); // On vide le tableau
            } else if (c >= 0x30 && c < 0x3A) { // On lit la longueur
                bState = 1;
                bLenTmp[bIndex++] = c  - 0x30;
            }

        } else if (bState == 2) {
            bBuffer[bIndex++] = c;
            if (bIndex == bLen) {
                handleSerial();
                bIndex = bLen = bState = 0;
                memset(&bBuffer[0], 0, BLUETOOTH_BUFFER); // On vide le tableau
            }
        }
    }
}



/*! ===========================================================================================================================
//                          programme principal
// ===========================================================================================================================*/

void loop(void)
{
        long distance = proximitySensor.getDistance();
        if (distance > 200 && distance < 300) {
            redLed.on();
        } else redLed.off();

}
