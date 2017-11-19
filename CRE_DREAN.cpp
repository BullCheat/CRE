/*! ===========================================================================================================================
  Project: CRE (Callo Run Energy)
  Target used = Arduino Uno R3
  Software version : v6
  Editor: Alexandre DRÉAN
  Description : Programme de contrôle du véhicule CRE.
                Capteur ultrasons via interrupt + FlexiTimer
                Capteur fourche (vitesse) via carte fréquence/tension (analog) + interrupt
//============================================================================================================================*/



/*! ===========================================================================================================================
//                         Directives
// ===========================================================================================================================*/


#include <Arduino.h>
#include <Led.h>
#include <Motor.h>
#include <ProximitySensor.h>
#include <FlexiTimer2.h>
#include <math.h>
using namespace std;

// Leds TODO FIXME
/// À configurer
#define GREEN_PIN  12 // Led verte
#define RED_PIN    13 // Led rouge
#define BLUE_PIN   11 // Led bleue
#define YELLOW_PIN 10 // Led jaune

#define SPEED 115200 // Vitesse UART
#define BLUETOOTH_BUFFER_LENGTH 16 // Taille maximale des messages bluetooth

// Motor
/// À configurer
#define ENA_PIN 8 // Pin ENA driver moteur
#define IN1_PIN 9 // Pin IN1 driver moteur
#define IN2_PIN 10 // Pin IN2 driver moteur

#define PROXIMITY_INTERVAL 4 // Intervalle de vérification de la proximité (en ms)
#define PRX_TRG_PIN 11 // Pin TRIG HC-SR04
#define PRX_ECH_PIN 12 // Pin ECHO HC-SR04

#define F_U_PIN A0 // Pin de la carte fréquence/tension
#define TEETH_COUNT 5 // Nombre de dents sur la roue permettant de mesurer la vitesse
#define DISTANCE_PER_ROTATION 0.06*3.141591 // Développé de la roue

#define PIN_ISR_DISTANCE 3 // Pin auquel est branché le capteur fourche pour mesurer la distance parcourue
#define PORTIQUE_HEIGHT 0.32
#define PORTIQUE_HEIGHT_TOLERANCE 0.05
#define PORTIQUE_DISTANCE_TOLERANCE 3

/*! ===========================================================================================================================
//                         Objets et variables globales
// ===========================================================================================================================*/

Led greenLed    (GREEN_PIN , false);
Led redLed      (RED_PIN   , false);
Led blueLed     (BLUE_PIN  , false);
Led yellowLed   (YELLOW_PIN, false);
volatile unsigned int dst; /// Attention, distance max ~1km
char bBuffer[BLUETOOTH_BUFFER_LENGTH]; // buffer bluetooth
int initialSpeeds[] = {255, 255}; // Vitesses initiales des scénarios
char scenario = -1; // Scénario (-1 par défaut)
char portique = 0; // Portique actuel (0 par défaut)
unsigned long lastWheelPing;
unsigned long prevWheelPing;
unsigned long lastDebugSent = 0;

// Système limitation scenario 2 portique 2
long timeTarget = 10000; // Temps attendu pour passer entre les deux portiques, en ms /// Pas unsigned sinon bug lors de la soustraction
float distanceLimit = 5; // Distance entre les deux portiques
unsigned long startMillis = 0; // Temps de passage au portique
unsigned long startDistance = 0; // Distance lors du passage au portique
unsigned long lastPortiqueMillis = 0;
float lastPortiqueDistance = 0;

Motor motor (ENA_PIN, IN1_PIN, IN2_PIN);	
ProximitySensor proximitySensor (PRX_TRG_PIN, PRX_ECH_PIN, PROXIMITY_INTERVAL);

/* ISR */

/**
    ISR incrémentant la distance
**/
void _isrDistance(void)
{
    dst++;
    prevWheelPing = lastWheelPing;
    lastWheelPing = micros();
}

/**
    ISR envoyant un ultrason pour la mesure de proximité
**/
void _pollProximitySensor(void)   // ISR pour envoyer un ultrason
{
    proximitySensor.poll();
}

/* Fonctions */

/**
    @param ascii Chaîne ascii contenant le nombre à parse
    @param offset Où commencer dans la chaîne ascii ?
    @default ascii 0
    @param end Fin du nombre dans la chaîne ascii
    @default end BLUETOOTH_BUFFER_LENGTH
**/
long int parseInt(char ascii[], char offset, char end)
{
    long int x = 0;
    char m = 1;
    for (int i = offset; i < end; i++)
    {
        char c = ascii[i];
        if (c >= 0x30 && c <= 0x39)
        {
            x*=10;
            x += c - 0x30;
        }
        else if (c == '+')
        {
        }
        else if (c == '-')
        {
            m = -m;
        }

    }
    return x * m;
}

/**
    @return distance parcourue depuis le début en mètre
**/
float getDistance(void)
{
    return (dst / (float) TEETH_COUNT) * DISTANCE_PER_ROTATION;
}
/**
    @return vitesse en m/s
**/
float getInstantSpeed(void)
{
    if (prevWheelPing == 0) return 0;
    float delta = (max(lastWheelPing - prevWheelPing, micros() - lastWheelPing))/(float)1000000; // delta en micros -> delta en sec
    delta *= TEETH_COUNT; // secondes par tour
    float speed = 1 / delta; // tours par seconde
    speed *= DISTANCE_PER_ROTATION; // m/s
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
    prevWheelPing = 0;
    lastWheelPing = micros();
}

/**
    Gestion des événements liés à la détection de portique

**//* MODIFIER les scénarios ICI *//**

    @param forced Si la fonction a été déclenchée par timeout et non par portique
**/
void handlePortique(bool forced)
{
    Serial.print("portique=");
    Serial.print(forced);
    Serial.println(static_cast<int>(portique));
    switch (scenario)
    {
    case 0: // Scénario 0
        switch (portique)
        {
	case 1:
	case 2:
		motor.forward(255);
		break;
        case 3:
            motor.stop();
            break;
        default:
            motor.free();
            break;
        }
        break;
    case 1: // Scénario 1
        switch (portique)
        {
        case 1:
            motor.free();
	    delay(3000);
            motor.forward(200);
	    delay(1000);
            motor.forward(255);
            break;
        case 2:
            motor.forward(255);
            break;
        case 3:
            motor.stop();
            break;
        default:
            motor.free();
            break;
        }
        break;
    default: // Scénario inconnu
        scenario = 0;
        motor.free();
        break;
    }
}

/**
    Gestion des événements liés aux commandes.
**/
/* AJOUTER les commandes ICI */
void handleSerial(void)
{
    switch (bBuffer[0])
    {
    case 'T': // Throttle
        motor.forward(parseInt(bBuffer, 1, BLUETOOTH_BUFFER_LENGTH));
        break;
    case 'F': // Roue libre
        motor.free();
        break;
    case 'S': // Scénario
        scenario = parseInt(bBuffer, 1, 2) - 1;
        if (scenario >= 0) motor.forward(initialSpeeds[scenario]); else motor.stop();
        dst = 0;
        portique = 0;
        break;
    default: // Erreur ou ping
        Serial.print(bBuffer[0]);
    }
}


/* Envoi des commandes vers handleSerial */ /// Ne pas éditer
void serialEvent(void)
{
    static unsigned char bLenTmp[3]; // Buffer de lecture de la longueur de la trame (3 caratères)
    static unsigned char bLen = 0; // Longueur de la trame
    static unsigned char bState = 0; // État (0 = rien, 1 = lecture de la taille, 2 = lecture de la trame)
    static unsigned char bIndex = 0; // Index de lecture de la trame
    while (Serial.available())
    {
        char c = Serial.read();
        if (bState == 0 || bState == 1)   // On est en cours de lecture du début de la trame
        {
            if (bState == 1 && c == '\r')   // Chiffres fini
            {
                bLen = 0;
                for (int i = bIndex - 1; i >= 0; i--)
                {
                    bLen *= 10;
                    bLen += bLenTmp[i];
                }
                memset(&bLenTmp[0], 0, 3);
            }
            else if (bState == 1 && c == '\n')     // On a fini l'en-tête, on passe en mode lecture des données dans le buffer
            {
                bState = 2; // État à 2
                bIndex = 0; // Indexes à 0
                memset(&bBuffer[0], 0, BLUETOOTH_BUFFER_LENGTH); // On vide le tableau
            }
            else if (c >= 0x30 && c < 0x3A)     // On lit la longueur
            {
                bState = 1;
                bLenTmp[bIndex++] = c  - 0x30;
            }

        }
        else if (bState == 2)     // On lit la trame
        {
            bBuffer[bIndex++] = c;
            if (bIndex == bLen)   // On a tout reçu
            {
                handleSerial();
                bIndex = bLen = bState = 0;
                memset(&bBuffer[0], 0, BLUETOOTH_BUFFER_LENGTH); // On vide le tableau
            }
        }
    }
}

void debug(float distance, int scenario, int portique, float speed)
{
    Serial.print("d=");
    Serial.println(distance);
    Serial.print("s=");
    Serial.println(scenario);
    Serial.print("p=");
    Serial.println(portique);
    Serial.print("v=");
    Serial.println(speed);
    Serial.print("m=");
    Serial.println(motor.getSpeed());
    Serial.print("u=");
    Serial.println(proximitySensor.getDistance());
}

/**
    Passe les portiques + appelle handlePortique
**/
void calcPortiques(void)
{
    float distance = getDistance(); // Distance parcourue
    float ceil = proximitySensor.getDistance(); // Distance ultrasons
    /* Gestion passage portique */
    if (ceil > PORTIQUE_HEIGHT - PORTIQUE_HEIGHT_TOLERANCE && ceil < PORTIQUE_HEIGHT + PORTIQUE_HEIGHT_TOLERANCE) // Si l'ultrasons détecte quelque-chose à la distance attendue pour un portique
    {
	    if (distance - lastPortiqueDistance > PORTIQUE_DISTANCE_TOLERANCE) // Si on est dans la zone d'erreur du portique (pas besoin de tester avec + TOLERANCE car on gère le tout plus bas (timeout))
	    {
		// On passe le portique et on appelle la fonction de gestion
		++portique;
		lastPortiqueDistance = distance;
		handlePortique(false);
	    }
    }
}


/*! ===========================================================================================================================
//                          programme principal
// ===========================================================================================================================*/

void loop(void)
{
    // On gère le passage des portiques si on est dans un scénario
    if (scenario != -1) calcPortiques();
    float ceil = proximitySensor.getDistance(); // Distance ultrasons
    /* Gestion passage portique */
    digitalWrite(RED_PIN, (ceil > PORTIQUE_HEIGHT - PORTIQUE_HEIGHT_TOLERANCE && ceil < PORTIQUE_HEIGHT + PORTIQUE_HEIGHT_TOLERANCE));

    unsigned long mic = millis();
    if (mic - lastDebugSent > 250)
    {
        lastDebugSent = mic;
        debug(getDistance(), scenario, portique, getInstantSpeed());
    }

}
