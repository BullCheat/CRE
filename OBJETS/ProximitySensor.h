#ifndef PROXIMITYSENSOR_H
#define PROXIMITYSENSOR_H

#define SOUND_SPEED 340 // En mm/ms à 290K sous 1 bar
#define DELAY_uS 10

class ProximitySensor
{
    public:
        ProximitySensor(char trigger, char echo, long wait);
        /** maxDst en mm, renvoie la distance ou 0 */
        void poll();
        long getDistance();
    protected:
    private:
        long wait;
        char triggerPin;
        char echoPin;
};

#endif // PROXIMITYSENSOR_H
