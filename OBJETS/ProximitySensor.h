#ifndef PROXIMITYSENSOR_H
#define PROXIMITYSENSOR_H

#define SOUND_SPEED 340.0 // En m/ms à 290K sous 1 bar
#define DELAY_uS 10

class ProximitySensor
{
    public:
        ProximitySensor(char trigger, char echo, long wait);
        void poll();
        float getDistance();
    protected:
    private:
        long wait;
        char triggerPin;
        char echoPin;
};

#endif // PROXIMITYSENSOR_H
