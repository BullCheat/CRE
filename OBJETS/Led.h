class Led {
private:
    char pin;
    bool state;
    bool reversed;
public:
    Led(char broche, bool reversed);
    void on();
    void off();
    char getPin();
    bool getState();
    void setState(bool state);
    void setPin(char pin);
    void setReversed(bool reversed);
    bool isReversed();
    void refresh();
    void blink(int delay);
};
