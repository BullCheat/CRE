class Motor
{
protected:
    char in1;
    char in2;
    char ena;
    char speed;
    bool power;
    bool reverse;
    void update();
public:
    Motor(char ena, char in1, char in2);
    void forward(int speed);
    void backward(int speed);
    void stop();
    void free();
    bool isFree();
};
