#include "Arduino.h"
#include "L298N.h"

class Movement{
    public:
        typedef enum 
        {
            LEFT = 0,
            RIGHT = 1
        } Motor;


        Movement(uint8_t pinEnable1, uint8_t pinIN1, uint8_t pinIN2, uint8_t pinEnable2, uint8_t pinIN3, uint8_t pinIN4);
        //void setMotorCompensation(int motor, int compensation);
        void Forward(int speed);
        void Backward(int speed);
        void Stop();
        void PivotLeft();
        void PivotRight();
    
    private:
        L298N _leftMotor;
        L298N _rightMotor;
        int _leftMotorCompensation;
        int _rightMotorCompensation;


};