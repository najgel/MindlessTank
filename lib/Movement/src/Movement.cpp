#include "Movement.h"

Movement::Movement(uint8_t pinEnable1, uint8_t pinIN1, uint8_t pinIN2, uint8_t pinEnable2, uint8_t pinIN3, uint8_t pinIN4):
_leftMotor (pinEnable1, pinIN1, pinIN2), _rightMotor (pinEnable2, pinIN3, pinIN4)
{
    _leftMotorCompensation = 0;
    _rightMotorCompensation = 0;
}

 void Movement::Forward(int speed){
     _leftMotor.setSpeed(speed + _leftMotorCompensation);
    _rightMotor.setSpeed(speed + _rightMotorCompensation);

    _leftMotor.forward();
    _rightMotor.forward();
 }

void Movement::Backward(int speed){
     _leftMotor.setSpeed(speed + _leftMotorCompensation);
    _rightMotor.setSpeed(speed + _rightMotorCompensation);

    _leftMotor.backward();
    _rightMotor.backward();
 }

void Movement::Stop(){
    _leftMotor.stop();
    _rightMotor.stop();
}

void Movement::PivotLeft(){
    int speed = 100;
    _leftMotor.backward();
    _rightMotor.forward();
}

void Movement::PivotRight(){
    int speed = 100;
    _leftMotor.forward();
    _rightMotor.backward();
}