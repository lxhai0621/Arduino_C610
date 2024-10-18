#ifndef AngleController_CAN_h
#define AngleController_CAN_h
#include <mcp2515.h>
#include <SPI.h>
#include "Arduino.h"
#include <PID_v1.h>//PID控制库 


class AngleController
{
protected:
    int _currentSpeed;
    int _maxAbsAngle, _minAbsAngle;
public:
    AngleController(unsigned int minAbsAngle,unsigned int maxAbsAngle);
    void init_CAN();    //初始化CAN
    short int  getAngleA();
    short int  getAngleB();
    int getSpeedC();
    int getSpeedD();
    void Motor5(int angle);
    void Motor6(int angle);
    void Motor1(int speed);
    void Motor2(int speed);
    void One(int rpm1);
    void Two(int rpm2);
    void Five(int rpm5); 
    void Six(int rpm6); 
    void Move(int speed1,int speed2);
    void init_mpid();
};

#endif
