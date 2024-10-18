#include "AngleController_C610_speed_CAN.h"
#include "Arduino.h"
//电机速度PID

double setpointm1 = 0;
double inputm1 = 0, outputm1 = 0;

double mKp1 = 2.5;
double mKi1 = 1;
double mKd1 = 0.0;

double setpointm2 = 0;
double inputm2 = 0, outputm2 = 0;

double mKp2 = 2.5;
double mKi2 = 1;
double mKd2 = 0.0;

double setpointm3 = 0;
double inputm3 = 0,outputm3 = 0;

double mKp3 = 5;
double mKi3 = 5;
double mKd3 = 0.01;

double setpointm4 = 0;
double inputm4 = 0,outputm4 = 0;

double mKp4 = 5;
double mKi4 = 5;
double mKd4 = 0.01;

PID mpid1(&inputm1, &outputm1, &setpointm1, mKp1, mKi1, mKd1, DIRECT);
PID mpid2(&inputm2, &outputm2, &setpointm2, mKp2, mKi2, mKd2, DIRECT);
PID mpid3(&inputm3, &outputm3, &setpointm3, mKp3, mKi3, mKd3, DIRECT);
PID mpid4(&inputm4, &outputm4, &setpointm4, mKp4, mKi4, mKd4, DIRECT);
struct can_frame rxCanMsg;
MCP2515 mcp2515(53);   // Set CS to pin 53
struct can_frame canMsg;


AngleController::AngleController(unsigned int minAbsAngle,unsigned int maxAbsAngle){
    _minAbsAngle = minAbsAngle;
    _maxAbsAngle = maxAbsAngle;
}

void AngleController::init_CAN(){
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS,MCP_8MHZ);
    mcp2515.setNormalMode();
    mpid1.SetMode(AUTOMATIC);
    mpid1.SetSampleTime(10);
    mpid1.SetOutputLimits(-10000,10000);
    mpid2.SetMode(AUTOMATIC);
    mpid2.SetSampleTime(10);
    mpid2.SetOutputLimits(-10000,10000);
    mpid3.SetMode(AUTOMATIC);
    mpid3.SetSampleTime(10);
    mpid3.SetOutputLimits(-16384,16384);
    mpid4.SetMode(AUTOMATIC);
    mpid4.SetSampleTime(10);
    mpid4.SetOutputLimits(-16384,16384); 

}
short int AngleController::getAngleA(){
    short int angle;
    mcp2515.readMessage(&rxCanMsg);
    if(rxCanMsg.can_id==0x205){
       angle=rxCanMsg.data[3]+(rxCanMsg.data[2]*256);
    }
    return angle; 
}

short int AngleController::getAngleB(){
    short int angle;
    mcp2515.readMessage(&rxCanMsg);
    if(rxCanMsg.can_id==0x206){
       angle=rxCanMsg.data[3]+(rxCanMsg.data[2]*256);
    }
    return angle; 
}

int AngleController::getSpeedC(){
    short int speed;
    mcp2515.readMessage(&rxCanMsg);
    if(rxCanMsg.can_id==0x201){
       speed=rxCanMsg.data[3]+(rxCanMsg.data[2]*256);
    }
    return speed; 
}

int AngleController::getSpeedD(){
    short int speed;
    mcp2515.readMessage(&rxCanMsg);
    if(rxCanMsg.can_id==0x202){
       speed=rxCanMsg.data[3]+(rxCanMsg.data[2]*256);
    }
    return speed; 
}
void AngleController::Five(int rpm5){
    short int drpm5;
    drpm5 =(short int)rpm5;
    canMsg.can_id  = 0x1FF;
    canMsg.can_dlc = 8;
    canMsg.data[0] = drpm5>>8;
    canMsg.data[1] = drpm5&0x00ff;
    mcp2515.sendMessage(&canMsg);
}
void AngleController::Six(int rpm6){
    short int drpm6;
    drpm6 =(short int)rpm6;
    canMsg.can_id  = 0x1FF;
    canMsg.can_dlc = 8;
    canMsg.data[2] = drpm6>>8;
    canMsg.data[3] = drpm6&0x00ff;
    mcp2515.sendMessage(&canMsg);
}
void AngleController::One(int rpm_1){
    short int drpm_1;
    drpm_1 =(short int)rpm_1;
    canMsg.can_id  = 0x200;
    canMsg.can_dlc = 8;
    canMsg.data[0] = drpm_1>>8;
    canMsg.data[1] = drpm_1&0x00ff;
    mcp2515.sendMessage(&canMsg);
}
void AngleController::Two(int rpm_2){
    short int drpm_2;
    drpm_2 =(short int)rpm_2;
    canMsg.can_id  = 0x200;
    canMsg.can_dlc = 8;
    canMsg.data[2] = drpm_2>>8;
    canMsg.data[3] = drpm_2&0x00ff;
    mcp2515.sendMessage(&canMsg);
}
void AngleController::Motor5(int angle){
    
    if (angle < 0)
    {
        angle = min(angle, -_minAbsAngle);
        angle = max(angle, -_maxAbsAngle);
    }
    else
    {
        angle = max(angle, _minAbsAngle);
        angle = min(angle, _maxAbsAngle);
    }
    mpid1.Compute();
    setpointm1 = angle;
    inputm1 = getAngleA();
    Five(outputm1);
}
void AngleController::Motor6(int angle){
    
    if (angle < 0)
    {
        angle = min(angle, -_minAbsAngle);
        angle = max(angle, -_maxAbsAngle);
    }
    else
    {
        angle = max(angle, _minAbsAngle);
        angle = min(angle, _maxAbsAngle);
    }
    mpid2.Compute();
    setpointm2 = angle;
    inputm2 = getAngleB();
    Six(outputm2);
}
void AngleController::Motor1(int speed){
    
    if (speed < 0)
    {
        speed = min(speed, -_minAbsAngle);//限制速度、最低minAbsSpeed
        speed = max(speed, -_maxAbsAngle);  //限制速度、最高maxAbsSpeed
    }
    else
    {
        speed = max(speed, _minAbsAngle);
        speed = min(speed, _maxAbsAngle);
    }
    mpid3.Compute();
    setpointm3 = speed;
    inputm3 = getSpeedC();
    One(outputm3);
}
void AngleController::Motor2(int speed){

    if (speed < 0)
    {
        speed = min(speed, -_minAbsAngle);//限制速度、最低minAbsSpeed
        speed = max(speed, -_maxAbsAngle);  //限制速度、最高maxAbsSpeed
    }
    else
    {
        speed = max(speed, _minAbsAngle);
        speed = min(speed, _maxAbsAngle);
    }
    mpid4.Compute();
    setpointm4 = speed;
    inputm4 = getSpeedD();
    Two(outputm4);
}
void AngleController::Move(int speed1,int speed2)
{
	if (speed1 < 0)
    {
        speed1 = min(speed1, -_minAbsAngle);//限制速度、最低minAbsSpeed
        speed1 = max(speed1, -_maxAbsAngle);  //限制速度、最高maxAbsSpeed
    }
    else
    {
        speed1 = max(speed1, _minAbsAngle);
        speed1 = min(speed1, _maxAbsAngle);
    }
    if (speed2 < 0)
    {
        speed2 = min(speed2, -_minAbsAngle);//限制速度、最低minAbsSpeed
        speed2 = max(speed2, -_maxAbsAngle);  //限制速度、最高maxAbsSpeed
    }
    else
    {
        speed2 = max(speed2, _minAbsAngle);
        speed2 = min(speed2, _maxAbsAngle);
    }
    mpid3.Compute();
    setpointm3 = speed1;
    inputm3 = getSpeedC();
    mpid4.Compute();
    setpointm4 = speed2;
    inputm4 = getSpeedD();
    short int drpm_1,drpm_2;
    drpm_1 =(short int)outputm3;
    drpm_2 =(short int)outputm4;
    canMsg.can_id  = 0x200;
    canMsg.can_dlc = 8;
    canMsg.data[0] = drpm_1>>8;
    canMsg.data[1] = drpm_1&0x00ff;
    canMsg.data[2] = drpm_2>>8;
    canMsg.data[3] = drpm_2&0x00ff;
    mcp2515.sendMessage(&canMsg);    
}
