#include "Feelix.h"

#if defined(__arm__) && defined(CORE_TEENSY) || defined(ESP_H) || defined(__arm__) && defined(__SAM3X8E__) 



void Feelix::init() {

    STATUS_LED = 13;
    pinMode(STATUS_LED, OUTPUT);
        
    sensor->init();     
    bldc->linkSensor(sensor);

    angle = bldc->shaftAngle(); //(180 / 3.14159);
    rotation_dir = Direction::CW;

    driver->init();
    driver->enable();

    bldc->linkDriver(driver);

    bldc->controller = MotionControlType::torque;
    bldc->torque_controller = TorqueControlType::voltage;
    
    bldc->voltage_sensor_align = 5;

    bldc->PID_velocity.P = 0.5;
    bldc->PID_velocity.I = 10.0;
    bldc->PID_velocity.D = 0.0;
    bldc->PID_velocity.output_ramp = 1000;
    bldc->LPF_velocity.Tf = 0.01;

    bldc->P_angle.P = 14.0;
    bldc->P_angle.I = 0.0;
    bldc->P_angle.D = 0.0;
    bldc->P_angle.output_ramp = 10000;

    bldc->velocity_limit = 20.0;
    bldc->voltage_limit = 12.0;
    driver->voltage_limit = 12.0;
    vol_limit = 12.0;

    bldc->init(); 
}

#endif
