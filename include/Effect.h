#ifndef EFFECT_H
#define EFFECT_H

#include "Arduino.h"

#define MAX_NR_OF_COPIES  20
#define VAL_NOT_SET -12345


struct df {
    volatile float x;
    volatile float y;
};

struct midi_cc {
    volatile int channel;
    volatile int message_type;
    volatile int data1;
};

struct db {
    volatile bool x;
    volatile bool y;
    volatile float middleline_y;
};

struct dir {
    volatile bool cw;
    volatile bool ccw;
};


enum Effect_type {
    INDEPENDENT  = 0, 
    DEPENDENT    = 1,
    NOTSET       = -12345 
};

enum Control_type {
    POSITION        = 3, 
    TORQUE          = 2,
    VELOCITY        = 1,
    VELOCITY_ANGLE  = 4,
    MIDI            = 5,
    UNDEFINED       = -12345   //not yet known or invalid state
};

struct EffectConfig_s {
    uint16_t data_size;
    float angle;
    float quality;
    Effect_type effect_type;
    Control_type control_type;
};




class Effect {
  public:

    int8_t isActive(float angle_rad, int8_t cw, int range, int time);

    float getArrayPointerValue(float position, float angle_deg, uint16_t range);
    float getEffectVoltage(float value);  

    uint16_t getArrayPointerValueVelocityEffect(long ms);
    float getVelocityOverTime(float value);
    float getAngleOverTime(float value);

    
    volatile uint16_t data_size;
    volatile bool infinite;
    volatile float position;
    df scale;
    db flip;
    midi_cc midi_config;
    
    Control_type control_type;
    volatile uint16_t data_ptr;

    volatile float angle;
    Effect_type effect_type; 
    dir direction;
    volatile float copy[MAX_NR_OF_COPIES];
    volatile uint8_t copy_count;
    volatile float quality;
    volatile float start_angle;    
    volatile long start_time;

};



class FeelixEffect : public Effect {

    public:
        bool isHapticEffectActive(float angle_deg, float position, int8_t cw, int range);
        bool isVelocityEffectActive(float angle_deg, long time);

        FeelixEffect(EffectConfig_s effect, float d[]);

        float getValueAtPointer(float value, uint8_t step, uint8_t offset);
        uint16_t getPointerValueVelocityEffect(long ms);
        void start();
        void stop();
        
        
        float* data[];
        
};






#endif