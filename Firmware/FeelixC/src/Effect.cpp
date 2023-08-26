#include "Effect.h"

#include "Arduino.h"
#include <math.h>       /* fmod */

#define _DEG_PI     0.017453292519
#define _PI_DEG     57.29577951308



FeelixEffect::FeelixEffect(EffectConfig_s effect, float d[]) {
    data_size = effect.data_size;
    angle = effect.angle;
    effect_type = effect.effect_type;
    control_type = effect.control_type;
    quality = (float) effect.quality;

    for (int i = 0; i < data_size; i++) {
        data[i] = &d[i]; 
    }
    
    direction.cw = true;
    direction.ccw = true;
    infinite = false;
    position = 0.0;
    scale.x = 1.0;
    scale.y = 1.0;
    flip.x = false;
    flip.y = false;
    flip.middleline_y = 0.0;
    start_angle = VAL_NOT_SET;
    copy[0] = 0.0;
    copy_count = 1;
}




float FeelixEffect::getValueAtPointer(float value, uint8_t step, uint8_t offset) {
    float dX = value - floor(value);
    int index = floor(value) + offset;

    float value_1 = *data[index];
    float value_2 = *data[index + step];

    return ((value_2 - value_1) * dX) + value_1;
}



static float getInfiniteAngle(float angle, float range) {
    angle = fmod(angle, range); 
    if (angle < 0) {
        angle += range;
    }
    return angle;
}



int8_t Effect::isActive(float angle_deg, int8_t cw, int range, int time) {
    
    if (control_type == Control_type::VELOCITY || control_type == Control_type::VELOCITY_ANGLE) {
        
        if (control_type == Control_type::VELOCITY_ANGLE && start_angle == VAL_NOT_SET) {
            start_angle = angle_deg * _DEG_PI;
        }

        if (infinite) { return 0; }

        if (time >= (int) copy[0] && time < (int) (copy[0] + angle)) {
            return 0;
        } else {
            start_angle = VAL_NOT_SET;
            return -1;
        }

    } else {
        
        if (control_type == Control_type::MIDI) return 0;
        
        if ((cw == 1 && !direction.cw) || (cw == -1 && !direction.ccw)) {  return -1; }

        if (effect_type == Effect_type::INDEPENDENT) { return 0; } 

        if (infinite) { angle_deg = getInfiniteAngle(angle_deg, range); }
        
        

        for (int i = 0; i < copy_count; i++) {
            if (angle_deg >= copy[i] && angle_deg <= (copy[i] + angle)) {
                return i;
            }
        }
        return -1;   
    }
}




bool FeelixEffect::isHapticEffectActive(float angle_deg, float position, int8_t cw, int range) {

    if ((cw == 1 && !direction.cw) || (cw == -1 && !direction.ccw)) {  return false; }

    if (effect_type == Effect_type::INDEPENDENT) { return true; } 

    if (infinite) { angle_deg = getInfiniteAngle(angle_deg, range); }

    return angle_deg >= position && angle_deg <= position + angle ? true : false;
}




bool FeelixEffect::isVelocityEffectActive(float angle_deg, long time) {

    if (start_time != VAL_NOT_SET) {

        if (control_type == Control_type::VELOCITY_ANGLE && start_angle == VAL_NOT_SET) {
            start_angle = angle_deg * _DEG_PI;
        }

        if (infinite) { return true; }

        if (time >= start_time && time <= (start_time + (long) angle)) {
            return true;
        } else {
            start_angle = VAL_NOT_SET;
            if (time > (start_time + (long) angle)) {
                start_time = VAL_NOT_SET;
            }
            return false;
        }
    }
}


void FeelixEffect::start() {
    start_time = millis();
}

void FeelixEffect::stop() {
    start_time = VAL_NOT_SET;
}



float Effect::getArrayPointerValue(float position, float angle_deg, uint16_t range) {
    if (infinite) { angle_deg = getInfiniteAngle(angle_deg, range); }

    float multiply = control_type == Control_type::POSITION ? 2.0 : 1.0;
    float angle_offset = ((angle_deg - position) * (1.0 / scale.x) / quality); 

    if (flip.x) { angle_offset = (data_size / multiply) - angle_offset; }

    if (effect_type == Effect_type::INDEPENDENT) {
    
        if (angle_offset < 0) { angle_offset = 0; }
        else if (angle_offset > (data_size / multiply) - 1) { angle_offset = (data_size / multiply) - 1; }
    } else {
        if (angle_offset < 0) { angle_offset *= -1; }
    }
    
    return angle_offset;
}





float Effect::getEffectVoltage(float value) {

    float voltage_at_angle = value * scale.y + position;

    if (flip.y) {
        voltage_at_angle = flip.middleline_y + (flip.middleline_y - voltage_at_angle);
    }

    return voltage_at_angle;
}



uint16_t Effect::getArrayPointerValueVelocityEffect(long ms) {

    int offset = round((ms - (long) copy[0]) * (1.0 / scale.x) / quality); 
    
    if (flip.x) { offset = data_size - offset; }
   
    return data_ptr + offset;

}

uint16_t FeelixEffect::getPointerValueVelocityEffect(long ms) {

    int offset = round((ms - start_time) * (1.0 / scale.x) / quality); 
    
    if (flip.x) { offset = data_size - offset; }
   
    return offset;
}



float Effect::getVelocityOverTime(float value) {

    float velocity = value * scale.y;

    // if (flip.y) {
    //     velocity = flip.middleline_y + (flip.middleline_y - velocity);
    // }


    return velocity;

 }


 float Effect::getAngleOverTime(float value) {

    float angle = value * scale.y;

    return angle + start_angle;

 }

