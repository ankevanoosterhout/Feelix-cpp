#include "Lib.h"



void Lib::init() {
    reset();
}



float Lib::getValueAtPointer(float value, uint16_t data_ptr, uint8_t step, uint8_t offset) {

    float dX = value - floor(value);
    ptr.value = data_ptr + (floor(value) * step) + offset; 
    ptr.mod = ptr.value % MAX_SUB_ARRAY_SIZE;
    ptr.section = floor(ptr.value / MAX_SUB_ARRAY_SIZE);

    volatile float value_1 = d[ptr.section].sub[ptr.mod];
    volatile float value_2 = d[ptr.section].sub[ptr.mod + step];

    return ((value_2 - value_1) * dX) + value_1;
}



float Lib::getValueAtPointerInt(uint16_t value) {

    ptr.mod = value % MAX_SUB_ARRAY_SIZE;
    ptr.section = floor(value / MAX_SUB_ARRAY_SIZE);

    return d[ptr.section].sub[ptr.mod];
}





void Lib::reset() {

    for (int e = 0; e < EFFECT_LIST_SIZE; e++) {
        effect[e].data_size = 0;
        effect[e].infinite = 0;
        effect[e].position = 0.0;
        effect[e].scale.x = 0.0;
        effect[e].scale.y = 0.0;
        effect[e].flip.x = 0;
        effect[e].flip.y = 0;
        effect[e].flip.middleline_y = 0.0;
        effect[e].direction.cw = 0;
        effect[e].direction.ccw = 0;
        effect[e].control_type = Control_type::UNDEFINED;
        effect[e].effect_type = Effect_type::NOTSET;
        effect[e].data_ptr = 0;
        effect[e].quality = 1;
        effect[e].start_angle = VAL_NOT_SET;

        for (int c = 0; c < MAX_NR_OF_COPIES; c++) {
            effect[e].copy[c] = 0.0;
        }

        effect[e].copy_count = 0;
    }

    for (int i = 0; i < MAX_ARRAY_SIZE; i++) {
        for (int j = 0; j < MAX_SUB_ARRAY_SIZE; j++) {
            d[i].sub[j] = 0.0;
        }
    }

    ptr.mod = 0;
    ptr.value = 0;
    ptr.section = 0;
    effect_count = 0;
}
