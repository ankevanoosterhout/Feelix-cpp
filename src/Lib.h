#ifndef LIB_H
#define LIB_H

#include "Effect.h"

#define EFFECT_LIST_SIZE    30
#define MAX_ARRAY_SIZE      90

#define MAX_SUB_ARRAY_SIZE  30

struct data {
  float sub[MAX_SUB_ARRAY_SIZE];
};


struct pointer {
  volatile int value;
  volatile int mod;
  volatile int section;
};


class Lib {
  public:

    void init();

    float getValueAtPointer(float value, uint16_t data_ptr, uint8_t step, uint8_t offset);
    float getValueAtPointerInt(uint16_t value);

    void reset();

    Effect effect[EFFECT_LIST_SIZE];

    data d[MAX_ARRAY_SIZE];  
    pointer ptr;

    volatile uint8_t effect_count;

};



#endif