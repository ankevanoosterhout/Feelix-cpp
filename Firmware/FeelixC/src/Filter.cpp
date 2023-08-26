#include "Filter.h"
#include <string.h>


void Filter::init() {
    this->active = false;
    reset(this->constrain);
    reset(this->amplify);
    reset(this->noise);
}


void Filter::reset(fltr filter) {
    filter.active = false;
    filter.current = 0.0;
    filter.derivative = 0.0;
    filter.goal = 0.0;
    filter.smoothness = 0;
    filter.type = 0;
}


float Filter::getFilterValue(fltr filter) {
    if (filter.goal != filter.current) {
        filter.current += filter.derivative;
    }

    return filter.current;
}


fltr Filter::updateFilter(fltr filter, float goalValue, int smoothness) {
    filter.goal = goalValue;
    filter.smoothness = smoothness;
    filter.active = true;

    filter.derivative = (filter.goal - filter.current) / (float) filter.smoothness;

    if (!this->active) { this->active = true; }

    return filter;
}