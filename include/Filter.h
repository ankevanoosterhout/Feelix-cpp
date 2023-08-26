#ifndef FILTER_H
#define FILTER_H

struct fltr {
    volatile float goal;
    volatile float current;
    volatile float derivative;
    volatile int smoothness;
    volatile int type;
    volatile bool active;
};


class Filter {

    public: 
        
        void init();
        fltr updateFilter(fltr filter, float goalValue, int smoothness);
        void reset(fltr filter);

        float getFilterValue(fltr filter);
      
        fltr constrain;
        fltr amplify;
        fltr noise;

        volatile bool active;
        
    private:
    
};




#endif