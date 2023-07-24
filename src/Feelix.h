#ifndef Feelix_h
#define Feelix_h

#include <SimpleFOC.h>


#include "Arduino.h"
#include "Lib.h"
#include "Filter.h"
#include <Wire.h>

#define VELOCITY_FILTER_SIZE 8


struct SoftwareVersion {
    int major;
    int minor;
    int patch;
};


enum I2C_State {
    NO_COMMUNICATION  = 0, 
    MASTER            = 1,
    SLAVE             = 2 
};





class Feelix
{
  public:

    Feelix(BLDCMotor* bldc, BLDCDriver3PWM* driver, MagneticSensorSPI* sensor, InlineCurrentSense* current_sense, char _id, I2C_State _state);
    Feelix(BLDCMotor* bldc, BLDCDriver3PWM* driver, MagneticSensorSPI* sensor, char _id, I2C_State _state);


    void init();


    void send_data();
    void blinkStatusLED();

    void run(uint8_t loop_count);
    void move();
    void moveTo();

    float applyFilters(int controlType, float targetValue);
    void getDirection();
    void getCurrentSenseValues();
    // void setPhaseVoltage(float voltage, float angle_el);

    void process_data(char* cmd);

    void initI2C();
    int listDevices();
    void requestDataI2C(int I2C_ADDR);
    void transmitDataI2C(int I2C_ADDR, char* str);
    void receiveDataI2C();

  
    // void initI2C(I2C_State state);

    // void transmitDataI2C(int I2C_ADDR, String data);
    // void requestDataI2C(int I2C_ADDR);

    // static void receiveDataI2C_slave(int n);
    

    void update();
    void playHapticEffectAtAngle(FeelixEffect effect, float angle);
    void playVelocityEffect(FeelixEffect effect);
    // void stop();

    void calibrateCurrentSenseValues();

    char* returnDataOnRequest();
    
    BLDCMotor* bldc;
    BLDCDriver3PWM* driver;
    MagneticSensorSPI* sensor;
    InlineCurrentSense* current_sense;

    volatile float angle;
    volatile float angle_deg;
    Direction sensor_dir;
    Direction rotation_dir;
    float sensor_offset;
    float velocity;
    float f_velocity;
    float target;
    float voltage;
    float target_val;
    float vol_limit;
    PhaseCurrent_s currents;
    float range;
    float start_pos;
    bool constrain_range;
    Control_type control_type;

    Filter filter;

    uint16_t communication_speed;
    unsigned long current_time;
    unsigned long start_time;
    bool loop;
  
    char id;

    int STATUS_LED;
    bool INITIALIZED;
    bool RUN;   
    bool MOVE;
    bool CURRENT_SENSE_ACTIVE;
    bool OVERHEAT_PROTECTION;
    bool OVERHEAT_PROTECTION_ACTIVE;
    uint16_t overheat_count;
    float current_threshold;
    

    I2C_State I2C_state;
    int I2C_address;
    int8_t nrOfConnectedDevices;
    int I2C_connections[20];

    String I2CincomingData = "";
    String I2CincomingDataMaster = "";
    String dataOutStr = "";   

    char dataRequestType;
    
    

  private: 
    void broadcastRequest();
    SoftwareVersion version;
    unsigned long blink_time;
    long blink_interval;
    
    int8_t active_effect;
    void velocityLimitProtection();
    bool transferData(char identifier, char* user_command);

    void BLDC_Config(char* cmd);
    void BLDC_Data(char* cmd);
    void Effect_Data(char* cmd);
    void Data_Points(char* cmd);
    void Filter_Data(char* cmd);
    void Return_Data(char* cmd);

    float velocity_filter[VELOCITY_FILTER_SIZE];
    uint8_t velocity_filter_index;

    float filterVelocity();
    float total_vel;

};


#endif
