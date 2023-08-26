#include "Feelix.h"
#include "FeelixCommands.h"

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>



#define _PI_DEG     57.29577951308
#define _DEG_PI     .0174532925199


uint8_t ledBrightness = 0;
bool brightnessChangeDirection = 1;
bool STATUS_LED_ENABLED = true;
bool LED_STATE = 0;
volatile bool PROCESSING = false;
volatile bool ML5DATACOLLECTION = false;


volatile uint16_t CURRENT_ARRAY_INDEX = 0;
bool BREAK_LOOP = false;

char charFloat[9];
char charLongFloat[18];

int8_t lastEffectID = -1;
 

int address = 0;

Lib library;

bool endOfDataReceive = false;



Feelix::Feelix(BLDCMotor* _bldc, BLDCDriver3PWM* _driver, MagneticSensorSPI* _sensor, char _id, I2C_State _state) {

    INITIALIZED = false;  
    MOVE = false;
    bldc = _bldc;
    sensor = _sensor;
    driver = _driver;
    current_sense = NULL;
    library.init();
    communication_speed = 50;
    control_type = Control_type::UNDEFINED;
    range = 360.0;
    start_pos = 0.0; //RAD
    start_time = 0;
    version.major = 3;
    version.minor = 0;
    version.patch = 0;
    filter.init();
    id = _id;
    I2C_address = (int)strtol(&_id, NULL, 16);
    I2C_state = _state;
    dataRequestType = '\0';
    nrOfConnectedDevices = 0;
    blink_time = millis();
    blink_interval = 1000;

    for (int i = 0; i<VELOCITY_FILTER_SIZE; i++) {
        velocity_filter[i] = 0.0;
    }
    velocity_filter_index = 0;
    f_velocity = 0.0;
    total_vel = 0.0;
}


Feelix::Feelix(BLDCMotor* _bldc, BLDCDriver3PWM* _driver, MagneticSensorSPI* _sensor, InlineCurrentSense* _current_sense, char _id, I2C_State _state) {

    Feelix(_bldc, _driver, _sensor, _id, _state);

    CURRENT_SENSE_ACTIVE = true;
    OVERHEAT_PROTECTION = false;
    OVERHEAT_PROTECTION_ACTIVE = false;
    current_threshold = .05;
    overheat_count = 0;
}

Feelix::Feelix(TorqueTuner* _TT, char _id, I2C_State _state){
    INITIALIZED = false;  
    TT = _TT;
    MOVE = false;
    bldc = NULL;
    sensor = NULL;
    driver = NULL;
    current_sense = NULL;
    library.init();
    communication_speed = 50;
    control_type = Control_type::UNDEFINED;
    range = 360.0;
    start_pos = 0.0; //RAD
    start_time = 0;
    version.major = 3;
    version.minor = 0;
    version.patch = 0;
    filter.init();
    id = _id;
    I2C_address = (int)strtol(&_id, NULL, 16);
    I2C_state = _state;
    dataRequestType = '\0';
    nrOfConnectedDevices = 0;
    blink_time = millis();
    blink_interval = 1000;

    for (int i = 0; i<VELOCITY_FILTER_SIZE; i++) {
        velocity_filter[i] = 0.0;
    }
    velocity_filter_index = 0;
    f_velocity = 0.0;
    total_vel = 0.0;
}




static int convert_ID_to_I2C_address(char* ID) {
  int address = strtol(ID, NULL, 16);
  return address;
}




static char* convertData(char* user_command, char* cstr) {
    char* cmd = strtok(user_command, "&");
    // cmd = strtok(NULL, "&");
    std::string str(cmd);
    str = "F" + str;
    str += "&";
    cstr = new char[str.length() + 1];
    strcpy(cstr, str.c_str());

    return cstr;
}


bool Feelix::transferData(char identifier, char* user_command) {
    // Serial.println((String) "&state " + I2C_state + " " + identifier + " " + id);

    if (I2C_state == I2C_State::MASTER && identifier != id) {
        // Serial.println((String) "&forward data to " + identifier);

        address = convert_ID_to_I2C_address(&identifier);
        // Serial.println((String) "&command " + user_command);
        char *cstr = convertData(user_command, cstr);
        // Serial.println((String) "&data " + cstr);
        transmitDataI2C(address, cstr);

        
        // Serial.println("*");
        
        return true;
    }
    return false;
}





void Feelix::receiveDataI2C() {

    if (I2C_state == I2C_State::MASTER) {
        I2CincomingDataMaster = "";
        endOfDataReceive = false;

        while(Wire.available()) {
            I2CincomingDataMaster = Wire.readStringUntil('/n');
            // Serial.print((String) " " + c);  

            // if (!endOfDataReceive) {
                // I2CincomingDataMaster += c;
                // if (c == '&') {
                //     endOfDataReceive = true;
                    // Serial.println();
                // }
            // }
           
                            
        }
        if (I2CincomingDataMaster != "") {
            endOfDataReceive = false;
            Serial.print("received data ");
            Serial.println(I2CincomingDataMaster); 
            
        }
    }
}


void Feelix::process_data(char* cmd) { 
    Serial.println((String) cmd);

    char command = cmd[0];
    // char* fullCommand = strtok(cmd, "&");
    // fullCommand = strtok(0, "&");

    // Serial.println((String) "&" + fullCommand);

    switch (command) {
        case 'T':
            {TorqueTuner_Data(cmd);
            break;}
        // case 'M':
        //     BLDC_Data(cmd);
        //     break;
        case 'E':
            {Effect_Data(cmd);
            break;}
        case 'D':
            {Data_Points(cmd);   
            break; }
        case 'C': 
            {BLDC_Config(cmd);            
            break;}
        case 'F':
            {Filter_Data(cmd);
            break;}
        case 'S':
            {Serial.println((String) "S" + version.major + '.' + version.minor + '.' + version.patch);
            Serial.println("*");
            break;}
        case 'G':
           { Return_Data(cmd);
            break;}
        case 'R':
            {Serial.println("*");
            break;}
        case 'L':
            {int devices = listDevices();
            Serial.println((String) "Ldevices " + devices + " " + I2C_state);
            break;}
        default:
        {
            Serial.print("Unrecognized command:");
            Serial.println((String) cmd);
            Serial.println("*");
            break;
            }
    }
  
  delay(5);
};



char* Feelix::returnDataOnRequest() {
    char *cstr = NULL;

    switch(dataRequestType) {
        case '\0':
            return cstr;
            break;
        case 'A': {
                dataOutStr = "A";
                dataOutStr += id;
                dataOutStr += ":";
                dtostrf(angle, 9, 4, charFloat); 
                dataOutStr += charFloat;
                dataOutStr += ":";
                dtostrf(velocity, 9, 3, charFloat); 
                dataOutStr += charFloat;
                dataOutStr += ":";
                dataOutStr += current_time;
                dataOutStr += ":";
                dtostrf(target_val, 9, 4, charFloat);  
                dataOutStr += charFloat;
                dataOutStr += "&";
                // if (dataOutStr.length() < 39) {
                //     for (int i = dataOutStr.length(); i < 39; i++) {
                //         dataOutStr += '0';
                //     }
                // }
            }
            break;
        case 'C': {
                dataOutStr = "Z";
                dataOutStr += id;
                dataOutStr += ":";
                dtostrf(bldc->zero_electric_angle, 18, 15, charLongFloat); 
                dataOutStr += charLongFloat;
                dataOutStr += ":";
                dataOutStr += bldc->sensor_direction;
                dataOutStr += "&";
            }
            break;
        case 'R': {
            dataOutStr = "R";
            dataOutStr += id;
            dataOutStr += ":A:";
            dtostrf(angle, 18, 15, charLongFloat); 
            dataOutStr += charLongFloat;
            dataOutStr += "&";
        }
    }

    cstr = new char[dataOutStr.length() + 1];
    strcpy(cstr, dataOutStr.c_str());
    dataRequestType = '\0';

    return cstr;
    
}


void Feelix::BLDC_Config(char* user_command) {
    // Serial.println((String) "&CONFIG" + user_command);

    // if (!transferData(user_command[1], user_command)) {

    // Serial.println((String) "CONFIG " + user_command[1]);

    // if (I2C_state == I2C_State::SLAVE) {
    //     dataRequestType = 'C';
    // } 

    // bldc->init();
    // bldc->initFOC();

    // if (bldc->zero_electric_angle != NOT_SET && bldc->sensor_direction != NOT_SET) {
    //     bldc->initFOC(bldc->zero_electric_angle, (bldc->sensor_direction == 1 ? Direction::CW : Direction::CCW));
    // } else {
    //     bldc->initFOC();
    // }
    
    // if (current_sense != NULL) {
    //     current_sense->init();
    //     current_sense->gain_b *= -1;
    // }

    PROCESSING = false;
    INITIALIZED = true; 

    delay(100);

    start_time = millis();    

    Serial.println("recieved");
    // Serial.print((String) "Z" + id + ":");
    // Serial.print(bldc->zero_electric_angle, 18);
    // Serial.print(":");
    // Serial.println(bldc->sensor_direction);
        
    
    // // } else 
    // if (transferData(user_command[1], user_command)) {
    //     volatile bool inlist = false;
    //     address = convert_ID_to_I2C_address(&user_command[1]);

    //     for (int n = 0; n<nrOfConnectedDevices; n++) {
    //         if (I2C_connections[n] == address) { inlist = true; }
    //     }
        
    //     if (!inlist) {
    //         I2C_connections[nrOfConnectedDevices] = address;
    //         Serial.println((String) "I2C_ADDRESS " + I2C_connections[nrOfConnectedDevices]);
    //         nrOfConnectedDevices++;
    //         Serial.println((String) "NR_I2C_CONNECTIONS " + nrOfConnectedDevices);            
    //     }    
    //     delay(300);      
    //     Serial.println((String) "#request data " + address);
    //     Wire.requestFrom(address, 24); 
    // }
}





void Feelix::update() {
    angle = (bldc->shaft_angle) * sensor_dir + sensor_offset; //(180 / 3.14159);
    velocity = bldc->shaft_velocity * sensor_dir;
    target = 0.0;
    target_val = 0.0;
    voltage = 0.0;
    control_type = Control_type::UNDEFINED;

    current_time = (millis() - start_time);

    getDirection();  
}


void Feelix::run(TorqueTuner* TT){
    angle = (float) TT->angle;
    angle_deg = angle/10.0;
    velocity = TT->velocity;
    getDirection();  
    f_velocity = filterVelocity();
    target = 0.0;
    control_type = Control_type::UNDEFINED;
    BREAK_LOOP = false;

    if (RUN && !PROCESSING) {
        current_time = (millis() - start_time);


        if (constrain_range && (angle_deg > (start_pos + range) || angle <= start_pos)) {
            target_val = 0.0;

        } else {
            for (int e = 0; e < library.effect_count; e++) {
                

                active_effect = library.effect[e].isActive(angle_deg, rotation_dir, range, current_time);
                // Serial.println(active_effect);
                if (active_effect > -1) {
                    switch(library.effect[e].control_type) {
                        
                        case Control_type::TORQUE: 
                            if (control_type == Control_type::UNDEFINED || control_type == Control_type::TORQUE) {
                                control_type = Control_type::TORQUE;
                                float ptr = library.effect[e].getArrayPointerValue(library.effect[e].copy[active_effect], angle_deg, range);
                                target = library.getValueAtPointer(ptr, library.effect[e].data_ptr, 1, 0)*50.0; //TODO: Should technically add a gain value instead of a flat 25 etc
                                // Serial.printf("Angle from TorqueTuner: %f",angle_deg);
                                // Serial.print("********* Target: ");
                                // Serial.print(target);
                                // Serial.println(
                                //     "*****************"
                                // );
                                TT->torque = target;
                                
                            }
                            break;
                        case Control_type::MIDI:
                            if (control_type == Control_type::UNDEFINED || control_type == Control_type::MIDI) {
                                control_type = Control_type::MIDI;
                                float ptr = library.effect[e].getArrayPointerValue(library.effect[e].copy[active_effect], angle_deg, range);
                                target = library.getValueAtPointer(ptr, library.effect[e].data_ptr, 1, 0);
                                int cmd = library.effect[e].midi_config.message_type + library.effect[e].midi_config.channel;
                                TT->midi_CC_out(library.effect[e].midi_config.channel, library.effect[e].midi_config.data1, target);
                            }
                            break;
                    }

                    if (library.effect[e].effect_type == Effect_type::NOTSET) { BREAK_LOOP = true; }
                }    

                if (BREAK_LOOP) { break; }
            }
        }
    }
}











void Feelix::velocityLimitProtection() {
    if (bldc->shaft_velocity > bldc->velocity_limit || bldc->shaft_velocity < -bldc->velocity_limit) {
       RUN = false;
       bldc->controller = MotionControlType::torque;
       bldc->move(0.0); 
       Serial.println("V");
   }
}




float Feelix::applyFilters(int controlType, float targetValue) {
    if (filter.active) {
        if (filter.amplify.active) {
            volatile float filterValue = filter.getFilterValue(filter.amplify);
            targetValue *= filterValue;
        }
        if (filter.noise.active) {
            volatile float filterValue = filter.getFilterValue(filter.noise);
            targetValue += (driver->voltage_power_supply * filterValue);
        }
        if (filter.constrain.active) {
            if (controlType < 2) {
                driver->voltage_limit = filter.constrain.goal * driver->voltage_power_supply;
            } else {
                bldc->velocity_limit = filter.constrain.goal * bldc->velocity_limit;
            }
        }
    }
    return targetValue;
}   

















void Feelix::calibrateCurrentSenseValues() {
    RUN = false;
    int x = 0;
    bldc->controller = MotionControlType::torque;
    float max_current_sense = 0.0;
    float min_current_sense = 0.0;
    while (x < 200) {
        bldc->loopFOC();
        bldc->move(driver->voltage_power_supply);
        if (x % 5 == 0) {
            currents = current_sense->getPhaseCurrents();
            max_current_sense = currents.a > max_current_sense ? currents.a : max_current_sense; 
            max_current_sense = currents.b > max_current_sense ? currents.b : max_current_sense; 
            min_current_sense = currents.a < min_current_sense ? currents.a : min_current_sense; 
            min_current_sense = currents.b < min_current_sense ? currents.b : min_current_sense; 
        }
        delay(1);
        x++;
    }
    bldc->move(0.0);
    current_threshold = min_current_sense * -1 > max_current_sense ? min_current_sense * -0.75 : max_current_sense * 0.75;
    
    Serial.println((String) "C" + id + ":" + current_threshold);
}




void Feelix::getCurrentSenseValues() {
    if (current_sense != NULL) {
        currents = current_sense->getPhaseCurrents();

        if (OVERHEAT_PROTECTION_ACTIVE && abs(bldc->shaft_velocity) < 5.0) {

            if ((currents.a < current_threshold * -1 || currents.a > current_threshold) || (currents.b < current_threshold * -1 || currents.b > current_threshold)) {
                overheat_count++;

                if (overheat_count > communication_speed * 100) {
                    OVERHEAT_PROTECTION = true;
                    Serial.println("O");
                    bldc->controller = MotionControlType::torque; 
                    bldc->move(0.0);
                    RUN = false;
                    MOVE = false;
                }
                return;
            }
        }
        overheat_count = 0;
    }
}





void Feelix::getDirection() {

    if (velocity > 1.0) {
        rotation_dir = Direction::CW;
    } else if (velocity < -1.0) {
        rotation_dir = Direction::CCW;
    }
}

// void Feelix::getDirection(TorqueTuner* TT) {
//     if (TT->velocity > 1.0) {
//         rotation_dir = Direction::CW;
//     } else if (TT->velocity < -1.0) {
//         rotation_dir = Direction::CCW;
//     }
// }



float Feelix::filterVelocity() {

    if (velocity_filter_index++ > VELOCITY_FILTER_SIZE) {
        velocity_filter_index = 0;
    }
    
    total_vel -= velocity_filter[velocity_filter_index];

    velocity_filter[velocity_filter_index] = velocity;

    if (velocity > -0.05 && velocity < 0.05) {

        for (int i = 0; i<VELOCITY_FILTER_SIZE; i++) {
            velocity_filter[i] = 0.0;
        }
        total_vel = 0.0;
        return total_vel;

    } else {

        total_vel += velocity;
        return total_vel / (float) VELOCITY_FILTER_SIZE;
    }
}



void Feelix::TorqueTuner_Data(char* user_command){
    char identifier = user_command[1];
}

void Feelix::BLDC_Data(char* user_command){ 
   
    char identifier = user_command[1];
    
    
    if (!transferData(identifier, user_command)) {

        char sub_cmd = user_command[2];
        char* value = strtok(user_command, ":");
        switch(sub_cmd){

            case M_DATA_SEQUENCE: {
                    PROCESSING = true;
                    library.reset();
                    lastEffectID = -1;
                    CURRENT_ARRAY_INDEX = 0;
                    RUN = true;
                    delay(50);
                }
                break;
            case M_ID:  
                PROCESSING = true;
                // id = user_command[3];
                break;
            case M_POLE_PAIRS:  
                bldc->pole_pairs = atoi(&user_command[3]);
                break;
            case M_ZERO_EL_ANGLE: 
                bldc->zero_electric_angle = atof(&user_command[3]);
                break;
            case M_SENSOR_DIRECTION: 
                bldc->sensor_direction = atoi(&user_command[3]) == 1 ? Direction::CW : Direction::CCW;
                break;
            case M_SENSOR_OFFSET:  
                sensor_offset = atof(&user_command[3]);
                sensor_offset = sensor_dir == Direction::CW ? sensor_offset : -sensor_offset;
                break;
            case M_MAG_ENC_CLK_SPD: 
                sensor->clock_speed = atol(&user_command[3]);
                break;
            case M_SUPPLY_VOL:  
                driver->voltage_power_supply = atof(&user_command[3]);
                break;
            case M_VOL_LIMIT: 
                bldc->voltage_limit = atof(&user_command[3]); 
                driver->voltage_limit = bldc->voltage_limit;
                vol_limit = bldc->voltage_limit;
                break;
            case M_COMM_SPEED:
                communication_speed = atoi(&user_command[3]);
                break;
            case M_LOOP:
                loop = atoi(&user_command[3]) == 1 ? true : false;
                break;
            case M_SENSOR_DIR: 
                sensor_dir = atoi(&user_command[3]) == 1 ? Direction::CW : Direction::CCW;
                sensor_offset = sensor_dir == Direction::CW ? sensor_offset : -sensor_offset;
                break;
            case M_VEL_LIMIT:  
                bldc->velocity_limit = atof(&user_command[3]);
                break;
            case M_ANGLE_PID:  
                value = strtok(0, ":");
                bldc->P_angle.P = atof(value);

                value = strtok(0, ":");
                bldc->P_angle.I = atof(value);

                value = strtok(0, ":");
                bldc->P_angle.D = atof(value);
                break;
            case M_VEL_PID:  
                value = strtok(0, ":");
                bldc->PID_velocity.P = atof(value);

                value = strtok(0, ":");
                bldc->PID_velocity.I = atof(value);

                value = strtok(0, ":");
                bldc->PID_velocity.D = atof(value);
                break;
            case M_RANGE:  
                range = atof(&user_command[3]);
                break;
            case M_STARTPOS:  
                start_pos = atof(&user_command[3]);
                break;
            case M_CONSTRAIN_RANGE:  
                constrain_range = atoi(&user_command[3]) == 1 ? true : false;
                break;
            case M_PLAY: {
                    uint8_t play = atoi(&user_command[3]);
                    RUN = play == 1 ? true : false;
                    if (RUN) { 
                        start_time = millis(); 
                        current_time = start_time;
                    } else {
                        bldc->controller = MotionControlType::torque;
                        bldc->move(0.0);
                    }
                }
                break;
            case M_RETURN: {
                    RUN = false;
                    MOVE = true;
                    float target_angle = atof(&user_command[3]);
                    target_val = target_angle * _DEG_PI * sensor_dir;
                    driver->voltage_limit = driver->voltage_power_supply;
                }
                break;
            case M_CALIBRATE:  
                bldc->zero_electric_angle = NOT_SET;
                break;
            case M_CURRENT_SENSE:
                CURRENT_SENSE_ACTIVE = atoi(&user_command[3]) == 1 ? true : false;
                break;
            case M_CALIBRATE_CS: {
                    if (current_sense != NULL) {
                        CURRENT_SENSE_ACTIVE = true;
                        calibrateCurrentSenseValues();
                    }
                }
                break;
            case M_CS_THRESHOLD: 
                current_threshold = atof(&user_command[3]);
                break;
            case M_OVERHEAT_PROTECT:
                OVERHEAT_PROTECTION_ACTIVE = true;
                break;
            // case M_I2C_CONNECTION:
                // const int state = atoi(&user_command[3]);
                // I2C_state = I2C_State(state);
                // Serial.println("&update i2c state" + I2C_state);

                // if (I2C_state != NO_COMMUNICATION) { initI2C(I2C_state); }
                // break;
        };
    }
    Serial.println("*");
}


void Feelix::Filter_Data(char* user_command) {

    char sub_cmd = user_command[1];
    char* value = strtok(user_command, ":"); 
    value = strtok(0, ":");
    float goal = atof(value);
    value = strtok(0, ":");
    int smoothness = atoi(value);

    // if (identifier != id && I2C_state != NO_COMMUNICATION) {

    //     int address = convert_ID_to_I2C_address(identifier);
    //     transmitDataI2C(address, user_command);
        
    // } else {
    
        switch(sub_cmd){

            case F_AMPLIFY:
                filter.amplify = filter.updateFilter(filter.amplify, goal, smoothness);
                break;

            case F_CONSTRAIN:
                filter.constrain = filter.updateFilter(filter.constrain, goal, smoothness);
                break;

            case F_NOISE:          
                filter.noise = filter.updateFilter(filter.noise, goal, smoothness);
                break;

            case F_RESET:
                filter.reset(filter.amplify);
                filter.reset(filter.constrain);
                filter.reset(filter.noise);
                break;
        };
    // }

    // Serial.println("*");
}

void Feelix::Return_Data(char* user_command) {
    char identifier = user_command[1];
    char sub_cmd = user_command[2];
    // Serial.println((String) "&" + identifier + " "+ sub_cmd);

    if (identifier != id && I2C_state == I2C_State::MASTER) {        
        address = convert_ID_to_I2C_address(&identifier);

        char *cstr = convertData(user_command, cstr);
        // Serial.println((String) "&data " + cstr);
        transmitDataI2C(address, cstr);

        // delay(100);
        // Serial.println((String) "&request from " + address);
        Wire.requestFrom(address, 23);
        
    } else {
        

        switch(sub_cmd){

            case G_ANGLE: {
                    Serial.println((String) "R" + id + ":A:" + angle);
                    if (I2C_state == I2C_State::SLAVE) {
                        dataRequestType = 'R';
                    } 
                }
                break;
        };
    }

}




void Feelix::Effect_Data(char* user_command){ 
    char identifier = user_command[1];
    
    if (!transferData(identifier, user_command)) {
        //SHOULD CHECK IF THERE IS A NULL VALUE IN STRTOK 
        uint8_t effect_id = atoi(&user_command[2]);
        char sub_cmd = user_command[3];
        char* value = strtok(user_command, ":");
        value = strtok(0, ":");

        if (effect_id > lastEffectID) {
            library.effect_count++;
            lastEffectID = effect_id;
        }

        switch(sub_cmd){
            
            case CMD_E_MIDI_CC:
                library.effect[effect_id].midi_config.channel = atoi(value);
                value = strtok(0, ":");
                library.effect[effect_id].midi_config.message_type = atoi(value);
                value = strtok(0, ":");
                library.effect[effect_id].midi_config.data1 = atoi(value);
                break;
            case CMD_E_DIR:  
                library.effect[effect_id].direction.cw = atoi(value) == 1 ? true : false;
                value = strtok(0, ":");
                library.effect[effect_id].direction.ccw = atoi(value) == 1 ? true : false;
                break;
            case CMD_E_FLIP:  
                library.effect[effect_id].flip.x = atoi(value);
                value = strtok(0, ":");
                library.effect[effect_id].flip.y = atoi(value);
                value = strtok(0, ":");
                library.effect[effect_id].flip.middleline_y = atof(value);
                break;
            case CMD_E_POS: 
                library.effect[effect_id].position = atof(value);
                break;
            case CMD_E_SCALE:  
                library.effect[effect_id].scale.x = atof(value);
                value = strtok(0, ":");
                library.effect[effect_id].scale.y = atof(value);
                break;
            case CMD_E_ANGLE: 
                library.effect[effect_id].angle = atof(value);
                break;
            case CMD_E_INF:
                library.effect[effect_id].infinite = atoi(value) == 1 ? true : false;
                break;
            case CMD_E_CONTROL_TYPE:
                if (atoi(value) == 0) {
                    library.effect[effect_id].control_type = Control_type::TORQUE;
                } else if (atoi(value) == 1) {
                    library.effect[effect_id].control_type = Control_type::POSITION;
                } else if (atoi(value) == 2) {
                    library.effect[effect_id].control_type = Control_type::VELOCITY;
                    RUN = false;
                } else if (atoi(value) == 3) {
                    library.effect[effect_id].control_type = Control_type::VELOCITY_ANGLE;
                    RUN = false;
                } else if (atoi(value) == 4) {
                    library.effect[effect_id].control_type = Control_type::MIDI;
                }
                break;
            case CMD_E_EFFECT_TYPE: 
                library.effect[effect_id].effect_type = atoi(value) == 0 ? Effect_type::DEPENDENT : Effect_type::INDEPENDENT;
                break;
            case CMD_E_DATA_SIZE: 
                library.effect[effect_id].data_size = atoi(value);
                break;
            case CMD_E_POINTER: 
                library.effect[effect_id].data_ptr = atoi(value);
                break;
            case CMD_E_QUALITY: 
                library.effect[effect_id].quality = atof(value);
                break;
            case CMD_E_COPIES:
                library.effect[effect_id].copy[library.effect[effect_id].copy_count] = atof(value);
                library.effect[effect_id].copy_count++;   
                break;
        }        
    }

    Serial.println("*");
}


void Feelix::Data_Points(char* user_command){ 
    // char cmd = user_command[1];
    char identifier = user_command[2];

    if (!transferData(identifier, user_command)) {
        
        char* value = strtok(user_command, ":");
        value = strtok(0, ":");
        uint8_t i = 0;

        while (value != NULL) {
                
            uint16_t mod = (CURRENT_ARRAY_INDEX % MAX_SUB_ARRAY_SIZE);
            uint16_t section = floor(CURRENT_ARRAY_INDEX / MAX_SUB_ARRAY_SIZE);
            library.d[section].sub[mod] = atof(value);

            CURRENT_ARRAY_INDEX++;
            
            
            if (CURRENT_ARRAY_INDEX > MAX_ARRAY_SIZE * MAX_SUB_ARRAY_SIZE) {
                Serial.println("M");
            }

            value = strtok(0, ":");
            i++;
        } 

    }

    Serial.println("*");

};


void Feelix::broadcastRequest() {
    for (int8_t n = 0; n < nrOfConnectedDevices; n++) {
        requestDataI2C(I2C_connections[n]);
    }
}


void Feelix::send_data() {
    if (!PROCESSING && (RUN || MOVE)) {   

        if (I2C_state == I2C_State::SLAVE) {
            dataRequestType = 'A';
        }

        
        if (CURRENT_SENSE_ACTIVE && current_sense != NULL) {
            getCurrentSenseValues();

            // if (I2C_state != I2C_State::SLAVE) {
                // Serial.println((String) "A" + id + ":" + angle + ":" + f_velocity + ":" + current_time + ":" + target_val + ":" + currents.a + ":" + currents.b);
            // }
        } else {
            // if (I2C_state != I2C_State::SLAVE) {
                // Serial.println((String) "A" + id + ":" + angle + ":" + f_velocity + ":" + current_time + ":" + target_val);
            // }
        }

        if (I2C_state == I2C_State::MASTER && nrOfConnectedDevices > 0) {
            broadcastRequest();
        }
        // }
    }
} 


int Feelix::listDevices() {
    if (I2C_state == I2C_State::MASTER) {
        PROCESSING = true;
        byte error, address;
        int nDevices;

        // Serial.println("LScanning...");

        nDevices = 0;
        for(address = 1; address < 127; address++ ) {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            Wire.beginTransmission(address);
            error = Wire.endTransmission(true);
            

            if (error == 0) {
                Serial.print("LI2C device found at address 0x");
                if (address<16)  
                    Serial.print("0");
                    Serial.println(address,HEX);

                nDevices++;
            } else {
                Serial.print("Lerror ");
                Serial.println(error);
            }

        }
        if (nDevices == 0)
            Serial.println("LNo I2C devices found\n");
        else
            Serial.println("Lscane complete");
            PROCESSING = false;
            return nDevices;
    }
    return 0;

    Serial.println("*");
}

void Feelix::blinkStatusLED() {
    if (STATUS_LED_ENABLED) {
        if (millis() - blink_time > blink_interval) {
            blink_time = millis();
            LED_STATE = !LED_STATE;  

            if (LED_STATE) {
                digitalWrite(STATUS_LED, LOW);  
            } else if (!LED_STATE) {
                digitalWrite(STATUS_LED, HIGH);  
            }
        }
    }
}




void Feelix::requestDataI2C(int I2C_ADDR) {

  Wire.requestFrom(I2C_ADDR, 40);
}






void Feelix::transmitDataI2C(int I2C_ADDR, char* str) {
//dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
//   Serial.println((String) "&Send data over I2C " + str);
  Wire.beginTransmission(I2C_ADDR); 
  Wire.write(str);              
  byte error = Wire.endTransmission(true);    
  if (error != 0) {
    Serial.println((String) "#error " + error);
  }
  
}

