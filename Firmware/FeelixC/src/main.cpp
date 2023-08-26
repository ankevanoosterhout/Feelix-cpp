//****************************************************************************//
// Torquetuner - firmware                                                     //
// SAT/Metalab                                                                //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Albert Niyonsenga and Christian Frisson (2022)                             //
//****************************************************************************//

/* Created using the Puara template: https://github.com/Puara/puara-module-template 
 * The template contains a fully commented version for the commonly used commands 
 */
//*****************************************************************************//
// INCLUDES Section

//#define SPARKFUN_ESP32_THING_PLUS 1
unsigned int firmware_version = 20230620;
// Define visual feedback
#define VISUAL_FEEDBACK
// Define libmapper
// #define LIBMAPPER
// #define WIFI

#include "Feelix.h"
#include "Arduino.h"
#include "variants.h"

// #include <SPI.h>
#include <Wire.h>

#include <cmath>
#include <mapper.h>
//#include "Filter.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <TaskScheduler.h>
// #include <freertos/FreeRTOS.h>

#ifndef SPARKFUN_ESP32_THING_PLUS
#include <TinyPICO.h>
#endif

#include "haptics.h"
#ifdef WIFI
#include <WiFi.h>
// For disabling power saving
#include "esp_wifi.h"

// Include Puara's module manager
// If using Arduino.h, include it before including puara.h
#include "puara.h"

#endif

const int SEL_PIN = 0;

#ifdef TSTICKJOINT
const int SDA_PIN = 26;
const int SCL_PIN = 25;
#else
#ifdef SPARKFUN_ESP32_THING_PLUS
const int SDA_PIN = 23;
const int SCL_PIN = 22;
#else
const int SDA_PIN = 21;
const int SCL_PIN = 22;
#endif
#endif
//**************************INITIALISE PUARA + MCU Libraries*********************//
#ifdef WIFI
// Initialise Puara
Puara puara;
#endif


#ifndef SPARKFUN_ESP32_THING_PLUS
// Initialise the TinyPICO library
TinyPICO tp = TinyPICO();
#endif

//**************************INITIALISE LCD***************************************//
// LCD properties
#ifdef VISUAL_FEEDBACK
  Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
  #define OFF 0x0
  #define RED 0x1
  #define YELLOW 0x3
  #define GREEN 0x2
  #define TEAL 0x6
  #define BLUE 0x4
  #define VIOLET 0x5
  #define WHITE 0x7 
  int gui_state = 0;
  int max_gui_state = 3; //0 = haptic mode, 1 = angle, 2 = velocity, 3 = torque
  int old_gui_state = 0;


// Initial States
int STATE = 8;
int OLD_VALUE = 9999;
int OLD_STATE = 4;
int MAX_MOTOR_STATES = 8;


void print_state(int cur_state) {
  lcd.setCursor(0,1);
  if (cur_state == 0) {
    lcd.print("CLICK");
  } 
  else if (cur_state == 1) {
    lcd.print("MAGNET");
  }
  else if (cur_state == 2) {
    lcd.print("WALL");
  }
  else if (cur_state == 3) {
    lcd.print("INERTIA");
  }  
  else if (cur_state == 4) {
    lcd.print("LINEAR SPRING");
  }
  else if (cur_state == 5) {
    lcd.print("EXP SPRING");
  }
  else if (cur_state == 6) {
    lcd.print("FREE");
  }
  else if (cur_state == 7) {
    lcd.print("SPIN");
  }
  // else if (cur_state == 8) {
  //   lcd.print("SERIAL LISTEN");
  // }
  else if (cur_state == 8) {
    lcd.print("FEELIX MODE");
  }
  else {
    lcd.print("Unknown State");
  }
}

int CHANGE_STATE(int cur_state, int max_state, int inc_flag) {
  int new_state = 0;
  if (inc_flag) {
    new_state = cur_state + 1;
  } else{
    new_state = cur_state - 1;
  }
  if (new_state > max_state) {
    new_state = 0;
  }
  if (new_state < 0) {
    new_state = max_state;
  }
  // printf("New State %d\n",new_state);
  return new_state;
}

const uint32_t DEBOUNCE_TIME = 10000; // 10 ms
bool update_btn(const int pin) {
  static bool last_val;
  static bool has_changed;
  static int32_t last_change;
  // Read button pin
  int32_t now = esp_timer_get_time();
  bool val =  !digitalRead(pin);
  if (val != last_val) {
    last_val = val;
    last_change = now;
    has_changed = true;
  }

  // Debounce button and trigger on release
  if (has_changed && (now - last_change) > DEBOUNCE_TIME  && !val) {
    has_changed = false;
    return true;
  } else {
    return false;
  }

}

bool update_btn_lcd(uint8_t buttonPressed){
  static bool last_val;
  static bool has_changed;
  static int32_t last_change;
  // Read button pin
  int32_t now = esp_timer_get_time();
  bool val =  !buttonPressed;
  if (val != last_val) {
    last_val = val;
    last_change = now;
    has_changed = true;
  }

  // Debounce button and trigger on release
  if (has_changed && (now - last_change) > DEBOUNCE_TIME  && !val) {
    has_changed = false;
    return true;
  } else {
    return false;
  }

}
#endif
//*************************SET UP TORQUETUNER************************************//
// I2C variables
const uint8_t I2C_BUF_SIZE = 10;
const uint8_t CHECKSUMSIZE = 2;
uint8_t tx_data[I2C_BUF_SIZE+CHECKSUMSIZE];
uint8_t rx_data[I2C_BUF_SIZE+CHECKSUMSIZE];
uint16_t checksum_rx = 0;
uint16_t checksum_tx = 0;

// Initialize TorqueTuner
TorqueTuner knob;

// State flags
int connected = 0;
bool is_playing = true;

//========TORQUETUNER FUNCTIONS========
uint16_t calcsum(uint8_t buf[], uint8_t length) {
  uint16_t val = 0;
  for (int k = 0; k < length; k++) {
    val += buf[k];
  }
  return val;
}

int read_param(float * param, uint8_t*  data, uint8_t length) {
  memcpy(param, data, length);
  if ( std::isnan(*param)) {
    return 1;
  } else {
    return 0;
  }
}

int receiveI2C(TorqueTuner * knob_) {
  Wire.requestFrom(8, I2C_BUF_SIZE + CHECKSUMSIZE);
  uint8_t k = 0;
  while (Wire.available()) {
    rx_data[k] = Wire.read();
    k++;
  }
  if (k != I2C_BUF_SIZE + CHECKSUMSIZE) { // check if all data is recieved
    //printf("Error in recieved data. Bytes missing :  %i\n", I2C_BUF_SIZE + CHECKSUMSIZE - k);
    return 1;
  }
  else {
    memcpy(&checksum_rx, rx_data + I2C_BUF_SIZE, 2); // read checksum
    if (checksum_rx != calcsum(rx_data, I2C_BUF_SIZE)) { // error in recieved data
      return 2;
    }
    else { // Succesfull recieve
      #ifdef MECHADUINO
      memcpy(&knob_->angle, rx_data, 2);
      #endif
      #ifdef MOTEUS
      memcpy(&knob_->angle, rx_data + 1, 2);
      #endif
      memcpy(&knob_->velocity, rx_data + 4, 4);
      //printf("Angle %d Velocity %f\n",knob_->angle,knob_->velocity );
      return 0; //Return 0 if no error has occured
    }
  }
}

void sendI2C(TorqueTuner * knob_) {
  Wire.beginTransmission(8); // transmit to device #8
  memcpy(tx_data, &knob_->torque, 2);
  memcpy(tx_data + 2, &knob_->target_velocity, 4);
  memcpy(tx_data + 6, &knob_->active_mode->pid_mode, 1);
  checksum_tx = calcsum(tx_data, I2C_BUF_SIZE);
  memcpy(tx_data + I2C_BUF_SIZE , &checksum_tx, 2);
  //printf("Torque %d Angle %d Velocity %f Target %f Mode %c\n",knob_->torque,knob_->angle,knob_->velocity,knob_->target_velocity,knob_->active_mode->pid_mode);
  int n = Wire.write(tx_data, I2C_BUF_SIZE + CHECKSUMSIZE);
  Wire.endTransmission();    // stop transmitting
}
//*************************SET UP Feelix************************************//

Commander command = Commander(Serial, '&', false);

Feelix feelix = Feelix(&knob, 'A', I2C_State::NO_COMMUNICATION);

void Feelix::init(){
  TT->set_mode(TorqueTuner::FEELIX_MODE);
}

void process_data(char* cmd) {
  // Serial.println((String) "&p " + cmd);
  feelix.process_data(cmd); 
};
//*************************SET UP LIBMAPPER**************************************//
#ifdef LIBMAPPER

#endif
#ifdef WIFI



#endif
//*********************TASK SCHEDULING*******************************************//
// Timing variables
const uint32_t LIBMAPPER_POLL_RATE = 10000 ; // us
const uint32_t LIBMAPPER_UPDATE_RATE = 10000 ; // us
const uint32_t OSC_UPDATE_RATE = 40000 ; // us
const uint32_t HAPTICS_UPDATE_RATE = 500 ; // 2 KHz
const uint32_t I2CUPDATE_FREQ = 3400000; // high speed mode;
const uint32_t GUI_RATE = 66000; //  15 FPS
const uint32_t INPUT_READ_RATE = 1000; // 1kHz
const uint32_t COMMANDER_RATE = 115200;

Scheduler runnerHaptics;
Scheduler runnerComms;

#ifdef LIBMAPPER

#endif

#ifdef VISUAL_FEEDBACK
void updateDisplay();
void readInputs();
#endif
void updateHaptics();

#ifdef WIFI

#endif

void updateCommander();

// Comms Tasks
#ifdef LIBMAPPER
#endif
#ifdef WIFI
#endif
Task commanderUpdate (COMMANDER_RATE, TASK_FOREVER, &updateCommander, &runnerComms, true);

// Haptic Tasks
Task HapticUpdate (HAPTICS_UPDATE_RATE, TASK_FOREVER, &updateHaptics, &runnerHaptics,true);
#ifdef VISUAL_FEEDBACK
Task DisplayUpdate (GUI_RATE, TASK_FOREVER, &updateDisplay, &runnerHaptics,true);
Task InputRead (INPUT_READ_RATE,TASK_FOREVER,&readInputs, &runnerHaptics,true);
#endif
//==========Functions for task scheduler===========
#ifdef LIBMAPPER

#endif

#ifdef VISUAL_FEEDBACK
void updateDisplay() {
  if ((gui_state == 0) && ((STATE != OLD_STATE) || (old_gui_state != gui_state))){
    if (old_gui_state != gui_state) {
      old_gui_state = 0;
    }
    if (STATE != OLD_STATE) {
      OLD_STATE = STATE;
    }
    
    lcd.clear();
    lcd.print("Haptic Effect");
    print_state(STATE); 

  }
}

void readInputs() {
  // Check buttons
  uint8_t buttons = lcd.readButtons();
  bool button_pressed = update_btn_lcd(buttons);
  if (button_pressed) {
    if ((buttons & BUTTON_RIGHT)) {
      OLD_STATE = STATE;
      STATE = CHANGE_STATE(STATE,MAX_MOTOR_STATES,1);
      knob.set_mode(STATE);
    }
    if (buttons & BUTTON_LEFT) {
      OLD_STATE = STATE;
      STATE = CHANGE_STATE(STATE,MAX_MOTOR_STATES,0);
      knob.set_mode(STATE);
    }
    if (buttons & BUTTON_SELECT) {
      if(feelix.RUN == true) feelix.RUN = false;
      else feelix.RUN = true;
    } 
  }
}
#endif

void updateHaptics() {
  // Recieve Angle and velocity from servo
    int err = receiveI2C(&knob);
    if (err) {
      //printf("i2c error \n");
    }
    else {
      // Update torque if valid angle measure is recieved.
      if (is_playing) {
        if (STATE == 8){
          feelix.run(&knob);
          // printf("Feelix Values goes here \n");
        }
        else{
          knob.update();

        }
      } else { 
        // OBS: Consider not updating? assign last last value instead? //
        knob.torque = 0;
        knob.target_velocity = 0;
      }
      sendI2C(&knob);
    }
}
void updateCommander(){
  command.run();
  // printf("command running");
}
#ifdef WIFI
#endif
// Set up multithreading
#define HAPTIC_CPU 0
#define COMMS_CPU 1

// ===== rtos task handles =========================
TaskHandle_t tHaptics;
TaskHandle_t tCommunications;

// Mappings
void tHapticTasks(void* parameters)  {
  for(;;){
    runnerHaptics.execute();
  }
}

void tCommunicationTasks(void* parameters) {
  for(;;){
    runnerComms.execute();
  }
}

void createCoreTasks() {
  xTaskCreatePinnedToCore(
    tHapticTasks,
    "haptics",
    8096,
    NULL,
    2,
    &tHaptics,
    HAPTIC_CPU);

  xTaskCreatePinnedToCore(
    tCommunicationTasks,   /* Task function. */
    "comms",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    2,           /* priority of the task */
    &tCommunications,  /* task handle */
    COMMS_CPU);
}




//*************************************SETUP*************************************//
void setup() {
  #ifdef VISUAL_FEEDBACK
  // Setup LCD
  lcd.begin(16,2);
  lcd.print("Booting up");
  #endif

  // Start serial
  Serial.begin(115200);

  #ifdef WIFI
  #endif
  

  #ifdef LIBMAPPER
  #endif

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2CUPDATE_FREQ); // Fast mode plus

  // Make a reading for initilization
  int err = 1;
  #ifdef VISUAL_FEEDBACK
  lcd.clear();
  lcd.print("Waiting for I2C");
  #endif
  while (err) {
    err = receiveI2C(&knob);
    #ifdef VISUAL_FEEDBACK
    lcd.setCursor(0, 1);
    lcd.print(millis()/1000);
    #endif
  }
  

  // Show current haptic effect
  #ifdef VISUAL_FEEDBACK
  gui_state = 0; 
  old_gui_state = 0;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Haptic Effect");
  print_state(STATE);
  #endif

  pinMode(SEL_PIN, INPUT);

  feelix.init();

  // analogueWriteFrequency(50000);

  command.add('F', process_data);

  // Print Stuff to Serial
  // Using Serial.print and delay to prevent interruptions
  delay(500);
  Serial.println(); 
  #ifdef WIFI
  #endif
  // Serial.println("Maxwell Gentili-Morin\nIDMIL - CIRMMT - McGill University");
  // Serial.print("Firmware version: "); Serial.println(firmware_version); Serial.println("\n");
  
  // Create tasks  
  createCoreTasks();

}

void loop() {
  //runnerHaptics.execute();
  //runnerComms.execute();
}