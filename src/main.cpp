#include "Feelix.h"


/*
 * This is a library written to be used to play exported effects from Feelix.
 * The library is based on the SimpleFOC library (Copyright (c) 2020 Antun Skuric)
 */





#define CS_MAGNETIC_SENSOR_ADDRESS  0x3FFF

/* for STM32F401 v1.1 boards / STM32F303 v1.1 boards */
#define CS_MAGNETIC_SENSOR_PIN      PA4 
/* uncomment for Teensy controlled motors */
/* #define CS_MAGNETIC_SENSOR_PIN      10 */

/* initialize commander for usage with SimpleFOC library */
Commander command = Commander(Serial, '&', false);

/* initialize motor (GB4315) */
BLDCMotor _bldc = BLDCMotor(7);

/* for STM32F401 v1.1 boards / STM32F303 v1.1 boards */
BLDCDriver3PWM _driver = BLDCDriver3PWM(PC8, PC7, PC6, PB15);
/* uncomment for Teensy controlled motors */
/* BLDCDriver3PWM _driver = BLDCDriver3PWM(21, 22, 23, 20); */

/* similar for boards with AS5047 and AS5048A */
MagneticSensorSPI _sensor = MagneticSensorSPI(CS_MAGNETIC_SENSOR_PIN, 14, CS_MAGNETIC_SENSOR_ADDRESS);

/* inline current sense: optional, only available on latest boards */
// InlineCurrentSense _current_sense  = InlineCurrentSense(0.05, 50, PC1, PC2);

/* initialize Feelix */
Feelix feelix = Feelix(&_bldc, &_driver, &_sensor, 'A', I2C_State::NO_COMMUNICATION); 

/* initialize Feelix with current sense*/
//Feelix feelix = Feelix(&_bldc, &_driver, &_sensor, &_current_sense);

uint16_t loop_count = 0;
bool endOfCommand = false;




void process_data(char* cmd) {
  // Serial.println((String) "&p " + cmd);
  feelix.process_data(cmd); 
};



/* I2C communication slave*/

static void receiveDataI2C_slave(int n) {
  digitalWrite(feelix.STATUS_LED, HIGH);  
  
  while(1 < Wire.available()) { 
    char c = Wire.read();
    if (!endOfCommand) {
      feelix.I2CincomingData += c;
      if (c == '&') {
        endOfCommand = true;
      }
    }
    
  }

  if (feelix.I2CincomingData != "") {

    Serial.println((String) "&" + feelix.I2CincomingData);


    char* cstr = new char[feelix.I2CincomingData.length() + 1];
    strcpy(cstr, feelix.I2CincomingData.c_str());
    feelix.process_data(cstr);  

    endOfCommand = false;
    feelix.I2CincomingData = "";
  }
  // int x = Wire.read();          
  // Serial.println(x);             
  digitalWrite(feelix.STATUS_LED, LOW);
}


static void requestDataI2C_slave() {
  // Wire.write(); 
  // std::string str = "A" + feelix.id + ":" + feelix.angle + ":" + feelix.velocity + ":" + feelix.current_time + ":" + feelix.target_val;

  // Serial.println((String) "&data request type " + feelix.dataRequestType);
  const char* cstr = feelix.returnDataOnRequest();
  // Serial.println((String) "&data " + cstr);

  if (cstr != NULL) Wire.write(cstr); 
  
}






void setup() {

  delay(2000);

  Serial.begin(115200);

  feelix.init(); 
  // feelix.initI2C();

  analogWriteFrequency(50000);
  
  command.add('F', process_data);

  // Serial.println("HANDSHAKE");

}




void loop() {

  // feelix.searchDevices();
  // delay(5000);

  
  
  feelix.run(loop_count);

  if (loop_count++ > feelix.communication_speed) {

      // feelix.receiveDataI2C();
      feelix.blinkStatusLED();

      if (feelix.INITIALIZED) { // || feelix.nrOfConnectedDevices > 0
        feelix.send_data();
      }

      loop_count = 0;
  }
  
  command.run();
}






void Feelix::initI2C() {

  if (I2C_state == I2C_State::SLAVE) {
    Wire.setSDA(PB7);
    Wire.setSCL(PB6);

    Wire.begin(I2C_address);

    Wire.onReceive(receiveDataI2C_slave);
    Wire.onRequest(requestDataI2C_slave); 
    
  } else if (I2C_state == I2C_State::MASTER){
    Wire.begin();
  }
}

