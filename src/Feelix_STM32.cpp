#include "Feelix.h"

#if defined(_STM32_DEF_)


void SystemClock_Config(void);

void Feelix::init() {
  
    STATUS_LED = PB13;
    
    pinMode(STATUS_LED, OUTPUT);
    SystemClock_Config();
    
    sensor->init();     
    bldc->linkSensor(sensor);

    angle = bldc->shaftAngle(); //(180 / 3.14159);
    rotation_dir = Direction::CW;
    
    driver->pwm_frequency = 50000;

    driver->init();

    driver->enable();

    bldc->linkDriver(driver);

    bldc->controller = MotionControlType::torque;
    bldc->torque_controller = TorqueControlType::voltage;
    // bldc->foc_modulation = FOCModulationType::SpaceVectorPWM;
    
    bldc->voltage_sensor_align = 5;

    bldc->PID_velocity.P = 0.5;
    bldc->PID_velocity.I = 10.0;
    bldc->PID_velocity.D = 0.0;
    bldc->PID_velocity.output_ramp = 1000;
    bldc->LPF_velocity.Tf = 0.01;

    bldc->P_angle.P = 14.0;
    bldc->P_angle.I = 0.0;
    bldc->P_angle.D = 0.0;
    bldc->P_angle.output_ramp = 10000;

    bldc->velocity_limit = 22.0;
    bldc->voltage_limit = 12.0;
    vol_limit = 12.0;
    driver->voltage_limit = 12.0;

    bldc->init(); 
}

/* system clock STM32F401RCT6 */
void SystemClock_Config() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  __HAL_RCC_TIM10_CLK_ENABLE();
  
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
/* system clock STM32F303RCT7 */
// void SystemClock_Config(void) {

//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
//                               |RCC_PERIPHCLK_ADC12;
//   PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
//   PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
//   PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
//   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

#endif 
