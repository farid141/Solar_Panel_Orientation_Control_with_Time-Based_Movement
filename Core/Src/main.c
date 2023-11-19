/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TJ_MPU6050.h"
#include "ds1307_for_stm32_hal.h"
#include "math.h"
#include "i2c-lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
RawData_Def myRawAccel;
ScaledData_Def myScaledAccel;
float roll; //roll=rotasi x
float pitch;

//float jadwal[7][2]= {
//    {9, -129.84},
//    {10, -144.14},
//    {11, -167.33},
//    {12, 163.63},
//    {13, 142.87},
//    {14, 128.23},
//    {15, 120.31}
//};

float jadwal[7][2]= {
    {9, -129.84},
    {10, -144.14},
    {11, -167.33},
    {12, 163.63},
    {13, 142.87},
    {14, 128.23},
    {15, 120.31}
};

uint8_t jam, menit, detik, i;
float setpoint_sudut, delta_sudut;

char buffer[50];
unsigned long prev_time;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    MPU_ConfigTypeDef myMpuConfig;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
	void stepper_CCW(void); 
	void stepper_CW(void);
	void printLCD(void);

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    //INISIALISASI MPU050
    //1. Initialise the MPU6050 module and I2C
    MPU6050_Init(&hi2c1);
    //2. Configure Accel and Gyro parameters
    myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
    myMpuConfig.ClockSource = Internal_8MHz;
    myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
    myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
    myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
    MPU6050_Config(&myMpuConfig);

    //INISIALISASI DS1307
    DS1307_Init(&hi2c1);

    /* Lookup table for the days of week. */
    const char *DAYS_OF_WEEK[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
    /* Start DS1307 timing. Pass user I2C handle pointer to function. */
    DS1307_Init(&hi2c1);
    HAL_Delay(200);

    //setting waktu jika diperlukan
//	DS1307_SetHour(16);
//	DS1307_SetMinute(19);
//	DS1307_SetSecond(25);

//    jam = DS1307_GetHour();
//    menit= DS1307_GetMinute();
//    detik= DS1307_GetSecond();

    //INISIALISASI LCD
	//LCD
	HAL_Delay(50);
	lcd_init();
	HAL_Delay(50);
	
	lcd_clear();
	lcd_send_cmd(0x80|0x00);//  lcd_setCursor(0,0);
	int printLen = snprintf((char*)buffer, 50, "Voltage Unbalance");
	lcd_send_string((char*)buffer);
	HAL_Delay(1000);


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
		
		HAL_Delay(10);
		jam = DS1307_GetHour();
		menit= DS1307_GetMinute();
		detik= DS1307_GetSecond();
		
		HAL_Delay(10);
		if((HAL_GetTick()-prev_time) > 1000){
			printLCD();
			prev_time = HAL_GetTick();
		}
		HAL_Delay(10);
        //BACA IMU
		//ACCELERO READ
		MPU6050_Get_Accel_Scale(&myScaledAccel);
		roll = (atan2(myScaledAccel.y, myScaledAccel.z)*180.0)/3.14; // rotasi pada sumbu x
		
		switch(jam){
			case 9:
				setpoint_sudut = 129.84;
				break;
			case 10:
				setpoint_sudut = -144;
				break;
			case 11:
				setpoint_sudut = -167;
				break;
			case 12:
				
				setpoint_sudut = 163;
				break;
			case 13:
				setpoint_sudut = 142;
				break;
			case 14:
				setpoint_sudut = 128;
				break;
			case 15:
				setpoint_sudut = -120;
				break;
			default:
				setpoint_sudut = -120;
				break;
		}
		delta_sudut = setpoint_sudut - roll;
		
		if((jam ==12|jam==13|jam==14|jam==15) && (roll <0)){
			stepper_CW();
		}else if ((jam ==9|jam==10|jam==11)&& (roll >0)){
			stepper_CCW();
		}else{
			if(fabs(delta_sudut)>4){
				if(delta_sudut>0){
					stepper_CCW();
				}else{
					stepper_CW();
				}
			}
		}
		HAL_Delay(10);
//        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
//        for(int i=0; i<200; i++) {
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
//            HAL_Delay(2);
//            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
//            HAL_Delay(2);
//        }


//		//BACA RTC
//		jam = DS1307_GetHour();
//        float errorSudut = sudutJadwal - roll;

//		//CARI JADWAL BERDASARKAN JAM
//        int panjangJadwal = sizeof(jadwal)/sizeof(jadwal[0]);
//        for(int i=0; i<panjangJadwal; i++) {
//            if(jam == jadwal[i][0]) {
//                sudutJadwal = jadwal[i][1];
//                break;
//            }
//        }

//        errorSudut = sudutJadwal - roll;
//        while((errorSudut>sudutJadwal+2) ) {
//        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                      |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pins : PA2 PA3 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PD10 PD11 PD12 PD13
                             PD14 PD15 */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void stepper_CW() {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
    HAL_Delay(2);
}

void stepper_CCW() {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
    HAL_Delay(2);
}

void printLCD(){
	int printLen;
	lcd_clear();
	lcd_send_cmd(0x80|0x00);//  lcd_setCursor(0,0);
	printLen = snprintf((char*)buffer, 50, "waktu: %d:%d:%d", jam, menit, detik);
	lcd_send_string((char*)buffer);
	
	lcd_send_cmd(0x80|0x40);//  lcd_setCursor(0,1);
	printLen = snprintf((char*)buffer, 50, "sudut targegt:%f", setpoint_sudut);
	lcd_send_string((char*)buffer);
	
	lcd_send_cmd(0x80|0x14);//	lcd_setCursor(0,2);
	printLen = snprintf((char*)buffer, 50, "sudut: %f", roll);
  lcd_send_string((char*)buffer);
	
	lcd_send_cmd(0x80|0x54);//	lcd_setCursor(0,3);
	printLen = snprintf((char*)buffer, 50, "test4");
  lcd_send_string((char*)buffer);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
