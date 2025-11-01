 /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
// Peripherals
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
// Input Output
#include "stdio.h"
// Math.abs
#include <stdlib.h>
// Types
#include "stdbool.h"
#include "string.h"


// GPIO
static void MX_GPIO_Init(void);

// Global Variables
// extern void initialise_monitor_handles(void);	// Log for semi-hosting support (printf)
static void UART1_Init(void); // UART Serial RxTx
static UART_HandleTypeDef huart1; // Serial RxTx
static ADC_HandleTypeDef ADC_HandlerLightSensor;
static I2C_HandleTypeDef hi2c;
int BUTTON_DURATION = 600; // 600 tick/ms for GetTick()
static bool buttonActive = false;
static bool isPlayer = false;
static int buttonPressTime;
static bool nearbyFlag = false;
static int nearbyStartTime;

static void UART1_Init(void) {
	/* Pin configuration for UART. BSP_COM_Init() can do
	this automatically */
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = ST_LINK_UART1_RX_Pin | ST_LINK_UART1_TX_Pin;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* Configuring UART1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&huart1) != HAL_OK) {
		while(1);
	}
}

// GPIO
static void MX_GPIO_Init(void) {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE(); // LED 1
	__HAL_RCC_GPIOB_CLK_ENABLE(); // For LED 2
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable AHB2 Bus for GPIOC for Button
	__HAL_RCC_GPIOD_CLK_ENABLE(); // For LSM6DSL

	// LED 1
	GPIO_InitTypeDef GPIO_InitStructLED1 = {0};
	/*Configure GPIO pin ARD_D13_Pin / LED1 */
	GPIO_InitStructLED1.Pin = ARD_D13_Pin;
	GPIO_InitStructLED1.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructLED1.Pull = GPIO_NOPULL;
	GPIO_InitStructLED1.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ARD_D13_GPIO_Port, &GPIO_InitStructLED1);
	HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_RESET);  // configure LED1 to LOW

	// LED 2
	GPIO_InitTypeDef GPIO_InitStructLED2 = {0};
	/*Configure GPIO pin LED2_Pin */
	GPIO_InitStructLED2.Pin = LED2_Pin;
	GPIO_InitStructLED2.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructLED2.Pull = GPIO_NOPULL;
	GPIO_InitStructLED2.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStructLED2);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);  // configure LED2 to LOW

	// BUTTON
	GPIO_InitTypeDef GPIO_InitStructButton = {0};
	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStructButton.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStructButton.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructButton.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStructButton);

	// set INT1 EXTI 11 as interrupt for LSM6DSL
	GPIO_InitTypeDef GPIO_InitStructLSM6DSL = {0};
	GPIO_InitStructLSM6DSL.Pin = LSM6DSL_INT1_EXTI11_Pin;
	GPIO_InitStructLSM6DSL.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructLSM6DSL.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStructLSM6DSL);

	// Enable NVIC EXTI line 13
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	// Buzzer
	GPIO_InitTypeDef GPIO_InitStructBuzzer = {0};
	/* Configuration of ARD_D4_Pin GPIO_PIN_3  ARD_D4_GPIO_Port GPIOA*/
	GPIO_InitStructBuzzer.Pin = ARD_D4_Pin;
	GPIO_InitStructBuzzer.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructBuzzer.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStructBuzzer);
	HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_RESET);

	// Light Sensor
	__HAL_RCC_ADC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructLightSensor = {0};
	GPIO_InitStructLightSensor.Pin = ARD_A0_Pin;
	GPIO_InitStructLightSensor.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStructBuzzer.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_A0_GPIO_Port, &GPIO_InitStructLightSensor);

	ADC_HandlerLightSensor.Instance = ADC1;
	ADC_HandlerLightSensor.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
	ADC_HandlerLightSensor.Init.Resolution            = ADC_RESOLUTION_12B;
	ADC_HandlerLightSensor.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	ADC_HandlerLightSensor.Init.ScanConvMode          = ADC_SCAN_DISABLE;
	ADC_HandlerLightSensor.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	ADC_HandlerLightSensor.Init.ContinuousConvMode    = DISABLE;
	ADC_HandlerLightSensor.Init.NbrOfConversion       = 1;
	ADC_HandlerLightSensor.Init.DiscontinuousConvMode = DISABLE;
	ADC_HandlerLightSensor.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	ADC_HandlerLightSensor.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
	ADC_HandlerLightSensor.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
	ADC_HandlerLightSensor.Init.OversamplingMode      = DISABLE;

	HAL_ADC_Init(&ADC_HandlerLightSensor);

	// Calibrate once after init
	HAL_ADCEx_Calibration_Start(&ADC_HandlerLightSensor, ADC_SINGLE_ENDED);

	// 3) Channel config — ***pick the channel that matches A0***
	ADC_ChannelConfTypeDef Config = {0};
	Config.Channel      = ADC_CHANNEL_14;                 // PC5 → IN14
	Config.Rank         = ADC_REGULAR_RANK_1;
	Config.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;     // long sample (LDR divider)
	Config.SingleDiff   = ADC_SINGLE_ENDED;
	Config.OffsetNumber = ADC_OFFSET_NONE;
	Config.Offset       = 0;
	HAL_ADC_ConfigChannel(&ADC_HandlerLightSensor, &Config);


	// I2C
	__HAL_RCC_I2C1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructOLED = {0};

	// PB8 -> I2C1_SCL, PB9 -> I2C1_SDA
	GPIO_InitStructOLED.Pin       = ARD_D15_Pin | ARD_D14_Pin;
	GPIO_InitStructOLED.Mode      = GPIO_MODE_AF_OD;          // open-drain for I2C
	GPIO_InitStructOLED.Pull      = GPIO_PULLUP;              // needs pull-ups
	GPIO_InitStructOLED.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructOLED.Alternate = GPIO_AF4_I2C1;            // AF4 on L4
	HAL_GPIO_Init(ARD_D14_GPIO_Port, &GPIO_InitStructOLED);

	hi2c.Instance             = I2C1;
	hi2c.Init.Timing          = 0x00707CBB;
	hi2c.Init.OwnAddress1 = 0;
	hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2 = 0;
	hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin) {
		while(1) {
			continue;
		}
	}

	if(GPIO_Pin == BUTTON_EXTI13_Pin) {
		char message_print[32];

		char message[] = "Blue button is pressed\r\n"; // Fixed message
		sprintf(message_print, "%s", message);
		HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);

		if (buttonActive == false) {
			buttonActive = true;
			buttonPressTime = HAL_GetTick();
		} else if (HAL_GetTick()-buttonPressTime > BUTTON_DURATION) {
			buttonActive = false;
		} else {
			buttonActive = false;
			toggleState();
		}


		if (nearbyFlag) {
			if (isPlayer) {
				char message[] = "Player escaped, good job!\r\n"; // Fixed message
				sprintf(message_print, "%s", message);
				HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
			} else {
				char message[] = "Player captured, good job!\r\n"; // Fixed message
				sprintf(message_print, "%s", message);
				HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
			nearbyFlag = false;
		}
	}
}

// Helper Methods
float TemperatureSensorHelper(bool output) {
	float tempData = BSP_TSENSOR_ReadTemp();

	if (output) {
		char tempMessage[32];
		sprintf(tempMessage, "Temp = %.2f deg C \r\n" , tempData);
		HAL_UART_Transmit(&huart1, (uint8_t*)tempMessage, strlen(tempMessage), 0xFFFF);
	}

	return tempData;
}

float PressureSensorHelper(bool output) {
	float pressureData = BSP_PSENSOR_ReadPressure();

	if (output) {
		char pressureMessage[32];
		sprintf(pressureMessage, "Pressure = %.2f \r\n" , pressureData);
		HAL_UART_Transmit(&huart1, (uint8_t*)pressureMessage, strlen(pressureMessage), 0xFFFF);
	}

	return pressureData;
}

float HumiditySensorHelper(bool output) {
	float humidityData = BSP_HSENSOR_ReadHumidity();

	if (output) {
		char humidityMessage[32];
		sprintf(humidityMessage, "Humidity = %.2f \r\n" , humidityData);
		HAL_UART_Transmit(&huart1, (uint8_t*)humidityMessage, strlen(humidityMessage), 0xFFFF);
	}

	return humidityData;
}

float AccelerometerHelper(float* buffer) {
	int16_t accel_data_i16[3] = {0}; // create array of size [3] with values {0} to store xyz reading
	BSP_ACCELERO_AccGetXYZ(accel_data_i16); // returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).

	for (int i = 0; i < 3; i++) {
		buffer[i] = (float)accel_data_i16[i] * (9.8/1000.0f); // mg -> m/s²
	}
	buffer[3] = sqrtf(buffer[0]*buffer[0] + buffer[1]*buffer[1] + buffer[2]*buffer[2]);

	char message_print[64];
	sprintf(message_print, "Accel: X: %.2f m/s², Y: %.2f m/s², Z: %.2f m/s², aggregated= %.2f m/s²\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
	HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);

	return buffer[3];
}

#define MOVING_AVG_SIZE 10  // number of samples for smoothing
float GyroscopeHelper(float* buffer) {
	static float gyroBuffer[3][MOVING_AVG_SIZE] = {0}; // buffers for X, Y, Z
	static int index = 0;                              // circular buffer index
	static int count = 0;                              // number of valid samples
	int16_t gyro_data_i16[3] = {0};
	float gyroData[3];

	BSP_GYRO_GetXYZ(gyro_data_i16);

	// Convert to deg/s
	for (int i = 0; i < 3; i++) {
		gyroData[i] = (float)gyro_data_i16[i] / 1000.0f;
		gyroBuffer[i][index] = gyroData[i];
	}

	// Update circular buffer
	index = (index + 1) % MOVING_AVG_SIZE;
	if (count < MOVING_AVG_SIZE) count++;

	// Compute moving average
	for (int i = 0; i < 3; i++) {
		float sum = 0.0f;
		for (int j = 0; j < count; j++) {
			sum += gyroBuffer[i][j];
		}
		buffer[i] = sum / count;
	}

	// Compute smoothed magnitude
	buffer[3] = sqrtf(buffer[0]*buffer[0] + buffer[1]*buffer[1] + buffer[2]*buffer[2]);

	// Print smoothed data
	char message_print[128];
	sprintf(message_print,
			"Gyro (avg %d): X: %.2f deg/s, Y: %.2f deg/s, Z: %.2f deg/s, magnitude= %.2f deg/s\r\n",
			count, buffer[0], buffer[1], buffer[2], buffer[3]);
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

	return buffer[3];
}

float MagnetometerHelper(float* buffer) {
    int16_t magRaw[3] = {0};
    BSP_MAGNETO_GetXYZ(magRaw);   // raw LSB data from sensor

    const float MAG_SENSITIVITY = 0.15f;  // µT/LSB for LIS3MDL at ±4 gauss
    for (int i = 0; i < 3; i++) {
        buffer[i] = (float)magRaw[i] * MAG_SENSITIVITY;
    }

    // Compute magnitude (total field strength)
    buffer[3] = sqrtf(buffer[0]*buffer[0] + buffer[1]*buffer[1] + buffer[2]*buffer[2]);

    char message_print[80];
    sprintf(message_print, "Magnetic Field (µT): X=%.2f, Y=%.2f, Z=%.2f, |B|=%.2f\r\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    //HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

    return buffer[3];
}

void EnforcerOutput(void) {
	char message_print[16];
	char message[] = "Player Out!\r\n";
	sprintf(message_print, "%s", message);
	HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
}

void LEDBlinkHelper(int modulo) {
	if (HAL_GetTick() % modulo <= 10) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	}
}

uint16_t ReadLightSensorRaw(void) {
    HAL_ADC_Start(&ADC_HandlerLightSensor);
    HAL_ADC_PollForConversion(&ADC_HandlerLightSensor, 10);
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&ADC_HandlerLightSensor);
    HAL_ADC_Stop(&ADC_HandlerLightSensor);
    return val; // 0..4095 for 12-bit
}

bool isLightSensor(void) {
	if (HAL_GPIO_ReadPin(ARD_A0_GPIO_Port, ARD_A0_Pin)) {
		return true;
	}
	return false;
}

void I2C_ScanBus(I2C_HandleTypeDef *hi2c)
{
    for (uint16_t addr = 1; addr < 0x80; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 5) == HAL_OK) {
            // Found a device at 7-bit address 'addr'
            // (Use your own print/log function)
        	char message[16];
			sprintf(message, "I2C device at: 0x%02X\r\n", addr);
			HAL_UART_Transmit(&huart1,(uint8_t*)message, strlen(message),0xFFFF);
        }
    }
}



// Custom Class for Switching
typedef struct {
	const char *name;
	void (*initialise)(void);
	void (*update)(void);
	void (*exit)(void);
} State;

void RedLightGreenLight_initialise(void);
void RedLightGreenLight_update(void);
void RedLightGreenLight_exit(void);
void CatchAndRun_initialise(void);
void CatchAndRun_update(void);
void CatchAndRun_exit(void);

// Implementation for RedLightGreenLight State
void RedLightGreenLight_initialise(void) {
	if (isPlayer) {
		char message[] = "Entering Red Light, Green Light as Player\r\n"; // Fixed message

		char message_print[64];
		sprintf(message_print, "%s", message);
		HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);

	} else {
		char message[] = "Entering Red Light, Green Light as Enforcer\r\n"; // Fixed message

		char message_print[64];
		sprintf(message_print, "%s", message);
		HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
	}
}
void RedLightGreenLight_update(void) {
	static bool isGreen = false;

	// timing shit
	static uint32_t lastToggle = 0;
	static uint32_t lastLEDToggle = 0;

	// compare movement
	static float lastAccel[4] = {0};
	static float lastGyro[4]  = {0};
	static bool lastCaptured = false;

	if (HAL_GetTick() - lastToggle >= 10000) {
        lastToggle = HAL_GetTick();

		char message_print[32]; // UART	transmit buffer.
		if (!isGreen) { // if isGreen =/= true, at the start is false -> true
			char message[] = "Green Light!\r\n";
			sprintf(message_print, "%s", message);
			isGreen = true;

			lastCaptured = false;
		} else {
			char message[] = "Red Light!\r\n";
			sprintf(message_print, "%s", message);
			isGreen = false;
		}

		HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
	}


	if (isGreen) {
		HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		lastCaptured = false;

		if (HAL_GetTick() % 2000 < 10) {
			TemperatureSensorHelper(true);
			PressureSensorHelper(true);
			HumiditySensorHelper(true);
		}
	} else {
		if (!lastCaptured) {
			AccelerometerHelper(lastAccel);
			GyroscopeHelper(lastGyro);
			lastCaptured = true;
		}

		if (HAL_GetTick() - lastLEDToggle >= 500) {
			HAL_GPIO_TogglePin(ARD_D13_GPIO_Port, ARD_D13_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		    lastLEDToggle = HAL_GetTick();
		}

		if (HAL_GetTick() % 2000 < 10) {
			float accelThreshold = 0.8f; // m/s², mild movement
			float gyroThreshold  = 1.5f; // deg/s, mild rotation -> this sensor noise is way too high

			float accelData[4], gyroData[4];
			AccelerometerHelper(accelData);
			GyroscopeHelper(gyroData);

			for (int i; i<4; i++) {
				lastGyro[i] = gyroData[i];
				lastAccel[i] = accelData[i];
			}

			if (isPlayer && (fabs(accelData[3] - lastAccel[3]) > accelThreshold) && (fabs(gyroData[3]  - lastGyro[3]) > gyroThreshold)) {
				endState();
				return;
			} else if (!isPlayer && (fabs(accelData[3] - lastAccel[3]) > accelThreshold) && (fabs(gyroData[3]  - lastGyro[3]) > gyroThreshold)) {
				EnforcerOutput();
			}
		}
	}
}

void RedLightGreenLight_exit(void) {
    char message[] = "--- State Exiting: RedLightGreenLight ---\r\n"; // Fixed message
	char message_print[64];
	sprintf(message_print, "%s", message);
	HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
}

// Implementation for CatchAndRun State
void CatchAndRun_initialise(void) {
    char message_print[80];

    if (isPlayer) {
        sprintf(message_print, "Entering Catch And Run as Player\r\n");
    } else {
        sprintf(message_print, "Entering Catch And Run as Enforcer\r\n");
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

	// Ensure LED starts OFF for controlled blinking
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
void CatchAndRun_update(void) {
	if (HAL_GetTick() % 1000 < 10) {
		// Tested in dorm room
//		Temp = 33.67 deg C
//		Pressure = 1008.35
//		Humidity = 63.11

		float tempThreshold = 35.0f; // m/s², mild movement
		float pressureThreshold  = 1030.0f;
		float humidityThreshold = 80.0;

		float temp = TemperatureSensorHelper(false);
		if (temp > tempThreshold) {
			char tempMessage[80];
			sprintf(tempMessage, "Temperature spike detected! T: %.2fC. Dangerous environment!\r\n" , temp);
			HAL_UART_Transmit(&huart1, (uint8_t*)tempMessage, strlen(tempMessage), 0xFFFF);
		}

		float pressure = PressureSensorHelper(false);
		if (pressure > pressureThreshold) {
			char pressureMessage[80];
			sprintf(pressureMessage, "Pressure spike detected! P: %.1fhPa. Dangerous environment!\r\n" , pressure);
			HAL_UART_Transmit(&huart1, (uint8_t*)pressureMessage, strlen(pressureMessage), 0xFFFF);
		}

		float humidity = HumiditySensorHelper(false);
		if (humidity > humidityThreshold) {
			char humidityMessage[80];
			sprintf(humidityMessage, "Humidity spike detected! H: %.1f%%. Dangerous environment!\r\n" , humidity);
			HAL_UART_Transmit(&huart1, (uint8_t*)humidityMessage, strlen(humidityMessage), 0xFFFF);
		}
	}

	float magnetData[4];
	MagnetometerHelper(magnetData); // constant continuous
	float magnetoThreshold = 400.0f;

	if (isPlayer && magnetData[3] > magnetoThreshold) {
		if (nearbyFlag == false) {
			nearbyStartTime = HAL_GetTick();
			nearbyFlag = true;
		}

		if (HAL_GetTick() % 500 < 10) {
			char magnetoMessage[80];
			sprintf(magnetoMessage, "Enforcer nearby! Be careful.\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)magnetoMessage, strlen(magnetoMessage), 0xFFFF);
		}

	} else if (!isPlayer && magnetData[3] > magnetoThreshold){
		if (nearbyFlag == false) {
			nearbyStartTime = HAL_GetTick();
			nearbyFlag = true;
		}

		if (HAL_GetTick() % 500 < 10) {
			char magnetoMessage[80];
			sprintf(magnetoMessage, "Player is Nearby! Move faster.\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)magnetoMessage, strlen(magnetoMessage), 0xFFFF);
		}
	}

	if (HAL_GetTick() - nearbyStartTime > 3000 && isPlayer && nearbyFlag) {
		endState();
	} else if (HAL_GetTick() - nearbyStartTime > 3000 && !isPlayer && nearbyFlag) {
		nearbyFlag = false;
		char chaseMessage[64];
		sprintf(chaseMessage, "Player escaped! Keep trying.\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)chaseMessage, strlen(chaseMessage), 0xFFFF);
	}

	// LED Blinking
	if (magnetData[3] > magnetoThreshold && magnetData[3] < (magnetoThreshold + 100)) {
		LEDBlinkHelper(600);
	} else if (magnetData[3] > (magnetoThreshold + 100) && magnetData[3] < (magnetoThreshold + 200)) {
		LEDBlinkHelper(200);
	} else if (magnetData[3] > (magnetoThreshold + 200)) {
		LEDBlinkHelper(50);
	}
}
void CatchAndRun_exit(void) {
    char message[] = "--- State Exiting: CatchAndRun ---\r\n"; // Fixed message
	char message_print[64];
	sprintf(message_print, "%s", message);
	HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
}

// State variable
State* currentState;         // Pointer
// Define the two states
State RedLightGreenLightState = {
    .name = "RedLightGreenLight",
    .initialise = RedLightGreenLight_initialise,
    .update = RedLightGreenLight_update,
    .exit = RedLightGreenLight_exit
};

State CatchAndRunState = {
    .name = "CatchAndRun",
    .initialise = CatchAndRun_initialise,
    .update = CatchAndRun_update,
    .exit = CatchAndRun_exit
};

// Running flag
volatile bool programRunning = true;

void toggleState(void) {
	if (currentState == &RedLightGreenLightState) {
		currentState->exit();
		currentState = &CatchAndRunState;
	} else if (currentState == &CatchAndRunState) {
		currentState->exit();
		currentState = &RedLightGreenLightState;
	}

	currentState->initialise();
}

void endState(void) {
	char message_print[16];
	char message[] = "Game Over\r\n";
	sprintf(message_print, "%s", message);
	HAL_UART_Transmit(&huart1,(uint8_t*)message_print, strlen(message_print),0xFFFF);
	programRunning = false;
}


void setup(void) {
	// initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Peripheral initializations using BSP functions */
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	// GPIO Pins
	MX_GPIO_Init();
	SENSOR_IO_Write(0xD4, 0x0D, 0x03);

	/* UART initialization */
	UART1_Init();

	I2C_ScanBus(&hi2c);

	// My state machine
	currentState = &RedLightGreenLightState;
	currentState->initialise();
}


// Main Program Code
int main(void) {
	setup();

	while (programRunning) {
		currentState->update();
	}
}
