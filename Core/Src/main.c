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
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_nfctag.h"
// Input Output
#include "stdio.h"
// Math.abs
#include <stdlib.h>
// Types
#include "stdbool.h"
#include "string.h"


static void MX_GPIO_Init(void); // GPIO
static void UART1_Init(void); // UART Serial RxTx
// extern void initialise_monitor_handles(void);	// Log for semi-hosting support (printf)
/* Global Variables */
static UART_HandleTypeDef huart1; // Serial RxTx
static ADC_HandleTypeDef ADC_HandlerLightSensor;
#define HT16K33_ADDR (0x70 << 1) // LED MATRIX
/* Grove - OLED Display 0.96inch - SSD1308 - width in pixels */
#define SSD1308_WIDTH 128
/* SSD1308 height in pixels */
#define SSD1308_HEIGHT 64
/* SSD1308 I2C Address */
#define SSD1308_ADDR (0x3C << 1) // 0x3C is 7bits but HAL expects 8bits

/* I2C_SPEED_HZ 4000000 //The same as System Clock, PCLK1_FREQ_HZ 4000000*/
static I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef hi2c2;

/* NFC (8-bit is enough) */
#define NFCTAG_IT_FIELDFALLING  ((uint8_t)0x08)  // RF field OFF
#define NFCTAG_IT_FIELDRISING   ((uint8_t)0x10)  // RF field ON

#define GROVE5_ADDR (0x03 << 1) // 5-Way Switch


int BUTTON_DURATION = 600; // 600 tick/ms for GetTick()
static bool buttonActive = false;
static bool isPlayer = false;
static int buttonPressTime;
static bool nearbyFlag = false;
static int nearbyStartTime;

volatile uint32_t g_nfc_it_flags = 0;
volatile uint32_t g_nfc_last_irq_ms = 0;
volatile uint8_t  g_nfc_rf_state = 0;      // 0=OFF, 1=ON
static   uint32_t g_nfc_last_report_ms = 0; // for UART throttle
float accelData[4], gyroData[4];

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
	    HAL_UART_Transmit(&huart1, (uint8_t*)"UART Init Failed\r\n", sizeof("UART Init Failed\r\n") - 1, 100);
		while(1);
	}
}

// GPIO
static void MX_GPIO_Init(void) {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE(); // LED 1
	__HAL_RCC_GPIOB_CLK_ENABLE(); // For LED 2
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable AHB2 Bus for GPIOC for Button
	__HAL_RCC_GPIOD_CLK_ENABLE(); // For LSM6DSL
	__HAL_RCC_GPIOE_CLK_ENABLE(); // For NFC Interrupt

	// LED 1
	GPIO_InitTypeDef GPIO_InitStructLED1 = {0};
	/* Configure GPIO pin ARD_D13_Pin / LED1 */
	GPIO_InitStructLED1.Pin = ARD_D13_Pin;
	GPIO_InitStructLED1.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructLED1.Pull = GPIO_NOPULL;
	GPIO_InitStructLED1.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ARD_D13_GPIO_Port, &GPIO_InitStructLED1);
	HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_RESET);  // configure LED1 to LOW

	// LED 2
	GPIO_InitTypeDef GPIO_InitStructLED2 = {0};
	/* Configure GPIO pin LED2_Pin */
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
	GPIO_InitStructLSM6DSL.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructLSM6DSL.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStructLSM6DSL);


	// Buzzer
	GPIO_InitTypeDef GPIO_InitStructBuzzer = {0};
	/* Configuration of ARD_D4_Pin GPIO_PIN_3  ARD_D4_GPIO_Port GPIOA*/
	GPIO_InitStructBuzzer.Pin = ARD_D4_Pin;
	GPIO_InitStructBuzzer.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructBuzzer.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStructBuzzer);
	HAL_GPIO_WritePin(ARD_D4_GPIO_Port, ARD_D4_Pin, GPIO_PIN_RESET);

	/* // Light Sensor
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
	HAL_ADC_ConfigChannel(&ADC_HandlerLightSensor, &Config); */

	/* --- I2C1 GPIO ---
	 * ARD_D15_Pin GPIO_PIN_8 // PB8 -> I2C1_SCL
	 * ARD_D15_GPIO_Port GPIOB
	 * ARD_D14_Pin GPIO_PIN_9 // PB9 -> I2C1_SDA
	 * ARD_D14_GPIO_Port GPIOB */
	__HAL_RCC_I2C1_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructI2C1 = {0};
	GPIO_InitStructI2C1.Pin       = ARD_D15_Pin | ARD_D14_Pin;
	GPIO_InitStructI2C1.Mode      = GPIO_MODE_AF_OD;          // open-drain for I2C
	GPIO_InitStructI2C1.Pull      = GPIO_PULLUP;              // needs pull-ups
	GPIO_InitStructI2C1.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructI2C1.Alternate = GPIO_AF4_I2C1;            // AF4 on L4
	HAL_GPIO_Init(ARD_D14_GPIO_Port, &GPIO_InitStructI2C1);
	/* --- I2C1 Config --- */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00320F13; // TIMINGR = (PRESC<<28) | (SCLDEL<<20) | (SDADEL<<16) | (SCLH<<8) | SCLL
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Master sends a signal to all I2C devices
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Disabling so slow slave to hold the clock line low to extend the clock pulse duration, giving it more time to process data.
	HAL_I2C_Init(&hi2c1);

	/* --- I2C2 GPIO ---
	 * INTERNAL_I2C2_SCL_Pin GPIO_PIN_10
	 * INTERNAL_I2C2_SCL_GPIO_Port GPIOB
	 * INTERNAL_I2C2_SDA_Pin GPIO_PIN_11
	 * INTERNAL_I2C2_SDA_GPIO_Port GPIOB */
	__HAL_RCC_I2C2_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructI2C2 = {0};
	GPIO_InitStructI2C2.Pin       = INTERNAL_I2C2_SCL_Pin | INTERNAL_I2C2_SDA_Pin;
	GPIO_InitStructI2C2.Mode      = GPIO_MODE_AF_OD;          // open-drain for I2C
	GPIO_InitStructI2C2.Pull      = GPIO_PULLUP;              // needs pull-ups
	GPIO_InitStructI2C2.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructI2C2.Alternate = GPIO_AF4_I2C2;            // AF4 on L4
	HAL_GPIO_Init(INTERNAL_I2C2_SCL_GPIO_Port, &GPIO_InitStructI2C2);
	/* --- I2C2 Config --- */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00320F13; // TIMINGR = (PRESC<<28) | (SCLDEL<<20) | (SDADEL<<16) | (SCLH<<8) | SCLL
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Master sends a signal to all I2C devices
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Disabling so slow slave to hold the clock line low to extend the clock pulse duration, giving it more time to process data.
	HAL_I2C_Init(&hi2c2);

	/* --- NFC GPO pin (interrupt from ST25DV) ---
	 * ST25DV04K_GPO_Pin GPIO_PIN_4
	 * ST25DV04K_GPO_GPIO_Port GPIOE*/
	GPIO_InitTypeDef GPIO_InitStructGPO = {0};
	GPIO_InitStructGPO.Pin = ST25DV04K_GPO_Pin;
	GPIO_InitStructGPO.Mode = GPIO_MODE_IT_FALLING;   // GPO often active-low pulses
	GPIO_InitStructGPO.Pull = GPIO_PULLUP;            // depends on board; pull-up is usually correct
	HAL_GPIO_Init(ST25DV04K_GPO_GPIO_Port, &GPIO_InitStructGPO);

	/* --- NVIC EXTI Interrupt --- */
	//__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_EXTI13_Pin);
	//__HAL_GPIO_EXTI_CLEAR_IT(ST25DV04K_GPO_Pin);
	// Enable NVIC EXTI line 0 - 15
	HAL_NVIC_SetPriority(BUTTON_EXTI13_EXTI_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(BUTTON_EXTI13_EXTI_IRQn);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	// GPO Interrupt
//	ST25DV_ITStatus_Dyn it;
//	ST25DV_GPOConfig cfg = {0};
//	cfg.RFUserDetect = 1;      // or RFFieldChange / MailboxMsg, depending on your header
//	ST25DV_WriteGPOConfig(&NfcTagObj, &cfg);
}

void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(ST25DV04K_GPO_Pin);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == ST25DV04K_GPO_Pin) {
		uint32_t now = HAL_GetTick();
		if (now - g_nfc_last_irq_ms < 100) return;   // debounce
		g_nfc_last_irq_ms = now;

		uint8_t dyn = 0;
		if (BSP_NFCTAG_ReadITSTStatus_Dyn(0, &dyn) != NFCTAG_OK) return; // <-- READ IT STATUS (clears)

		g_nfc_it_flags |= dyn;                      // pass bits to main loop

		if (dyn & NFCTAG_IT_FIELDRISING)  g_nfc_rf_state = 1; // RF ON
		if (dyn & NFCTAG_IT_FIELDFALLING) g_nfc_rf_state = 0; // RF OFF
	}


	if(GPIO_Pin == BUTTON_EXTI13_Pin) {
	    HAL_UART_Transmit(&huart1, (uint8_t*)"Blue button is pressed\r\n", sizeof("Blue button is pressed\r\n") - 1, 100);

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
			    HAL_UART_Transmit(&huart1, (uint8_t*)"Player escaped, good job!\r\n", sizeof("Player escaped, good job!\r\n") - 1, 100);
			} else {
			    HAL_UART_Transmit(&huart1, (uint8_t*)"Player captured, good job!\r\n", sizeof("Player captured, good job!\r\n") - 1, 100);
			}
			nearbyFlag = false;
		}
	}
}

static void I2C_Scan(I2C_HandleTypeDef *hi2c, const char *tag) {
  char m[64];
  int n = sprintf(m, "\r\n--- %s ---\r\n", tag);
  HAL_UART_Transmit(&huart1,(uint8_t*)m,n,0xFFFF);
  for (uint8_t a=1; a<0x7F; a++) {
    if (HAL_I2C_IsDeviceReady(hi2c, a<<1, 2, 5)==HAL_OK) {
      n = sprintf(m, "I2C ACK @ 0x%02X\r\n", a);
      HAL_UART_Transmit(&huart1,(uint8_t*)m,n,0xFFFF);
    }
  }
}

void rf_poll_debug(void) {
    uint8_t rf_on = 0;
    if (BSP_NFCTAG_GetRFField_Dyn(0, &rf_on) == NFCTAG_OK) {
        if (rf_on) HAL_UART_Transmit(&huart1,(uint8_t*)"RF=ON\r\n",7,100);
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
    HAL_UART_Transmit(&huart1, (uint8_t*)"Player Out!\r\n", sizeof("Player Out!\r\n") - 1, 100);

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

void OLED_ON(void) {
    uint8_t buf[] = {0x00, 0xAF};
    HAL_I2C_Master_Transmit(&hi2c1, SSD1308_ADDR, buf, sizeof(buf), 100 /*ms timeout*/);
}

static HAL_StatusTypeDef oled_write_data(const uint8_t *data, uint16_t len) {
    // We prepend the data control byte (0x40)
    // For large transfers, you can chunk it; this small helper sends in one go if len is modest.
    HAL_StatusTypeDef st;
    // stack-friendly small chunking
    uint16_t offset = 0;
    while (offset < len) {
        uint16_t chunk = (len - offset);
        if (chunk > 16) chunk = 16; // conservative chunk to avoid big I2C bursts
        uint8_t buf[1 + 16];
        buf[0] = 0x40;
        memcpy(&buf[1], &data[offset], chunk);
        st = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SSD1308_ADDR, buf, (uint16_t)(1 + chunk), 100 /* I2C_TIMEOUT_MS */);
        if (st != HAL_OK) return st;
        offset += chunk;
    }
    return HAL_OK;
}
static HAL_StatusTypeDef oled_write_cmd(uint8_t cmd) {
    uint8_t b[2] = {0x00, cmd };
    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SSD1308_ADDR, b, sizeof(b), 100 /* I2C_TIMEOUT_MS */);
}

static HAL_StatusTypeDef oled_write_cmd2(uint8_t cmd0, uint8_t cmd1) {
    uint8_t b[3] = {0x00, cmd0, cmd1 };
    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)SSD1308_ADDR, b, sizeof(b), 100 /* I2C_TIMEOUT_MS */);
}

static HAL_StatusTypeDef oled_clear() {
    HAL_StatusTypeDef st;

    // Make sure it shows RAM (not forced all-pixels-on) and normal polarity
    if ((st = oled_write_cmd(0xA4)) != HAL_OK) return st; // Resume RAM content display
    if ((st = oled_write_cmd(0xA6)) != HAL_OK) return st; // Normal (not inverted)

    // Horizontal addressing; full window
    if ((st = oled_write_cmd2(0x20, 0x00)) != HAL_OK) return st; // Addressing mode = horizontal
    if ((st = oled_write_cmd2(0x21, 0x00)) != HAL_OK) return st; // Column start
    if ((st = oled_write_cmd(127)) != HAL_OK) return st;         // Column end (0..127)
    if ((st = oled_write_cmd2(0x22, 0x00)) != HAL_OK) return st; // Page start
    if ((st = oled_write_cmd(7)) != HAL_OK) return st;           // Page end (0..7 for 64px)

    // Stream zeros across the whole screen: 128 cols × 8 pages = 1024 bytes
    uint8_t zeros[16] = {0};  // small chunk buffer
    for (int i = 0; i < (128 * 8) / (int)sizeof(zeros); i++) {
        st = oled_write_data(zeros, sizeof(zeros));  // prepends 0x40 inside helper
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

static uint8_t g_fb[SSD1308_WIDTH * (SSD1308_HEIGHT/8)];   // 1,024 bytes
static const uint8_t font5x7[][5] = {
    // ' ' (32)
    {0x00,0x00,0x00,0x00,0x00},
    // '0'..'9' (48..57)
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    // 'A'..'Z' (65..90)
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x7F,0x20,0x18,0x20,0x7F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    // '-' '.' ':' '^' '/'
	{0x08,0x08,0x08,0x08,0x08}, // '-'
	{0x00,0x60,0x60,0x00,0x00}, // '.'
	{0x00,0x36,0x36,0x00,0x00}, // ':'
	{0x04,0x02,0x01,0x02,0x04}, // '^'
	{0x20,0x10,0x08,0x04,0x02}, // '/'
	{0x06,0x09,0x09,0x06,0x00}, // '°'
	{0x61,0x12,0x08,0x24,0x43}, // '%'
	{0x24,0x52,0x4A,0x24,0x50}, // '&'

};

static inline const uint8_t* glyph_for(char c) {
    if (c == ' ') return font5x7[0];
    if (c >= '0' && c <= '9') return font5x7[1 + (c - '0')];
    if (c >= 'A' && c <= 'Z') return font5x7[11 + (c - 'A')];

    int puncBase = 11 + 26;  // index of '-'

    if (c == '-') return font5x7[puncBase + 0];
    if (c == '.') return font5x7[puncBase + 1];
    if (c == ':') return font5x7[puncBase + 2];
    if (c == '^') return font5x7[puncBase + 3];
    if (c == '/') return font5x7[puncBase + 4];
    if ((unsigned char)c == 0xB0) return font5x7[puncBase + 5]; // '°'
    if (c == '%') return font5x7[puncBase + 6];
    if (c == '&') return font5x7[puncBase + 7];

    // Fallback: blank
    return font5x7[0];
}

static void fb_set_pixel(int x, int y, int on) {
    if (x < 0 || x >= SSD1308_WIDTH || y < 0 || y >= SSD1308_HEIGHT) return;
    int page = y >> 3;
    uint8_t bit = 1u << (y & 7);
    uint8_t *b = &g_fb[page * SSD1308_WIDTH + x];
    if (on) *b |= bit;
    else    *b &= (uint8_t)~bit;
}

static void fb_clear(void) {
    memset(g_fb, 0x00, sizeof(g_fb));
}

// Draw a 5x7 glyph with 1px spacing; returns advance (6 px)
static int fb_draw_char(int x, int y, char c) {
    const uint8_t *g = glyph_for(c);
    for (int col = 0; col < 5; col++) {
        uint8_t colbits = g[col]; // bit0 = top pixel
        for (int row = 0; row < 7; row++) {
            int on = (colbits >> row) & 1;
            fb_set_pixel(x + col, y + row, on);
        }
    }
    // 1 column spacing
    for (int row = 0; row < 7; row++) fb_set_pixel(x + 5, y + row, 0);
    return 6;
}
static void fb_draw_text(int x, int y, const char *s) {
    int cx = x;
    while (*s) {
        if (*s == '\n') { y += 8; cx = x; s++; continue; }
        cx += fb_draw_char(cx, y, *s++);
        if (cx > SSD1308_WIDTH - 6) { y += 8; cx = x; } // wrap to next row
        if (y > SSD1308_HEIGHT - 8) break;
    }
}
static HAL_StatusTypeDef oled_flush_full(void) {
    HAL_StatusTypeDef st;
    // Follow RAM & normal polarity (safety)
    if ((st = oled_write_cmd(0xA4)) != HAL_OK) return st;
    if ((st = oled_write_cmd(0xA6)) != HAL_OK) return st;
    // Horizontal addressing; full window
    if ((st = oled_write_cmd2(0x20, 0x00)) != HAL_OK) return st; // horizontal
    if ((st = oled_write_cmd2(0x21, 0x00)) != HAL_OK) return st; // col start
    if ((st = oled_write_cmd(SSD1308_WIDTH - 1)) != HAL_OK) return st;  // col end
    if ((st = oled_write_cmd2(0x22, 0x00)) != HAL_OK) return st; // page start
    if ((st = oled_write_cmd((SSD1308_WIDTH/8) - 1)) != HAL_OK) return st; // page end

    // Send whole buffer in small chunks
    // (Each oled_write_data prepends control byte 0x40)
    for (int off = 0; off < (int)sizeof(g_fb); ) {
        int chunk = 16;
        if (off + chunk > (int)sizeof(g_fb)) chunk = (int)sizeof(g_fb) - off;
        st = oled_write_data(&g_fb[off], (uint16_t)chunk);
        if (st != HAL_OK) return st;
        off += chunk;
    }
    return HAL_OK;
}

// Game 1: Red Light / Green Light
static void OLED_Show_RLGL(bool isGreen, bool isPlayer, float accelMag, float gyroMag) {
	char line[32];

	fb_clear();
    // ----- Header (centered) -----
	fb_draw_text(0,  0, "--------------------"); // top line
    fb_draw_text(16, 8, "GAME 1 : RLGL");        // title
    fb_draw_text(0,  16, "--------------------"); // bottom line
    // ----- Role & Phase -----
    sprintf(line, "ROLE : %s", isPlayer ? "PLAYER" : "ENFORCER");
    fb_draw_text(0, 24, line);

    sprintf(line, "PHASE: %s", isGreen ? "GREEN" : "RED");
    fb_draw_text(0, 32, line);

    if (isGreen) {
		// ----- Show environment in GREEN phase -----
		sprintf(line, "TEM : %4.1f \xB0""C", TemperatureSensorHelper(true));
		fb_draw_text(0, 40, line);

        sprintf(line, "HUM : %4.1f %%", HumiditySensorHelper(true));
        fb_draw_text(0, 48, line);

		sprintf(line, "PRE : %5.1f HPA", PressureSensorHelper(true));
		fb_draw_text(0, 56, line);


    } else {
        // ----- Show motion in RED phase -----
        sprintf(line, "ACC  : %4.1f M/S^2", accelMag);
        fb_draw_text(0, 40, line);

        sprintf(line, "GYRO : %4.1f DEG/SEC", gyroMag);
        fb_draw_text(0, 48, line);
    }

    oled_flush_full();
}

// Game 2: Catch & Run
static void OLED_Show_CatchRun(bool isPlayer, float magField) {
	char line[32];

	fb_clear();

    // thresholds (adjust as needed)
    const float MAG_VERY_NEAR = 600.0f;
    const float MAG_NEAR      = 400.0f;

    // ----- Header -----
    fb_draw_text(0,  0, "---------------------");
    fb_draw_text(6,  8, "GAME 2 : CATCH & RUN");
    fb_draw_text(0, 16, "---------------------");

    // ----- Role -----
    sprintf(line, "ROLE : %s", isPlayer ? "PLAYER" : "ENFORCER");
    fb_draw_text(0, 24, line);

    // ----- Proximity status -----
    const char *status;
    if (magField > MAG_VERY_NEAR) {
        status = "VERY NEAR";
    } else if (magField > MAG_NEAR) {
        status = "NEAR";
    } else {
        status = "FAR";
    }

    sprintf(line, "STATUS: %s", status);
    fb_draw_text(0, 32, line);

    // ----- Magnetometer magnitude -----
    sprintf(line, "MAG: %4.1f T", magField);
    fb_draw_text(0, 40, line);

    // ----- Environment (2 lines) -----
    sprintf(line, "TEM: %4.1f\xB0""C HUM: %3.0f%%", TemperatureSensorHelper(true), HumiditySensorHelper(true));
    fb_draw_text(0, 48, line);

    sprintf(line, "PRE: %5.1f hPa", PressureSensorHelper(true));
    fb_draw_text(0, 56, line);

    oled_flush_full();
}


#define HT16K33_CMD_OSC_ON  0x21
#define HT16K33_CMD_DISPLAY 0x80          // + blink rate << 1 | display on
#define HT16K33_DISPLAY_ON  0x01
#define HT16K33_CMD_DIM     0xE0          // + brightness (0-15)

static inline void HT16K33_Init(uint8_t brightness /* 0..15 */, uint8_t blink /* 0..3 */)
{
    uint8_t cmd;

    cmd = HT16K33_CMD_OSC_ON;
    HAL_I2C_Master_Transmit(&hi2c1, HT16K33_ADDR, &cmd, 1, HAL_MAX_DELAY);

    cmd = HT16K33_CMD_DISPLAY | (blink << 1) | HT16K33_DISPLAY_ON;
    HAL_I2C_Master_Transmit(&hi2c1, HT16K33_ADDR, &cmd, 1, HAL_MAX_DELAY);

    cmd = HT16K33_CMD_DIM | (brightness & 0x0F);
    HAL_I2C_Master_Transmit(&hi2c1, HT16K33_ADDR, &cmd, 1, HAL_MAX_DELAY);
}

// Write an 8x8 frame using 16-bit rows (LSB sent first for each row).
// rows[0] is first row; bit0 = column 0. Only low 8 bits are used by mono 8x8.
static inline HAL_StatusTypeDef HT16K33_WriteRows16(const uint16_t rows[8])
{
    uint8_t buf[1 + 16];
    buf[0] = 0x00; // start at display RAM address 0x00

    for (uint8_t r = 0; r < 8; r++) {
        buf[1 + (r * 2) + 0] = (uint8_t)(rows[r] & 0xFF);
        buf[1 + (r * 2) + 1] = (uint8_t)((rows[r] >> 8) & 0xFF);
    }
    return HAL_I2C_Master_Transmit(&hi2c1, HT16K33_ADDR, buf, sizeof(buf), HAL_MAX_DELAY);
}

// Convenience: write using 8-bit rows (mono 8x8). Each byte is a row bitmap.
static inline HAL_StatusTypeDef HT16K33_WriteRows8(const uint8_t rows[8])
{
    uint8_t buf[1 + 16];
    buf[0] = 0x00; // start at display RAM address 0x00

    for (uint8_t r = 0; r < 8; r++) {
        buf[1 + (r * 2) + 0] = rows[r];  // low byte (columns 0..7)
        buf[1 + (r * 2) + 1] = 0x00;     // high byte (columns 8..15 unused)
    }
    return HAL_I2C_Master_Transmit(&hi2c1, HT16K33_ADDR, buf, sizeof(buf), HAL_MAX_DELAY);
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

			 // Show simple RLGL screen (no movement yet)
			OLED_Show_RLGL(true, isPlayer,0.0f, 0.0f);
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


		// Red Light


		if (HAL_GetTick() % 2000 < 10) {
			float accelThreshold = 0.8f; // m/s², mild movement
			float gyroThreshold  = 1.5f; // deg/s, mild rotation -> this sensor noise is way too high


			AccelerometerHelper(accelData);
			GyroscopeHelper(gyroData);

			// NEW: update OLED with magnitudes (index 3)
			OLED_Show_RLGL(false, isPlayer,accelData[3], gyroData[3]);


			for (int i = 0; i<4; i++) {
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

	// NEW: show Catch & Run info every loop
	OLED_Show_CatchRun(isPlayer, magnetData[3]);

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
	HAL_UART_Transmit(&huart1, (uint8_t*)"Game Over\r\n", sizeof("Game Over\r\n") - 1, 100);
	programRunning = false;
}


void setup(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	MX_GPIO_Init(); // GPIO Pins
	/* UART initialization */
	UART1_Init();
	// initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Peripheral initializations using BSP functions */
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	if (BSP_NFCTAG_Init(0) != NFCTAG_OK) {
	    HAL_UART_Transmit(&huart1, (uint8_t*)"NFC Init Failed\r\n", sizeof("NFC Init Failed\r\n") - 1, 100);
	} else {
	    HAL_UART_Transmit(&huart1, (uint8_t*)"NFC Init OK\r\n", sizeof("NFC Init OK\r\n") - 1, 100);
	}

	ST25DV_PASSWD pwd = { .MsbPasswd = 0, .LsbPasswd = 0 };
	(void)BSP_NFCTAG_PresentI2CPassword(0, pwd);   // required for static writes

	// 1) Set GPO pulse width (pick an enum your header defines)
	(void)BSP_NFCTAG_WriteITPulse(0, (ST25DV_PULSE_DURATION)2);

	// 2) Enable only the RF field events you want on GPO
	uint16_t itmask = 0;
	itmask |= NFCTAG_IT_FIELDRISING;   // RF ON
	itmask |= NFCTAG_IT_FIELDFALLING;  // RF OFF (optional)
	(void)BSP_NFCTAG_ConfigIT(0, itmask);

	// 3) Make sure dynamic GPO is enabled
	(void)BSP_NFCTAG_SetGPO_en_Dyn(0);

	// 4) Clear any stale flags so the first pulse comes clean
	uint16_t dummy; (void)BSP_NFCTAG_GetITStatus(0, &dummy);
	uint8_t  dyn;   (void)BSP_NFCTAG_ReadITSTStatus_Dyn(0, &dyn);


	OLED_ON(); // on screen
	oled_clear();           // blank screen at start
	// Show initial Game 1 screen:
	OLED_Show_RLGL(false, isPlayer,0.0f, 0.0f);

	// LED MATRIX
	HT16K33_Init(15, 0);
	uint8_t frame[8] = {
	        0b00000000,
	        0b00000010,
	        0b00000100,
	        0b00001000,
	        0b00010000,
	        0b00100000,
	        0b01000000,
	        0b10000000,
	};
	HT16K33_WriteRows8(frame); // LED MATRIX

	// SENSOR_IO_Write(0xD4, 0x0D, 0x03); // gyro

	// My state machine
	currentState = &RedLightGreenLightState;
	currentState->initialise();
}



static void nfc_service(void)
{
    uint32_t it = g_nfc_it_flags;
    if (!it) return;
    g_nfc_it_flags = 0;

    if (it & NFCTAG_IT_FIELDRISING) {
        uint32_t now = HAL_GetTick();
        if (now - g_nfc_last_report_ms >= 1000) { // 1 s rate-limit
            const char *msg = "RF field ON (phone near)\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)strlen(msg), 100);
            g_nfc_last_report_ms = now;
        }
    }
}

// Main Program Code
int main(void) {
	setup();

  	while (programRunning) {
  		nfc_service();
  		currentState->update();
	}
}
