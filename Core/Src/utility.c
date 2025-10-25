#include <stdio.h>

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM5_Init(void);
void MX_USART1_UART_Init(void);
void delay_us(uint32_t us);
int write(int fd, char *ptr, int len);

int main() {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM5_Init();
    MX_USART1_UART_Init();
	HAL_TIM_Base_Start(&htim5);
	printf("Starting...\n");
	printf("\n\n\n-----------\nStarting\n");
	uint32_t loop_cnt = 0; uint32_t now=0 ; uint32_t next_blink=500; uint32_t next_tick = 1000;
    while (1) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    delay_us(500000);
    }
}
void MX_TIM5_Init(void) {
htim5.Instance = TIM5;
htim5.Init.Prescaler = 83;
htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
htim5.Init.Period = 0xFFFF;
htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
HAL_TIM_Base_Init(&htim5);
}

int write(int fd, char * ptr, int len) {
	HAL_StatusTypeDef hStatus;
	if (fd == 1 || fd== 2) {
		hStatus= HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
		if (hStatus== HAL_OK)
			return len;
		else
			return -1;
		}
	return -1;
		}
	void delay_us(uint32_t us) {
		uint32_t start= __HAL_TIM_GET_COUNTER(&htim5);
		while ((__HAL_TIM_GET_COUNTER(&htim5)- start)< us) {

		}
}

