/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "atraso.h"
#include "defPrincipais.h"
#include "NOKIA5110_fb.h"
#include "figuras.h"
#include "PRNG_LFSR.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

//osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t ADC_buffer[2];
uint32_t valor_ADC[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		valor_ADC[0] = ADC_buffer[0];
		valor_ADC[1] = ADC_buffer[1];
	}
}

//---------------------------------------------------------------------------------------------------
// Tarefa para atualizar periodicamente o LCD
void vTask_LCD_Print(void *pvParameters) {
	while (1)
		imprime_LCD();
}

//---------------------------------------------------------------------------------------------------
#define X_LIMIT_LEFT		1
#define X_LIMIT_RIGHT		82
#define Y_LIMIT_UP 			9
#define Y_LIMIT_DOWN		46
#define MAX_SNAKE_LENGTH	30

uint32_t x[MAX_SNAKE_LENGTH];
uint32_t y[MAX_SNAKE_LENGTH];
uint32_t temp_x;
uint32_t temp_y;
uint32_t x_swap;
uint32_t y_swap;
uint32_t food_x;
uint32_t food_y;
uint32_t speed_delay = 500;
uint32_t score = 0;

uint32_t snake_length = 6;
uint32_t move_direction;

enum{
	STOP,
	LEFT,
	RIGHT,
	UP,
	DOWN,
}movement_position;

/**
 * @brief Atualiza os desenhos na tela
 * 
 * @retval 
 */
void draw_screen(void)
{
	struct pontos_t screen;

	screen.x1 = 0;
	screen.y1 = 7;
	screen.x2 = 83;
	screen.y2 = 47;

	goto_XY(0,0);
	string_LCD_Nr("score ", score, 3);
	desenha_retangulo(&screen, 1);				// Desenha os limites da tela
	desenha_circulo(food_x, food_y, 1, 1); 		// Desenha a comida
}

/**
 * @brief Gera uma nova comida em uma posição aleatória
 * @param
 * @retval
 */
void create_food(void)
{
	uint32_t rand_prng = prng_LFSR();
/*	
	food_x = rand_prng & (0x00000053 | 0x00000005);		// Limita o valor entre 1 e 83
	food_y = rand_prng & (0x0000002F | 0x00000009);		// Limita o valor entre 7 e 47

	// Se a posição da comida estiver fora dos limites, gera outro valor
	while(food_x <= 3 || food_x >= 85)
	{
		food_x = rand_prng & (0x00000053 | 0x00000005);
	}


	while(food_y <= 9 || food_y >= 45)
	{
		food_y = rand_prng & (0x0000002F | 0x00000009);
	}
*/	

	do 
	{
		rand_prng = prng_LFSR();
		food_x = rand_prng & (0x00000053 | 0x00000005);
	} while(food_x < 2 || food_x > 79);

	do
	{
		rand_prng = prng_LFSR();
		food_y = rand_prng & (0x0000002F | 0x00000009);
	} while(food_y < 10 || food_y > 42);

}

/**
 * @brief
 * @param
 * @retval
 */
void check_colision(void)
{
	uint32_t i;

	// Colisão com a parede ou com o próprio corpo
	for(i = 1; i < snake_length; i++)
	{
		if( (x[0] <= X_LIMIT_LEFT || x[0] >= X_LIMIT_RIGHT) || (y[0] <= Y_LIMIT_UP || y[0] >= Y_LIMIT_DOWN) || (x[i] == x[0] && y[i] == y[0]))
		{

			limpa_LCD();
			escreve2fb((unsigned char*) game_over);
			imprime_LCD();
			while(1);
		}
	}
}

/**
 * @brief
 * @param
 * @retval
 */
void check_food_colision(void)
{
	if(x[0] == food_x || x[0] == food_x + 1 || x[0] == food_x + 2 || x[0] == food_x - 1)
	{
		if(y[0] == food_y || y[0] == food_y + 1 || y[0] == food_y + 2 || y[0] == food_y - 1)
		{
			score++;
			snake_length++;
			//speed_delay-= 50;

			// Apaga o círculo antigo e gera outro
			desenha_circulo(food_x, food_y, 1, 0);
			create_food();
		}
	}
}

/**
 * @brief Controla o movimento da cobra
 * @param
 * @retval
 */
void move_snake(void) {
	uint32_t i;

	for (i = 0; i < snake_length; i++) {
		x_swap = x[i];
		y_swap = y[i];
		x[i] = temp_x;
		y[i] = temp_y;
		temp_x = x_swap;
		temp_y = y_swap;
	}

	limpa_LCD();
	draw_screen();

	for (i = 0; i < snake_length; i++) {
		desenha_circulo(x[i], y[i], 1, 1);
	}
}

/**
 * @brief Verifica os valores do ADC e aplica o movimento
 * @param
 * @retval
 */
void check_movement(void) 
{
	if (valor_ADC[0] < 20) 
		move_direction = RIGHT;

	if (valor_ADC[0] > 3400) 
		move_direction = LEFT;
		
	if (valor_ADC[1] < 20) 
		move_direction = DOWN;

	if (valor_ADC[1] > 3400) 
		move_direction = UP;

	switch(move_direction)
	{
		case LEFT:
			temp_x = x[0] - 3;
			temp_y = y[0];
			break;

		case RIGHT:
			temp_x = x[0] + 3;
			temp_y = y[0];
			break;

		case UP:
			temp_x = x[0];
			temp_y = y[0] + 3;
			break;

		case DOWN:
			temp_x = x[0];
			temp_y = y[0] - 3;
			break;
	}
}

/**
 * @brief Tarefa principal do jogo
 * @param *pvParameters: Parâmetros passados para a função
 * @retval nenhum
 */
void vTask_Game(void *pvParameters) {
	uint32_t i;

	// Cria a primeira comida
	food_x = 10;
	food_y = 15;

	draw_screen();
	for (i = 0; i < snake_length; i++) {
		x[i] = 25 - 3 * i;
		y[i] = 30;
	}
	for (i = 0; i < snake_length; i++) {
		desenha_circulo(x[i], y[i], 1, 1);
		vTaskDelay(150/portTICK_RATE_MS);
	}

	move_direction = RIGHT;		// Inicia o movimento da cobra

	//vTaskDelay(1000/portTICK_RATE_MS);

	while (1) {
		check_colision();
		check_food_colision();
		check_movement();
		move_snake();
		
		vTaskDelay(speed_delay/ portTICK_RATE_MS);
	}

}

/**
 * @brief
 * @param
 * @retval
 */
void LED_Task(void *pvParameters) {
	while (1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

//---------------------------------------------------------------------------------------------------
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint32_t semente_PRNG = 1;

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();

	/* USER CODE BEGIN 2 */

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_buffer, 2);
	HAL_ADC_Start_IT(&hadc1);

	// inicializa LCD 5110
	inic_LCD();
	limpa_LCD();
	escreve2fb((unsigned char*) snake);

	imprime_LCD();
	HAL_Delay(1200);
	limpa_LCD();

/*
	goto_XY(0, 0);
	string_LCD("Press.  Botao");
	imprime_LCD();
*/
	limpa_LCD();
	escreve2fb((unsigned char*) press_button);
	imprime_LCD();

//	vTaskDelay(2000/portTICK_RATE_MS);

	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) // enquando nao pressionar joystick fica travado
	{
		semente_PRNG++;	// semente para o gerador de n�meros pseudoaleatorios
						// pode ser empregado o ADC lendo uma entrada flutuante para gerar a semente.
	}

	init_LFSR(semente_PRNG);// inicializacao para geracao de numeros pseudoaleatorios
	//rand_prng = prng_LFSR();	// sempre q a funcao prng() for chamada um novo nr � gerado.

	limpa_LCD();
	imprime_LCD();

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	// osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	// defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	/* USER CODE BEGIN RTOS_THREADS */
	
	xTaskCreate(vTask_LCD_Print, "Task 1", 100, NULL, 1, NULL);
	xTaskCreate(vTask_Game, "Game Task", 500, NULL, 1, NULL);
	xTaskCreate(LED_Task, "LED", 100, NULL, 1, NULL);

	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Start scheduler */
	vTaskStartScheduler();// apos este comando o RTOS passa a executar as tarefas

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA3 PA4 PA5 PA6
	 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
			| GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
//void StartDefaultTask(void const * argument)
//{
/* USER CODE BEGIN 5 */
/* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
/* USER CODE END 5 */
//}
/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
