/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define pino_rs_0   GPIOA->BSRR = 1<<(16+9)  
#define pino_rs_1		GPIOA->BSRR = 1<<9
#define pino_e_0		GPIOC->BSRR = 1<<(16+7)
#define pino_e_1		GPIOC->BSRR = 1<<7

#define cursor_on 	 0x0c
#define cursor_off   0x0e
#define cursor_blink 0x0f

struct Controle{
	
	float temp;
	float umid;
  float ppm_gas;
	
	char str[60];

  char tempo_estado;
  char estado;
		
	int hora, min, seg;
	int dia, mes, ano;
	
}c;

char dados[300];

float le_temperatura(void);
float le_umidade(void);
float le_gas(void);
void testa_bluetooth(void);
void print_serial(char * string);
void envia_bluetooth(char * string);
void transmite(void);

void renderiza_tela1(void);
void renderiza_relogio(void);
void renderiza_sensores(void);
void renderiza_tela2(void);
void limpa_lcd(void);

void uDelay(void)
{
  int x=10;
	
  while(x) x--;
}
void delayUs(int tempo)
{
  while(tempo--) uDelay();
}
void lcd_send4bits(unsigned char dat)
{
	if((dat & (1 << 0)) == 0)
		GPIOB->BRR = (1 << 5);
	else
		GPIOB->BSRR = (1 << 5);
	
	if((dat & (1 << 1)) == 0)
		GPIOB->BRR = (1 << 4);
	else
		GPIOB->BSRR = (1 << 4);

	if((dat & (1 << 2)) == 0)
		GPIOB->BRR = (1 << 10);
	else
		GPIOB->BSRR = (1 << 10);
	
	if((dat & (1 << 3)) == 0)
		GPIOA->BRR = (1 << 8);
	else
		GPIOA->BSRR = (1 << 8);	
}
void lcd_wrcom4(unsigned char com)
{
	lcd_send4bits(com);
	pino_rs_0;
	pino_e_1;
	delayUs(5);
	pino_e_0;
	HAL_Delay(5);
}
void lcd_wrcom(unsigned char com)
{
	lcd_send4bits(com/0x10);
	pino_rs_0;
	pino_e_1;
	delayUs(5);
	pino_e_0;
	
	lcd_send4bits(com%0x10);
	pino_rs_0;
	pino_e_1;
	delayUs(5);
	pino_e_0;
	
	HAL_Delay(5);
}
void lcd_wrchar(unsigned char ch)
{
	lcd_send4bits(ch/0x10);
	pino_rs_1;
	pino_e_1;
	delayUs(5);
	pino_e_0;
	
	lcd_send4bits(ch % 0x10);
	pino_rs_1;
	pino_e_1;
	delayUs(5);
	pino_e_0;
	
	HAL_Delay(5);
}
void lcd_init(unsigned char cursor)
{
	lcd_wrcom4(3);
  lcd_wrcom4(3);
  lcd_wrcom4(3);
  lcd_wrcom4(2);

  lcd_wrcom(0x28);
  lcd_wrcom(cursor);
  lcd_wrcom(0x06);
  lcd_wrcom(0x01);

}
void lcd_goto(unsigned char x, unsigned char y)
{
  if(x<16)
  {
    if(y==0) lcd_wrcom(0x80+x);
    if(y==1) lcd_wrcom(0xc0+x);
    if(y==2) lcd_wrcom(0x90+x);
    if(y==3) lcd_wrcom(0xd0+x);
  }
}
void lcd_wrstr(char * str)
{
	while((*str))
	{
		lcd_wrchar(*str++);
	}
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
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
	
	c.dia = 30;
	c.mes = 3;
	c.ano = 95;
	
	c.hora = 23;
	c.min = 59;
	c.seg = 45;
	
	c.temp = 10.0;
	c.umid = 15.0;
	
  c.tempo_estado = 3;
  c.estado = 0;

	int tick_i = 0;
	int tick_f = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	lcd_init(cursor_off);
	GPIOB->BSRR = (1 << 6); //backlight on
	
  testa_bluetooth();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		tick_i = HAL_GetTick();
		
		c.temp = le_temperatura();
		c.umid = le_umidade();
    c.ppm_gas = le_gas();

    transmite();

    if(c.estado == 0)
    {
      renderiza_tela1();

      c.tempo_estado--;
      if(c.tempo_estado == 0)
      {
        c.tempo_estado = 3;
        c.estado = 1;
      }
    }
    else if(c.estado == 1)
    {
      renderiza_tela2();

      c.tempo_estado--;
      if(c.tempo_estado == 0)
      {
        c.tempo_estado = 3;
        c.estado = 0;
      }
    }
		
		tick_f = HAL_GetTick();
		while(tick_f - tick_i < 1000) tick_f = HAL_GetTick();	
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void transmite(void)
{
  sprintf(dados,"%02.1f %02.1f %04.1f", c.temp, c.umid, c.ppm_gas);
  print_serial(dados);
  envia_bluetooth(dados);
}
float le_temperatura(void)
{
	uint8_t dado[2];
	uint16_t T0_degC = 0, T1_degC = 0;
	int16_t  T1_out = 0, T0_out = 0,T_out= 0;
	
	dado[0] = 0x82;
	HAL_I2C_Mem_Write(&hi2c2,0xBE,0x20,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x32,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x33,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T0_degC = dado[0]; 
	T1_degC = dado[1];
	
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x35,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	
	T1_degC = ((dado[0] & 0xC) << 6) + T1_degC; 
	T0_degC = ((dado[0]  & 3) << 8) + T0_degC;
	T0_degC = T0_degC /8; 
	T1_degC = T1_degC / 8;
	
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x3C,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x3D,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T0_out = (dado[1] << 8) + dado[0];
	
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x3E,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x3F,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T1_out = (dado[1] << 8) + dado[0];
	
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x2A,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x2B,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T_out = (dado[1] << 8) + dado[0];
	
	return (((T1_degC - T0_degC) * (T_out - T0_out))/(T1_out - T0_out) + T0_degC);
}
float le_umidade(void)
{
	uint8_t dado[2];
	dado[0] = 0x82;
	dado[1] = 0;
	
	uint16_t H0_rH_x2, H1_rH_x2;
	
	int16_t H0_T0_OUT, H1_T0_OUT;
	
	int16_t H_OUT;
	
	//escrever na memoria do sensor pra dar WAKE UP
	HAL_I2C_Mem_Write(&hi2c2,0xBE,0x20,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	
	//agora seguir roteiro
	
	//1 leitura dos registradores das posicoes 0x30 e 0x31
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x30,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x31,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H0_rH_x2 = dado[0]/2;
	H1_rH_x2 = dado[1]/2;
	
	float h = 0;
	
	//3 leitura dos 0x36, 0x37
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x36,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x37,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H0_T0_OUT = (dado[1] << 8) + dado[0];
	
	//4 leitura 0x3a, 0x3b
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x3a,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x3b,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H1_T0_OUT = (dado[1] << 8) + dado[0];
	
	// 5 leitura do 0x28 e 0x29
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x28,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c2,0xBF,0x29,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H_OUT = (dado[1] << 8) + dado[0];
	
	// 6 calcular
	
	h = (((H1_rH_x2 - H0_rH_x2) * (H_OUT - H0_T0_OUT))/(H1_T0_OUT - H0_T0_OUT))+H0_rH_x2;
		
	return h;
}
void renderiza_relogio(void)
{
	lcd_goto(0,1);
	sprintf(c.str,"%02d:%02d:%02d",c.hora,c.min,c.seg);
	lcd_wrstr(c.str);
	
	lcd_goto(11,1);
	sprintf(c.str,"%02d/%02d",c.dia,c.mes);
	lcd_wrstr(c.str);
	
	c.seg++;
	
	if(c.seg == 60)
	{
		c.seg = 0;
		c.min++;
		if(c.min == 60)
		{
			c.min = 0;
			c.hora++;
			if(c.hora == 24)
			{
				c.hora = 0;
				c.dia++;
				if(c.dia == 31)
				{
					c.dia = 1;
					c.mes++;
					if(c.mes == 13)
					{
						c.mes = 1;
						c.ano++;
					}
				}
			}
		}
	}
}
void testa_bluetooth(void)
{
	char str[30];
	sprintf(str,"Hello World\n");
	HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),100);
	
	/*sprintf(dados,"%2.1fC\n%2.1f%%\nEnviado\n\n",c.temp,c.umid);
	HAL_UART_Transmit(&huart3,(uint8_t*)dados,strlen(dados),100);*/
}
void print_serial(char * string)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)string,strlen(string),100);
}
void envia_bluetooth(char * string)
{
	HAL_UART_Transmit(&huart3,(uint8_t*)string,strlen(string),100);
}
void renderiza_tela1(void)
{
  renderiza_relogio();
  renderiza_sensores();
}
void renderiza_sensores(void)
{
  lcd_goto(0,0);
	sprintf(c.str,"%2.1fC  %2.1f%%",c.temp,c.umid);
	lcd_wrstr(c.str);
}
void renderiza_tela2(void)
{
  sprintf(c.str,"%2.1f ppm",c.ppm_gas);
  lcd_goto(0,1);
  lcd_wrstr(c.str);

  if(c.ppm_gas < 5.0)
  {
    sprintf(c.str,"Situacao Regular");
    lcd_goto(0,0);
    lcd_wrstr(c.str);  
  }
  else if(c.ppm_gas >= 5.0)
  {
    // algo
  }
}
void limpa_lcd(void)
{
  sprintf(c.str,"               ");
  lcd_goto(0,0);
  lcd_wrstr(c.str);
  lcd_goto(0,1);
  lcd_wrstr(c.str);
}
float le_gas(void)
{
  return 0.0;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
