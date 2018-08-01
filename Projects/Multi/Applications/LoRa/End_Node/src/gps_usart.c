#define UART                                USART2

#define UART_TX_PIN                       GPIO_PIN_2
#define UART_RX_PIN                       GPIO_PIN_3
#define UART_GPIO_PORT                     GPIOA 
#define UART_AF                            GPIO_AF4_USART2
#define UART_RCC_CLK_ENABLE()           __HAL_RCC_USART2_CLK_ENABLE();
#define GPIO_PORT_RCC_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE();

#define UART_IRQn                         USART2_IRQn

#define NUM_BYTE_RCV_IT                     1

void SERIAL_Uart_IoInit(void);

//Uart Handle
UART_HandleTypeDef serialConfHandle;

uint8_t confRxBuffer[NUM_BYTE_RCV_IT];

/*!
 * @brief Initializes the UART interface
 * @param None
 * @retval None
 */
void DEVCONFSERIAL_Init(void)
{     
     //Enable clock
     GPIO_PORT_RCC_CLK_ENABLE();
     
     SERIAL_Uart_IoInit();
     
     serialConfHandle.Instance = UART;
     serialConfHandle.Init.BaudRate = 9600;
    serialConfHandle.Init.WordLength = UART_WORDLENGTH_8B;
     serialConfHandle.Init.StopBits = UART_STOPBITS_1;
     serialConfHandle.Init.Parity = UART_PARITY_NONE;

    serialConfHandle.Init.Mode = UART_MODE_TX_RX;
    serialConfHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    serialConfHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    serialConfHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    serialConfHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&serialConfHandle) != HAL_OK)
    {
      //Initialization Error
      Error_Handler(); 
    }
     
     if(HAL_UART_GetState(&serialConfHandle) != HAL_UART_STATE_READY)
     {
          //Initialization Error
          Error_Handler();
     }
     
     HAL_UART_Receive_IT(&serialConfHandle, (uint8_t *)confRxBuffer, NUM_BYTE_RCV_IT);
     
     //Enable interrupt
     //__HAL_UART_ENABLE_IT(&serialConfHandle, UART_IT_RXNE);
     
     //__HAL_UART_ENABLE_IT(&serialConfHandle, UART_IT_TC);
}

/*!
 * @brief Initializes the GPIO for UART interface
 * @param None
 * @retval None
 */
void SERIAL_Uart_IoInit(void)
{
     UART_RCC_CLK_ENABLE();
     
     GPIO_InitTypeDef GPIO_InitStruct = {0};
     
     //Set pin
     GPIO_InitStruct.Pin = UART_TX_PIN|UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = UART_AF;
    HAL_GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

    //USART1 interrupt Init
    HAL_NVIC_SetPriority(UART_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART_IRQn);
}

/*!
 * @brief IRQ handler for USART2
 * @param None
 * @retval None
 */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&serialConfHandle);
}

/*!
 * @brief Receive Complete Callbacks
 * @param None
 * @retval None
 */
void DEVCONFSERIAL_RcvSignal(void)
{
     uint8_t buff[1];
     
     buff[0] = confRxBuffer[0];
     
     //DEVCONFSERIAL_Send(buff, 1);
     
     HAL_UART_Receive_IT(&serialConfHandle, (uint8_t *)confRxBuffer, NUM_BYTE_RCV_IT);
}