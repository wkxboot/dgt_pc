#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"


int board_init()
{
  
  
return 0; 
}

void bsp_protocol_485_enable_read()
{
 HAL_GPIO_WritePin(HOST_485_RWE_CTRL_GPIO_Port,HOST_485_RWE_CTRL_Pin,GPIO_PIN_RESET);  
}

void bsp_protocol_485_enable_write()
{
 HAL_GPIO_WritePin(HOST_485_RWE_CTRL_GPIO_Port,HOST_485_RWE_CTRL_Pin,GPIO_PIN_SET);    
}

void bsp_modbus_485_enable_read()
{
 HAL_GPIO_WritePin(SLAVE_485_RWE_CTRL_GPIO_Port,SLAVE_485_RWE_CTRL_Pin,GPIO_PIN_RESET);      
}

void bsp_modbus_485_enable_write()
{
 HAL_GPIO_WritePin(SLAVE_485_RWE_CTRL_GPIO_Port,SLAVE_485_RWE_CTRL_Pin,GPIO_PIN_SET);     
}


