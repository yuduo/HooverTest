#include "stm32f10x_it.h"
#include "sys.h"

void TIM4_IRQHandler(void)
{
}

void TIM2_IRQHandler(void)
{
}

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  TargetSysReset();
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/******************************************************************************/
/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
/******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
}

void CAN1_RX1_IRQHandler(void)
{
} 

void DMA1_Channel1_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
  {	 
	 DMA_ClearITPendingBit(DMA1_IT_TC1);
  }
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(IR_L_LINE)!=RESET)
	{
		EXTI_ClearITPendingBit(IR_L_LINE);
	}		
}
void EXTI9_5_IRQHandler(void)
{

	if(EXTI_GetITStatus(IR_M_LINE)!=RESET){
		EXTI_ClearITPendingBit(IR_M_LINE);
	}

	if(EXTI_GetITStatus(IR_R_LINE)!=RESET){
		EXTI_ClearITPendingBit(IR_R_LINE);
	}

	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		if(GPIOB->IDR & GPIO_Pin_5)
		{
			 		
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

void EXTI4_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line4))
   {
	  EXTI_ClearFlag(EXTI_Line4);
   }
}

void EXTI2_IRQHandler(void)
{
   if(EXTI_GetITStatus(EXTI_Line2))
   {
	  EXTI_ClearFlag(EXTI_Line2);
   }
}

void TIM1_TRG_COM_IRQHandler(void)
{ 
  TIM1->SR&=0;           //清中断标志
	     //调用换向函数
}
/**********************************************************************
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void TIM1_BRK_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
}

/**********************************************************************
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void TIM6_IRQHandler(void)
{
	 if(TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET)
	 {
	 	  TIM6->CNT=0;	 	  
		  TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	 }	 
}

/**********************************************************************
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention      : None
**********************************************************************/
void TIM7_IRQHandler(void)
{
	 if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
	 {
	 	  TIM7->CNT=0;	 	  
		  TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  
	 }	 
}

void TIM8_UP_IRQHandler(void)
{
}

void TIM8_CC_IRQHandler(void)
{
	 TIM8->CNT=0;
}

void TIM3_IRQHandler(void)
{
}


void TIM1_CC_IRQHandler(void)
{
}


void ADC1_2_IRQHandler(void)
{
	if(ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET)
	{
		ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);		
	}
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

