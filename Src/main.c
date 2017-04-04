 /**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    22-April-2016 
  * @brief   STM32F7xx HAL API Template project 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* This project template calls firstly two functions in order to configure MPU feature 
     and to enable the CPU Cache, respectively MPU_Config() and CPU_CACHE_Enable().
     These functions are provided as template implementation that User may integrate 
     in his application, to enhance the performance in case of use of AXI interface 
     with several masters. */ 
  
  /* Configure the MPU attributes as Write Through */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to have a frequency of 216 MHz */
  SystemClock_Config();


  /* Add your application code here
     */

  /* Init variables */

  /* Configure LED1 */
  BSP_LED_Init(LED1);

  /* -------------------------------------------------------------------------------- */
  /* -- Configure LCD                                                              -- */
  /* -------------------------------------------------------------------------------- */
  /* LCD Initialization */
  BSP_LCD_Init();

  /* LCD Initialization */
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

  /* Enable the LCD */
  BSP_LCD_DisplayOn();

  /* Select the LCD Foreground Layer  */
  BSP_LCD_SelectLayer(0);

  /* Clear the Foreground Layer */
  BSP_LCD_Clear(LCD_COLOR_BLACK);

  /* Configure the transparency : Increase the transparency */
  BSP_LCD_SetTransparency(0, 0xFF);


  /* -------------------------------------------------------------------------------- */
  /* -- Configure TS module                                                        -- */
  /* -------------------------------------------------------------------------------- */
  if(BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize()) != TS_OK)
	  Error_Handler();
  else
  {
  	if (BSP_TS_ITConfig() != TS_OK)
  		Error_Handler();
  }


  /* uGUI Init */
  UG_Init(&gui, (void (*)(UG_S16, UG_S16, UG_COLOR))pset, BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  UG_FillScreen(C_WHITE);

  /* uGUI : create window */
  createMainMenuWindow();
  createGraphWindow();

  /* Infinite loop */
  while (1)
  {
	/* Get present tick */
	NowTickCount = HAL_GetTick();

	/* Reset touch screen data if specified time has passed */
	if( ((NowTickCount - LastTouchedTickCount) > uGUI_UPDATE_MS) && (TS_State.touchDetected > 0) )
	  BSP_TS_ResetTouchData(&TS_State);

	/* Update uGUI */
	if(NowTickCount % uGUI_UPDATE_MS == 0)
	{
	  /* Touch screen data update */
	  if (TS_State.touchDetected > 0)
	    UG_TouchUpdate(TS_State.touchX[0], TS_State.touchY[0], TOUCH_STATE_PRESSED);
	  else
	  	UG_TouchUpdate(-1, -1, TOUCH_STATE_RELEASED);

	  /* Update display */
	  UG_Update();
	}


	/* Draw graph */
	if(gui.active_window == &wnd_Graph)
	{
		if(NowTickCount % 1000 == 0)
			UG_FillFrame(15, 120-50, 15+1000/4, 120+50, C_WHITE); /* Clear draw area */

		float sin_val, cos_val;
		sin_val = sinf(2.0f * M_PI * ((float)NowTickCount / 1000.0f) );
		cos_val = cosf(2.0f * M_PI * ((float)NowTickCount / 1000.0f) );
		UG_DrawPixel(15 + (NowTickCount % 1000)/4, 120.0f - 50.0f * sin_val, C_RED);
		UG_DrawPixel(15 + (NowTickCount % 1000)/4, 120.0f - 50.0f * cos_val, C_BLUE);
	}
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive to reach the 216 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/* ---------------------------------------------------------------- */
/* -- Main Menu window                                           -- */
/* ---------------------------------------------------------------- */
void createMainMenuWindow(void)
{
	/* Configure window */
	UG_WindowCreate(&wnd_MainMenu, obj_buff_wnd_MainMenu, uGUI_MAX_OBJECTS, MainMenuWindow_callback);
	UG_WindowSetTitleText(&wnd_MainMenu, "MAIN MENU");
	UG_WindowSetTitleTextFont(&wnd_MainMenu, &FONT_12X20);
	UG_WindowSetTitleInactiveColor(&wnd_MainMenu, C_GRAY);

	/* Configure buttons */
	UG_ButtonCreate(&wnd_MainMenu, &btn_MainMenu_LED, BTN_ID_0,
			10, 40, 150, 120);
	UG_ButtonSetFont(&wnd_MainMenu, BTN_ID_0, &FONT_12X20);
	UG_ButtonSetBackColor(&wnd_MainMenu, BTN_ID_0, C_GRAY);
	UG_ButtonSetText(&wnd_MainMenu, BTN_ID_0, "TEST\nLED");

	UG_ButtonCreate(&wnd_MainMenu, &btn_MainMenu_switchGraphWindow, BTN_ID_1,
			10, 130, 150, 210);
	UG_ButtonSetFont(&wnd_MainMenu, BTN_ID_1, &FONT_12X20);
	UG_ButtonSetBackColor(&wnd_MainMenu, BTN_ID_1, C_GRAY);
	UG_ButtonSetText(&wnd_MainMenu, BTN_ID_1, "Graph");

	/* Configure textbox */
	UG_TextboxCreate(&wnd_MainMenu, &txt_MainMenu_Hello, TXB_ID_0,
			10, 10, 200, 30);
	UG_TextboxSetFont(&wnd_MainMenu, TXB_ID_0, &FONT_12X20);
	UG_TextboxSetText(&wnd_MainMenu, TXB_ID_0, "Hello uGUI!");
	UG_TextboxSetAlignment(&wnd_MainMenu, TXB_ID_0, ALIGN_TOP_LEFT);

	UG_WindowShow(&wnd_MainMenu);
}

void MainMenuWindow_callback(UG_MESSAGE* msg)
{
	static GPIO_PinState LED1_state = GPIO_PIN_RESET;

	if (msg->type == MSG_TYPE_OBJECT)
	{
		if (msg->id == OBJ_TYPE_BUTTON)
		{
			switch (msg->sub_id)
			{
			case BTN_ID_0:
				/* Toggle LED */
				if(LED1_state == GPIO_PIN_RESET)
				{
					BSP_LED_On(LED1);
					LED1_state = GPIO_PIN_SET;
					UG_ButtonSetBackColor(&wnd_MainMenu, BTN_ID_0, C_LIME);
				}
				else
				{
					BSP_LED_Off(LED1);
					LED1_state = GPIO_PIN_RESET;
					UG_ButtonSetBackColor(&wnd_MainMenu, BTN_ID_0, C_GRAY);
				}
				break;

			case BTN_ID_1:
				/* Switch Graph window */
				UG_WindowShow(&wnd_Graph);
				break;

			default:
				Error_Handler();
				break;
			}
		}
	}
}


/* ---------------------------------------------------------------- */
/* -- Graph window                                               -- */
/* ---------------------------------------------------------------- */
void createGraphWindow(void)
{
	/* Configure window */
	UG_WindowCreate(&wnd_Graph, obj_buff_wnd_Graph, uGUI_MAX_OBJECTS, GraphWindow_callback);
	UG_WindowSetTitleText(&wnd_Graph, "Graph");
	UG_WindowSetTitleTextFont(&wnd_Graph, &FONT_12X20);
	UG_WindowResize(&wnd_Graph, 10, 10, BSP_LCD_GetXSize() - 10, BSP_LCD_GetYSize() - 10);

	/* Configure buttons */
	UG_ButtonCreate(&wnd_Graph, &btn_Graph_switchMainMenu, BTN_ID_0,
			UG_WindowGetInnerWidth(&wnd_Graph)/2 + 5, UG_WindowGetInnerHeight(&wnd_Graph) - 50,
			UG_WindowGetInnerWidth(&wnd_Graph) - 5  , UG_WindowGetInnerHeight(&wnd_Graph) - 5);
	UG_ButtonSetFont(&wnd_Graph, BTN_ID_0, &FONT_12X20);
	UG_ButtonSetBackColor(&wnd_Graph, BTN_ID_0, C_GRAY);
	UG_ButtonSetText(&wnd_Graph, BTN_ID_0, "Close");

	/* Configure textboxes */
	UG_TextboxCreate(&wnd_Graph, &txt_Graph_sin, TXB_ID_0,
			5, UG_WindowGetInnerHeight(&wnd_Graph) - 50,
			UG_WindowGetInnerWidth(&wnd_Graph)/2 - 5, UG_WindowGetInnerHeight(&wnd_Graph) - 30 );
	UG_TextboxSetFont(&wnd_Graph, TXB_ID_0, &FONT_8X12);
	UG_TextboxSetForeColor(&wnd_Graph, TXB_ID_0, C_RED);
	UG_TextboxSetText(&wnd_Graph, TXB_ID_0, "y=sin(x)");
	UG_TextboxSetAlignment(&wnd_Graph, TXB_ID_0, ALIGN_TOP_LEFT);

	UG_TextboxCreate(&wnd_Graph, &txt_Graph_cos, TXB_ID_1,
			5, UG_WindowGetInnerHeight(&wnd_Graph) - 25,
			UG_WindowGetInnerWidth(&wnd_Graph)/2 - 5, UG_WindowGetInnerHeight(&wnd_Graph) - 5 );
	UG_TextboxSetFont(&wnd_Graph, TXB_ID_1, &FONT_8X12);
	UG_TextboxSetForeColor(&wnd_Graph, TXB_ID_1, C_BLUE);
	UG_TextboxSetText(&wnd_Graph, TXB_ID_1, "y=cos(x)");
	UG_TextboxSetAlignment(&wnd_Graph, TXB_ID_1, ALIGN_TOP_LEFT);
}

void GraphWindow_callback(UG_MESSAGE* msg)
{
	if (msg->type == MSG_TYPE_OBJECT)
	{
		if (msg->id == OBJ_TYPE_BUTTON)
		{
			switch (msg->sub_id)
			{
			case BTN_ID_0:
				/* Switch Main menu window */
				UG_WindowShow(&wnd_MainMenu);
				break;

			default:
				Error_Handler();
				break;
			}
		}
	}
}




/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
	case TS_INT_PIN:	// Touch screen interrupt
		LastTouchedTickCount = HAL_GetTick();
		BSP_TS_ITGetStatus();
		BSP_TS_ITClear();
		BSP_TS_GetState(&TS_State);
		break;

	default:
		break;
	}
}


/* -------------------------------------------------------------------------------- */
/* -- Porting function for uGUI                                                  -- */
/* -------------------------------------------------------------------------------- */
void pset(UG_S16 x, UG_S16 y, UG_COLOR col)
{
	BSP_LCD_DrawPixel( (uint16_t)x, (uint16_t)y, (0xFF000000 | (uint32_t)col) );
}

/* Hardware accelerator */
UG_RESULT _HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c)
{
	UG_DrawLine(x1, y1, x2, y2, c);
	return UG_RESULT_OK;
}

UG_RESULT _HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c)
{
	UG_FillFrame(x1, y1, x2, y2, c);
	return UG_RESULT_OK;
}

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
