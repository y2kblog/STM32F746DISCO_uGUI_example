/**
  ******************************************************************************
  * @file    Templates/main.h 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    22-April-2016 
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* EVAL includes component */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

/* uGUI */
#include "ugui.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define uGUI_UPDATE_MS		50		/* uGUI update time in milliseconds */
#define uGUI_MAX_OBJECTS	20

/* Exported variables --------------------------------------------------------*/
uint32_t NowTickCount;

/* Touch screen */
TS_StateTypeDef TS_State;
uint32_t LastTouchedTickCount;

/* uGUI structure */
UG_GUI gui;

/* uGUI : Main Menu Window */
UG_WINDOW	wnd_MainMenu;
UG_OBJECT 	obj_buff_wnd_MainMenu[uGUI_MAX_OBJECTS];
UG_BUTTON 	btn_MainMenu_LED;
UG_BUTTON 	btn_MainMenu_switchGraphWindow;
UG_TEXTBOX	txt_MainMenu_Hello;

/* uGUI : Graph Window */
UG_WINDOW	wnd_Graph;
UG_OBJECT 	obj_buff_wnd_Graph[uGUI_MAX_OBJECTS];
UG_BUTTON 	btn_Graph_switchMainMenu;
UG_TEXTBOX	txt_Graph_sin;
UG_TEXTBOX	txt_Graph_cos;

/* Exported functions ------------------------------------------------------- */
/* uGUI porting function */
void pset(UG_S16, UG_S16, UG_COLOR);

/* uGUI : Main Menu Window */
void createMainMenuWindow(void);
void MainMenuWindow_callback(UG_MESSAGE*);

/* uGUI : Graph Window */
void createGraphWindow(void);
void GraphWindow_callback(UG_MESSAGE*);


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
