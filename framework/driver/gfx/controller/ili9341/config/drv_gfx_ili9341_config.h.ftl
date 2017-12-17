<#--
/*******************************************************************************
  Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_ili9341_config.h.ftl

  Summary:
    LCC Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->
 <#if CONFIG_USE_DRV_GFX_ILI9341 == true>
 /*** ILI9341 Driver Configuration ***/
#define  GFX_USE_DISPLAY_CONTROLLER_ILI9341
 <#if CONFIG_DRV_ILI9341_MODE_SPI == true>
#define  GFX_DRV_ILI9341_MODE_SPI
#define DRV_ILI9341_SPI_MODULE_INDEX  ${CONFIG_DRV_ILI9341_SPI_MODULE_INDEX}
#define LCD_RS_PORT		${CONFIG_DRV_ILI9341_RS_PORT_CHANNEL} 
#define LCD_CS_PORT		${CONFIG_DRV_ILI9341_CS_PORT_CHANNEL}
#define LCD_REST_PORT	${CONFIG_DRV_ILI9341_REST_PORT_CHANNEL}	  		

#define LCD_RS_BIT_POS		${CONFIG_DRV_ILI9341_RS_BIT_POSITION} 
#define LCD_CS_BIT_POS		${CONFIG_DRV_ILI9341_CS_BIT_POSITION} 
#define LCD_REST_BIT_POS	${CONFIG_DRV_ILI9341_REST_BIT_POSITION}
 </#if>
 <#if CONFIG_DRV_ILI9341_MODE_16BIT == true>
#define  GFX_DRV_ILI9341_MODE_16BIT
#define LCD_RS_PORT		${CONFIG_DRV_ILI9341_RS_PORT_CHANNEL} 		 
#define LCD_WR_PORT		${CONFIG_DRV_ILI9341_WR_PORT_CHANNEL}  		  
#define LCD_RD_PORT		${CONFIG_DRV_ILI9341_RD_PORT_CHANNEL}  			     
#define LCD_CS_PORT		${CONFIG_DRV_ILI9341_CS_PORT_CHANNEL}  			
#define LCD_REST_PORT	${CONFIG_DRV_ILI9341_REST_PORT_CHANNEL}	

#define LCD_RS_BIT_POS		${CONFIG_DRV_ILI9341_RS_BIT_POSITION}  		 
#define LCD_WR_BIT_POS		${CONFIG_DRV_ILI9341_WR_BIT_POSITION}  		  
#define LCD_RD_BIT_POS		${CONFIG_DRV_ILI9341_RD_BIT_POSITION}  			     
#define LCD_CS_BIT_POS		${CONFIG_DRV_ILI9341_CS_BIT_POSITION}  			
#define LCD_REST_BIT_POS	${CONFIG_DRV_ILI9341_REST_BIT_POSITION}	   
 </#if>
 </#if>
<#--
/*******************************************************************************
 End of File
*/
-->

