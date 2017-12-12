<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
 -->
// <editor-fold defaultstate="collapsed" desc="Graphics Library Initialization Data">
/*** GFX Initialization Data ***/

 const GFX_INIT gfxInit0 =
{
<#if CONFIG_USE_DRV_GFX_LCC == true>
    .drvInitialize    = NULL,
    .drvOpen          = DRV_GFX_LCC_Open,
    .drvInterfaceSet  = DRV_GFX_LCC_InterfaceSet,
</#if>
<#if CONFIG_USE_DRV_GFX_S1D13517 == true>
    .drvInitialize    = NULL,
    .drvOpen          = DRV_GFX_S1D13517_Open,
    .drvInterfaceSet  = DRV_GFX_S1D13517_InterfaceSet,
</#if>
<#if CONFIG_USE_DRV_GFX_OTM2201A>
    .drvInitialize    = NULL,
    .drvOpen          = DRV_GFX_OTM2201A_Open,
    .drvInterfaceSet  = DRV_GFX_OTM2201A_InterfaceSet,
</#if>
<#if CONFIG_USE_DRV_GFX_SSD1926>
    .drvInitialize    = NULL,
    .drvOpen          = DRV_GFX_SSD1926_Open,
    .drvInterfaceSet  = DRV_GFX_SSD1926_InterfaceSet,
</#if>
<#if CONFIG_USE_DRV_GFX_ILI9341>
    .drvInitialize    = NULL,
    .drvOpen          = DRV_GFX_ILI9341_Open,
    .drvInterfaceSet  = DRV_GFX_ILI9341_InterfaceSet,
</#if>
<#if CONFIG_USE_DRV_GFX_GLCD>
    .drvInitialize    = NULL,
    .drvOpen          = DRV_GFX_GLCD_Open,
    .drvInterfaceSet  = DRV_GFX_GLCD_InterfaceSet,
</#if>
    .preemptionLevel  = GFX_SELF_PREEMPTION_LEVEL
};
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
