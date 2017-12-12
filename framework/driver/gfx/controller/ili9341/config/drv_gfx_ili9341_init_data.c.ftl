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

/*** GFX Initialization Data ***/

<#-- Instance 0 -->
GFX_INIT gfxInit0 =
{
    .drvInitialize    = DRV_GFX_ILI9341_Initialize,
    .drvOpen          = DRV_GFX_ILI9341_Open,
    .drvInterfaceSet  = DRV_GFX_ILI9341_InterfaceSet,
    .driverInitData.orientation             = DISP_ORIENTATION,
    .driverInitData.horizontalResolution    = DISP_HOR_RESOLUTION,
    .driverInitData.verticalResolution      = DISP_VER_RESOLUTION,
    .driverInitData.dataWidth               = DISP_DATA_WIDTH,
    .driverInitData.horizontalPulseWidth    = DISP_HOR_PULSE_WIDTH,
    .driverInitData.horizontalBackPorch     = DISP_HOR_BACK_PORCH,
    .driverInitData.horizontalFrontPorch    = DISP_HOR_FRONT_PORCH,
    .driverInitData.verticalPulseWidth      = DISP_VER_PULSE_WIDTH,
    .driverInitData.verticalBackPorch       = DISP_VER_BACK_PORCH,
    .driverInitData.verticalFrontPorch      = DISP_VER_FRONT_PORCH,
    .driverInitData.logicShift              = DISP_INV_LSHIFT,
    .driverInitData.LCDType                 = 1,
    .driverInitData.colorType               = 0,
    .driverInitData.TCON_Init               = TCON_MODULE,
};
<#--

<#if CONFIG_GFX_INST_1 == true>
GFX_INIT gfxInit1 =
{
    .drvInitialize    = DRV_GFX_ILI9341_Initialize,
    .drvOpen          = DRV_GFX_ILI9341_Open,
    .drvInterfaceSet  = DRV_GFX_ILI9341_InterfaceSet,
    .driverInitData.orientation             = DISP_ORIENTATION,
    .driverInitData.horizontalResolution    = DISP_HOR_RESOLUTION,
    .driverInitData.verticalResolution      = DISP_VER_RESOLUTION,
    .driverInitData.dataWidth               = DISP_DATA_WIDTH,
    .driverInitData.horizontalPulseWidth    = DISP_HOR_PULSE_WIDTH,
    .driverInitData.horizontalBackPorch     = DISP_HOR_BACK_PORCH,
    .driverInitData.horizontalFrontPorch    = DISP_HOR_FRONT_PORCH,
    .driverInitData.verticalPulseWidth      = DISP_VER_PULSE_WIDTH,
    .driverInitData.verticalBackPorch       = DISP_VER_BACK_PORCH,
    .driverInitData.verticalFrontPorch      = DISP_VER_FRONT_PORCH,
    .driverInitData.logicShift              = DISP_INV_LSHIFT,
    .driverInitData.LCDType                 = 1,
    .driverInitData.colorType               = 0,
    .driverInitData.TCON_Init               = TCON_MODULE,
};
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
