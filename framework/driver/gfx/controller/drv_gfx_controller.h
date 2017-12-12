/*******************************************************************
  Company:
    Microchip Technology Inc.
    
  File Name:
    gfx_display_driver.h

  Summary:
    GFX Display Driver definitions
    
  Description:
    GFX Display Driver definitions
    
    This file describes the GFX Display Driver specific definitions.
  *******************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
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

#ifndef DRV_GFX_DISPLAYDRIVER_H
    #define DRV_GFX_DISPLAYDRIVER_H
// DOM-IGNORE-END

#include "system_config.h"
#include "system_definitions.h"
#include "driver/gfx/gfx_common.h"
#include "driver/pmp/drv_pmp.h"

// DOM-IGNORE-BEGIN

#ifdef __cplusplus
    extern "C" {
#endif
        
#if !defined (GFX_CONFIG_DRIVER_COUNT)
    #define GFX_CONFIG_DRIVER_COUNT 1
#endif

// DOM-IGNORE-END

// *****************************************************************************
/*

  GFX Driver Module Index Numbers

  Summary:
    GFX display driver index definitions.

  Description:
    These constants provide the GFX display driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_GFX_<driver>_Initialize and
    DRV_GFX_<driver>_Open functions to identify the driver instance in use.
*/

typedef enum {

    DRV_GFX_INDEX_0 = 0,
    DRV_GFX_INDEX_1,
    DRV_GFX_INDEX_2,
    DRV_GFX_INDEX_3,
    NUMBER_OF_DRV_INDEXES

} DRV_GFX_INDEX;


// *****************************************************************************
/*  Data type for GFX DRV handle.

  Summary:
    Data type for GFX DRV handle.

  Description:
    The data type of the handle that is returned from GFX_DRV_Open
    function.

  Remarks:
    None.
*/

typedef uintptr_t DRV_GFX_HANDLE;

// *****************************************************************************
/* GFX Device Layer Invalid Handle

  Summary:
    Constant that defines the value of an Invalid Device Handle.

  Description:
    This constant is returned by the DRV_GFX_Open() function when the
    function fails.

  Remarks:
    None.
*/

#define DRV_GFX_HANDLE_INVALID /*DOM-IGNORE-BEGIN*/((DRV_GFX_HANDLE)(-1))/*DOM-IGNORE-END*/

// *****************************************************************************
/*
  Driver GFX Supported Display Drivers

  Summary:
    GFX supported Graphics Display Drivers.

  Description:
    These constants provide the GFX display driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_GFX_Open functions to
    identify the driver to use.
*/

typedef enum {
    DRV_GFX_SSD1926 = 0
} DRV_GFX_DISPLAY_DRIVER;

// *****************************************************************************
/* Driver State Machine States

   Summary
    Defines the various states that can be achieved by the driver operation.

   Description
    This enumeration defines the various states that can be achieved by the
    driver operation.

   Remarks:
    None.
*/

typedef enum
{
    /*  Driver state busy */
    DRV_GFX_STATE_BUSY,

    /*  Driver state init */
    DRV_GFX_STATE_INIT,

} DRV_GFX_STATES;


#define GFX_BUFFER1 (0)
#define GFX_BUFFER2 (1)

// DOM-IGNORE-END

// DOM-IGNORE-BEGIN
/*********************************************************************
* Function: Alpha Macros
********************************************************************/
#define 	NUM_ALPHA_LEVELS 	0x20			// Specific to device
#define 	ALPHA_DELTA 		((NUM_ALPHA_LEVELS) << 5)
/*********************************************************************
* Function: BYTE Percentage2Alpha(BYTE alphaPercentage)
********************************************************************/
extern inline uint8_t __attribute__ ((always_inline)) Percentage2Alpha(uint8_t alphaPercentage)
{
    uint32_t percent = (uint32_t)(alphaPercentage);
    percent *= (ALPHA_DELTA);
    percent /= 100;
    percent >>= 5;

    return (uint8_t)percent;
}
// DOM-IGNORE-END

typedef struct
{
   SYS_MODULE_OBJ (*init) (const SYS_MODULE_INDEX   index,
                          const SYS_MODULE_INIT    * const init);

   DRV_HANDLE      (*open) ( const SYS_MODULE_INDEX index,
                                 const DRV_IO_INTENT intent );
   
   DRV_GFX_INTERFACE * (*interfaceGet)( DRV_HANDLE handle );

} DRV_CLIENT_INTERFACE;

// *****************************************************************************
/*  Driver Module Client Status

  Summary:
    Enumerated data type that identifies the  Driver Module Client Status.

  Description:
    This enumeration defines the possible status of the  Driver Module Client.
    It is returned by the () function.

  Remarks:
    None.
*/

typedef enum
{
     /* Client is closed or the specified handle is invalid */
    DRV_GFX_STATUS_CLOSED
            /*DOM-IGNORE-BEGIN*/ = 0 /*DOM-IGNORE-END*/,

    /* Client is ready */
    DRV_GFX_STATUS_READY
            /*DOM-IGNORE-BEGIN*/ = 1 /*DOM-IGNORE-END*/

} DRV_GFX_CLIENT_STATUS;

// *****************************************************************************
/*  GFX Driver client object structure

  Summary:
    GFX Driver client object structure.

  Description:
    GFX Driver client object structure.

  Remarks:
   None.
*/

typedef struct
{
    /* Client status */
    DRV_GFX_CLIENT_STATUS         clientState;

    /* Set to true if this object is in use */
    bool                          inUse;

    /* Object from which the client is derived */
    void *                        drvObj;

} DRV_GFX_CLIENT_OBJ;

// DOM-IGNORE-BEGIN
// *********************************************************************
/* 
  Enumeration: PAGE_TYPE

  Summary: types of pages supported by the GFX Library

  Description: 

*/
typedef enum
{
    ACTIVE_PAGE   = 0,
    VISUAL_PAGE,
    BACKGROUND_PAGE,
    FOREGROUND_PAGE,
    DESTINATION_PAGE,
    TRANSITION_PAGE 
} PAGE_TYPE;
// DOM-IGNORE-END

// DOM-IGNORE-BEGIN
// *********************************************************************
/* 
  Enumeration: LAYER_TYPE

  Summary: types of Layers supported by the GFX Library

  Description: 

*/
typedef enum
{
    PIP1 = 0,
    PIP2
} LAYER_TYPE;
// DOM-IGNORE-END


#if(GFX_CONFIG_DRIVER_COUNT > 1)
    #define DRV_GFX_PixelPut(x,y)                           gfxDrv->PixelPut(x,y)
#else
    #ifdef GFX_USE_DISPLAY_CONTROLLER_LCC
    #define DRV_GFX_PixelPut(x,y)                           DRV_GFX_LCC_PixelPut(x,y)
    #endif
    #ifdef GFX_USE_DISPLAY_CONTROLLER_S1D13517
    #define DRV_GFX_PixelPut(x,y)                           DRV_GFX_S1D13517_PixelPut(x,y)
    #endif
    #ifdef  GFX_USE_DISPLAY_CONTROLLER_SSD1926
    #define DRV_GFX_PixelPut(x,y)                           DRV_GFX_SSD1926_PixelPut(x,y)
    #endif
    #ifdef  GFX_USE_DISPLAY_CONTROLLER_OTM2201A
    #define DRV_GFX_PixelPut(x,y)                           DRV_GFX_OTM2201A_PixelPut(x,y)
    #endif
	#ifdef  GFX_USE_DISPLAY_CONTROLLER_ILI9341
	#define DRV_GFX_PixelPut(x,y)                           DRV_GFX_ILI9341_PixelPut(x,y)
	#endif
#endif

#ifdef __cplusplus
    }
#endif
    
#endif
