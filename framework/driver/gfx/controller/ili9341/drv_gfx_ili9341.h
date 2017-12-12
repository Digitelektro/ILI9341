/*******************************************************************************
 Module for Microchip Graphics Library

  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_ILI9341.h

  Summary:
    Interface for the graphics library, which initializes the SYSTECH ILI9341 
	Timing Controller.
	
  Description:
    This header file contains the function definition for the interface 
	to the SYSTECH ILI9341 Timing Controller.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _DRV_GFX_ILI9341_H
    #define _DRV_GFX_ILI9341_H
// DOM-IGNORE-END

#include "driver/gfx/controller/drv_gfx_controller.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* ILI9341 Driver Module Index Count

  Summary:
    Number of valid ILI9341 driver indices.

  Description:
    This constant identifies ILI9341 driver index definitions.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is device-specific.
*/

#define DRV_GFX_ILI9341_INDEX_COUNT     DRV_GFX_ILI9341_NUMBER_OF_MODULES


// *****************************************************************************
/*
  Structure: DRV_GFX_ILI9341_COMMAND

  Summary:
        Structure for the commands in the driver queue.

  Description:
        Structure for the commands in the driver queue.

  Input:
        address     - pixel address
        array       - pointer to array of pixel data
        data        - pixel color
*/
// *****************************************************************************

typedef struct
{
   uint32_t                     address;                 //whether or not the task is complete
   uint16_t                      *array;
   uint16_t                        data;
} DRV_GFX_ILI9341_COMMAND;

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function: uint8_t DRV_GFX_ILI9341_SetReg(uint16_t index, uint8_t value)

  Summary:
    updates graphics controller register value (byte access)

  Description:
    none

  Input:
    index - register number 
    value - value to write to register

  Output:
    1 - call was not passed
    0 - call was passed
*/
uint16_t DRV_GFX_ILI9341_SetReg(uint16_t index, uint8_t value);

// *****************************************************************************
/*
  Function: uint8_t DRV_GFX_ILI9341_GetReg(uint16_t index, uint8_t *data)

  Summary:
    returns graphics controller register value (byte access)

  Description:
    none

  Input:
    index - register number 
    *data - array to store data

  Output:
    0 - when call was passed
*/
uint8_t  DRV_GFX_ILI9341_GetReg(uint16_t  index, uint8_t *data);

/*********************************************************************
  Function:
     DRV_GFX_ILI9341_Open(uint8_t instance)
    
  Summary:
    opens an instance of the graphics controller

  Description:
    none

  Return:
                  
  *********************************************************************/
DRV_HANDLE DRV_GFX_ILI9341_Open( const SYS_MODULE_INDEX index,
                                 const DRV_IO_INTENT intent );

// *****************************************************************************
/*
  Function: void DRV_GFX_ILI9341_Close( DRV_HANDLE handle )

  Summary:
    closes an instance of the graphics controller

  Description:
    none

  Input:
    instance of the driver

*/
void DRV_GFX_ILI9341_Close( DRV_HANDLE handle );

/*********************************************************************
  Function:
     void DRV_GFX_ILI9341_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface )

  Summary:
    Returns the API of the graphics controller

  Description:
    none

  Return:

  *********************************************************************/
void DRV_GFX_ILI9341_InterfaceSet( DRV_HANDLE handle, DRV_GFX_INTERFACE * interface );

// *****************************************************************************
/*
  Function:
     void DRV_GFX_ILI9341_MaxXGet()

  Summary:
     Returns x extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_ILI9341_MaxXGet();

// *****************************************************************************
/*
  Function:
     void GFX_MaxYGet()

  Summary:
     Returns y extent of the display.

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    <code>

  Remarks:
*/
uint16_t DRV_GFX_ILI9341_MaxYGet();


// *****************************************************************************
/*
  Function: void DRV_GFX_ILI9341_SetColor(GFX_COLOR color)

  Summary: Sets the color for the driver instance

  Description:
  
  Output: none

*/

void DRV_GFX_ILI9341_SetColor(GFX_COLOR color);

// *****************************************************************************
/*
  Function: void DRV_GFX_ILI9341_SetInstance(uint8_t instance)

  Summary: Sets the instance for the driver

  Description:
  
  Output: none

*/

void DRV_GFX_ILI9341_SetInstance(uint8_t instance);

// *****************************************************************************
/*
  Function: uint16_t DRV_GFX_ILI9341_PixelPut(short x, short y)

  Summary:
    outputs one pixel into the frame buffer at the x,y coordinate given

  Description:
    none

  Input:
        x,y - pixel coordinates
  Output:
    NULL - call not successful
    !NULL - address of the display driver queue command
*/
void DRV_GFX_ILI9341_PixelPut(uint16_t x, uint16_t y);

// *****************************************************************************
/*
  Function: void  DRV_GFX_ILI9341_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount)

  Summary:
    outputs an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
              lineCount - number of lines
  Output:
        NULL - call not successful
        !NULL - handle to the number of pixels remaining
*/
void DRV_GFX_ILI9341_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount);

// *****************************************************************************
/*
  Function: uint16_t  DRV_GFX_ILI9341_PixelArrayGet(uint16_t *color, short x, short y, uint16_t count)

  Summary:
    gets an array of pixels of length count starting at *color 

  Description:
    none

  Input:
          instance - driver instance
          *color - start of the array
		  x - x coordinate of the start point.
		  y - y coordinate of the end point.
		  count - number of pixels
  Output:
         NULL - call not successful
         !NULL - address of the display driver queue command
*/ 
uint16_t* DRV_GFX_ILI9341_PixelArrayGet(uint16_t *color, uint16_t x, uint16_t y, uint16_t count);

// *****************************************************************************
/*
  Function: void DRV_GFX_ILI9341_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)

  Summary:
    Hardware accelerated barfill function

  Description:
    see primitive BarFill
  
  Output:
    1 - call not successful (PMP driver busy)
    0 - call successful
*/
void DRV_GFX_ILI9341_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom);

// *****************************************************************************
/*
  Function: SYS_MODULE_OBJ DRV_GFX_ILI9341_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                                          const SYS_MODULE_INIT    * const moduleInit)
  Summary:
    resets LCD, initializes PMP

  Description:
    none

  Input:
        instance - driver instance
  Output:
    1 - call not successful (PMP driver busy)
    0 - call successful
*/
SYS_MODULE_OBJ DRV_GFX_ILI9341_Initialize(const SYS_MODULE_INDEX   moduleIndex,
                                          const SYS_MODULE_INIT    * const moduleInit);

// *****************************************************************************
/*
  Function: uint16_t DRV_GFX_ILI9341_Busy(uint8_t instance)

  Summary:
    Returns non-zero if LCD controller is busy 
          (previous drawing operation is not completed).

  Description:
    none

  Input:
          instance - driver instance
  Output:
         1 - busy
         0 - not busy
*/ 
uint16_t DRV_GFX_ILI9341_Busy(uint8_t instance);

/*************************************************************************

  Function:
      void DRV_GFX_ILI9341_Tasks(void)
    
  Summary:
    Task machine that renders the driver calls for the graphics library it
    must be called periodically to output the contents of its circular
    buffer                                                                
  *************************************************************************/
void DRV_GFX_ILI9341_Tasks(SYS_MODULE_OBJ object);

/**************************************************************************
  Function:
       SYS_STATUS DRV_GFX_ILI9341_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the ILI9341 driver module.

  Description:
    This function provides the current status of the ILI9341 driver module.

  Conditions:
    The DRV_GFX_ILI9341_Initialize function must have been called before calling
    this function.

  Input:
    object -  Driver object handle, returned from DRV_GFX_ILI9341_Initialize

  Return:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_GFX_ILI9341_Initialize
    SYS_STATUS          status;

    status = DRV_GFX_ILI9341_Status( object );
    if( SYS_STATUS_READY != status )
    {
        // Handle error
    }
    </code>
*/
SYS_STATUS DRV_GFX_ILI9341_Status ( SYS_MODULE_OBJ object );

//DOM-IGNORE-BEGIN
/*********************************************************************
* Overview: ILI9341 registers definitions.
*********************************************************************/

//DOM-IGNORE-END

#ifdef __cplusplus
    }
#endif
    
#endif // _DRV_GFX_ILI9341_H



