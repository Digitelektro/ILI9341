/*******************************************************************************
 Touch Controller XPT2046 Driver Interface File

  File Name:
    drv_XPT2046.c

  Summary:
    Touch controller XPT2046 Driver interface header file.

  Description:
    This header file describes the macros, data structure and prototypes of the 
    touch controller XPT2046 driver interface.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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
 ******************************************************************************/
// DOM-IGNORE-END

#ifndef _DRV_TOUCH_XPT2046_H
#define _DRV_TOUCH_XPT2046_H

#include "driver/touch/drv_touch.h"
#include "driver/spi/drv_spi.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/* XPT2046 Driver Handle

  Summary:
    Touch screen controller XPT2046 driver handle.

  Description:
    Touch controller XPT2046 driver handle is a handle for the driver
    client object. Each driver with succesful open call will return a new handle
    to the client object.

  Remarks:
    None.
*/

typedef uintptr_t DRV_TOUCH_XPT2046_HANDLE;



// *****************************************************************************
/* XPT2046 Driver Invalid Handle

  Summary:
    Definition of an invalid handle.

  Description:
    This is the definition of an invalid handle. An invalid handle is 
    is returned by DRV_TOUCH_XPT2046_Open() and DRV_XPT2046_Close()
    functions if the request was not successful.

  Remarks:
    None.
*/

#define DRV_TOUCH_XPT2046_HANDLE_INVALID ((DRV_TOUCH_XPT2046_HANDLE)(-1))


// *****************************************************************************
/* XPT2046 Driver Module Index Numbers

  Summary:
    XPT2046 driver index definitions.

  Description:
    These constants provide the XPT2046 driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_XPT2046_Initialize and
    DRV_XPT2046_Open functions to identify the driver instance in use.
*/

#define DRV_TOUCH_XPT2046_INDEX_0         0

// *****************************************************************************
/* XPT2046 Driver Module Index Count

  Summary:
    Number of valid Touch controller XPT2046 driver indices.

  Description:
    This constant identifies the number of valid Touch Controller XPT2046
    driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

#define DRV_TOUCH_XPT2046_INDEX_COUNT     1


typedef enum {

    XPT2046_ID_1 = 0,
    XPT2046_NUMBER_OF_MODULES
            
} DRV_TOUCH_XPT2046_MODULE_ID;

// *****************************************************************************
/* XPT2046 Touch Controller Driver Task State

  Summary:
    Enumeration defining XPT2046 touch controller driver task state.

  Description:
    This enumeration defines the XPT2046 touch controller driver task state.
    The task state helps to synchronize the operations of initialization the
    the task, adding the read input task to the task queue once the touch
    controller notifies the available touch input and a decoding the touch input
    received.

  Remarks:
    None.
*/

typedef enum
{
    /* Task initialize state */
    DRV_TOUCH_XPT2046_TASK_STATE_INIT = 0,

    /* Task read touch input request state */
    DRV_TOUCH_XPT2046_TASK_STATE_READ_INPUT,

    /* Task touch input decode state */
    DRV_TOUCH_XPT2046_TASK_STATE_DECODE_INPUT,
            
    /* Task complete state */
    DRV_TOUCH_XPT2046_TASK_STATE_DONE,

} DRV_TOUCH_XPT2046_TASK_STATE;

// *****************************************************************************
/* XPT2046 Touch Controller driver task data structure.

  Summary:
    Defines the XPT2046 Touch Controller driver task data structure.

  Description:
    This data type defines the data structure maintaing task context in the task
    queue. The inUse flag denotes the task context allocation for a task.
    The enum variable taskState maintains the current task state. The SPI
    buffer handle drvSPIReadBufferHandle maintains the SPI driver buffer handle
    returned by the SPI driver read request. The byte array variable
    drvSPIReadFrameData maintains the SPI frame data sent by XPT2046 after a
    successful read request.

  Remarks:
    None.
*/
typedef struct
{
    /* Flag denoting the allocation of task */
    bool                            inUse;

    /* Enum maintaining the task state */
    DRV_TOUCH_XPT2046_TASK_STATE   taskState;

    /* SPI Buffer handle */
    DRV_SPI_BUFFER_HANDLE           drvSPIReadBufferHandle;


} DRV_TOUCH_XPT2046_TASK_QUEUE;

// *****************************************************************************
/* XPT2046 Driver Instance Object.

  Summary:
    Defines the data structure maintaining XPT2046 driver instance object.

  Description:
    This data structure maintains the XPT2046 driver instance object. The
    object exists once per hardware instance.

  Remarks:
    None.
*/

typedef struct
{
    /* The status of the driver */
    SYS_STATUS                      status;

    /* The peripheral Id associated with the object */
    int                             touchId;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    SYS_MODULE_INDEX                drvIndex;

    /* Flag to indicate instance in use  */
    bool                            inUse;

    /* Flag to indicate module used in exclusive access mode */
    bool                            isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t                         numClients;

    /* Touch input interrupt source */
    INT_SOURCE                      interruptSource;

    /* Orientation of the display (given in degrees of 0,90,180,270) */
    uint16_t                        orientation;

    /* Horizontal Resolution of the displayed orientation in Pixels */
    uint16_t                        horizontalResolution;

    /* Vertical Resolution of the displayed orientaion in Pixels */
    uint16_t                        verticalResolution;

    /* Callback for SPI Driver Open call */
    DRV_HANDLE                      (*drvOpen) ( const SYS_MODULE_INDEX index,
                                                const DRV_IO_INTENT intent );

    /* Touch Input read request counter */
    int32_t                         readRequest;

    /* SPI bus driver handle */
    DRV_HANDLE                      drvSPIHandle;

    /* Touch status */
    DRV_TOUCH_POSITION_STATUS       touchStatus;

} DRV_TOUCH_XPT2046_OBJECT;

// *****************************************************************************
/* XPT2046 Driver client object

  Summary:
    XPT2046 Driver client object maintaining client data.

  Description:
    This defines the object required for the maintenance of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

typedef struct _DRV_XPT2046_CLIENT_OBJECT
{
    /* Driver Object associated with the client */
    DRV_TOUCH_XPT2046_OBJECT*                      driverObject;

    /* The intent with which the client was opened */
    DRV_IO_INTENT                                   intent;

    /* Next driver client object */
    struct DRV_TOUCH_XPT2046_CLIENT_OBJECT*        pNext;

} DRV_TOUCH_XPT2046_CLIENT_OBJECT;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_TOUCH_XPT2046_Initialize(const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the XPT2046 instance for the specified driver index.
	<p><b>Implementation:</b> Dynamic</p>

  
  Parameters:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the XPT2046 ID.  The hardware XPT2046 ID is set in the 
             initialization structure. This is the index of the driver index to
             use.

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver. If this pointer is NULL, the driver
             uses the static initialization override macros for each
             member of the initialization data structure.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

*/

SYS_MODULE_OBJ DRV_TOUCH_XPT2046_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init );

/*************************************************************************
  Function:
       void DRV_TOUCH_XPT2046_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the XPT2046 driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the XPT2046 driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Preconditions:
    Function DRV_TOUCH_XPT2046_Initialize must have been called before calling 
    this routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameter:
    object -  Driver object handle, returned from DRV_TOUCH_XPT2046_Initialize

  Returns:
    None.
*/

void DRV_TOUCH_XPT2046_Deinitialize ( SYS_MODULE_OBJ object );


/**************************************************************************
  Function:
       SYS_STATUS DRV_TOUCH_XPT2046_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the XPT2046 driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function provides the current status of the XPT2046 driver module.

  Precondition:
    The DRV_TOUCH_XPT2046_Initialize function must have been called before 
    calling this function.

  Parameters:
    object -  Driver object handle, returned from DRV_TOUCH_XPT2046_Initialize

  Returns:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system-level operation and cannot start another

  
*/

SYS_STATUS DRV_TOUCH_XPT2046_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_TOUCH_XPT2046_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. This routine decodes the touch input data
        available in drvSPIReadFrameData.

  Precondition:
    The DRV_TOUCH_XPT2046_Initialize routine must have been called for the 
    specified XPT2046 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_XPT2046_Initialize)

  Returns:
    None.

*/

void DRV_TOUCH_XPT2046_Tasks ( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_XPT2046_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified XPT2046 driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
	
  Description:
    This routine opens the specified XPT2046 driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The current version of driver does not support the DRV_IO_INTENT feature.
    The driver is by default non-blocking.	
	
  Precondition:
    The DRV_TOUCH_XPT2046_Initialize function must have been called before 
    calling this function.
	
  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_TOUCH_XPT2046_Initialize().
                
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver. The current version of driver does not support
				the selective IO intent feature.
				
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_TOUCH_XPT2046_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid
		
*/

DRV_HANDLE DRV_TOUCH_XPT2046_Open ( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    intent );

// *****************************************************************************
/* Function:
    void DRV_TOUCH_XPT2046_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the XPT2046 driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function closes an opened instance of the XPT2046 driver, invalidating
    the handle.

  Precondition:
    The DRV_TOUCH_XPT2046_Initialize routine must have been called for the 
    specified XPT2046 driver instance.

    DRV_TOUCH_XPT2046_Open must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
 
*/

void DRV_TOUCH_XPT2046_Close ( DRV_HANDLE handle );

/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_XPT2046_TouchStatus( const SYS_MODULE_INDEX index )

  Summary:
    Returns the status of the current touch input.

  Description:
    It returns the status of the current touch input.

  Parameters
    None.

  Returns
    It returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_XPT2046_TouchStatus( const SYS_MODULE_INDEX index );


/*********************************************************************
  Function:
    void DRV_TOUCH_XPT2046_TouchDataRead( const SYS_MODULE_INDEX index )

  Summary:
    Notifies the driver that the current touch data has been read

  Description:
    Notifies the driver that the current touch data has been read

  Parameters
    None.

  Returns
    None.

*/
void DRV_TOUCH_XPT2046_TouchDataRead( const SYS_MODULE_INDEX index );


/*********************************************************************
  Function:
    short DRV_TOUCH_XPT2046_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    It returns the x coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the x coordinate of the touch input in terms of number of pixels.

*/
short DRV_TOUCH_XPT2046_TouchGetX( uint8_t touchNumber );


/*********************************************************************
  Function:
    short DRV_TOUCH_XPT2046_TouchGetY( uint8 touchNumber )

  Summary:
    Returns the y coordinate of touch input.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    It returns the y coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the y coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_XPT2046_TouchGetY( uint8_t touchNumber );

#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_TOUCH_XPT2046_H