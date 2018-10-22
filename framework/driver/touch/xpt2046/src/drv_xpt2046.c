/*******************************************************************************
 Touch controller XPT2046 driver file

  File Name:
    drv_XPT2046.c

  Summary:
    Touch controller XPT2046 driver interface file.

  Description:
    This file consist of touch controller XPT2046 driver interfaces. It
    implements the driver interfaces which read the touch input data from
    XPT2046 through I2C bus.
 ******************************************************************************/


#include <sys/attribs.h>
#include <sys/kmem.h>
#include "system/int/sys_int.h"
#include "system/touch/sys_touch.h"
#include "system/ports/sys_ports.h"
#include "driver/i2c/drv_i2c.h"
#include "driver/touch/XPT2046/drv_XPT2046.h"



typedef struct Point {
        long    x,
                y ;
        } POINT ;

typedef struct Matrix {
    long    An,     /* A = An/Divider */
             Bn,     /* B = Bn/Divider */
             Cn,     /* C = Cn/Divider */
             Dn,     /* D = Dn/Divider */
             En,     /* E = En/Divider */
             Fn,     /* F = Fn/Divider */
             Divider ;
 } MATRIX ;

int16_t XPT2046_Read16(SYS_MODULE_OBJ object, unsigned char Command);
char getDisplayPoint( POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr );
char setCalibrationMatrix( POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr);
void Calibrate();


#define XPT2046_SAMPLES 8 /* number of samples to take */
#define XPT2046_DEBOUNCE 5 /* Debound value in mS */
#define XPT2046_EVENTS 10 /* Number of queued touch events */

///@brief only need 4 commands for reading position or touch information
#define XPT2046_READ_Y  0x91    /* Read Y position*/
#define XPT2046_READ_Z1 0xb1	/* Read Z1 */
#define XPT2046_READ_Z2 0xc1    /* read Z2 */
#define XPT2046_READ_X  0xd1	/* Read X position */



/* Touch input data */
static int16_t PCapX[5]= {-1,-1,-1,-1,-1};
static int16_t PCapY[5] = { -1, -1, -1, -1, -1 };

MATRIX CalibrationData;

/* XPT2046 Driver instance object */
static DRV_TOUCH_XPT2046_OBJECT sXPT2046DriverInstances[DRV_TOUCH_XPT2046_INSTANCES_NUMBER];

/* XPT2046 Driver client object */
static DRV_TOUCH_XPT2046_CLIENT_OBJECT sXPT2046ClientInstances[DRV_TOUCH_XPT2046_CLIENTS_NUMBER];


// *****************************************************************************
// *****************************************************************************
// Section: Initialization
// *****************************************************************************
// *****************************************************************************




// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_TOUCH_XPT2046_Initialize(const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the XPT2046 instance for the specified driver index

  Description:
    This routine initializes the XPT2046 driver instance for the specified
    driver index, making it ready for clients to open and use it. 

  Precondition:
    None.

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

SYS_MODULE_OBJ DRV_TOUCH_XPT2046_Initialize( const SYS_MODULE_INDEX index, const SYS_MODULE_INIT * const init )
{
    const DRV_TOUCH_INIT *pInit = NULL;

    if ( index >= DRV_TOUCH_XPT2046_INDEX_COUNT )
    {
        SYS_ASSERT(false, "XPT2046 Driver: Attempting to initialize an instance number greater than the max");
        return SYS_MODULE_OBJ_INVALID;
    }

    DRV_TOUCH_XPT2046_OBJECT * pDrvInstance =
                ( DRV_TOUCH_XPT2046_OBJECT *)&sXPT2046DriverInstances[index];

    if ( pDrvInstance->inUse == true )
    {
        SYS_ASSERT(false, "XPT2046 Driver: Attempting to reinitialize a driver instance that is already in use");
        return SYS_MODULE_OBJ_INVALID;
    }

    pDrvInstance->inUse = true;

    pInit = (const DRV_TOUCH_INIT * const)init;

    pDrvInstance->touchId               = pInit->touchId;
    pDrvInstance->drvOpen               = pInit->drvOpen;
    pDrvInstance->orientation           = pInit->orientation;
    pDrvInstance->verticalResolution    = pInit->verticalResolution;
    pDrvInstance->horizontalResolution  = pInit->horizontalResolution;
    pDrvInstance->numClients            = 0;
    pDrvInstance->readRequest           = 0;
    pDrvInstance->drvSPIHandle          = DRV_HANDLE_INVALID;
    pDrvInstance->touchStatus           = DRV_TOUCH_POSITION_NONE;

    pDrvInstance->status = SYS_STATUS_READY;

    return (SYS_MODULE_OBJ)pDrvInstance;

}

/*************************************************************************
  Function:
       void DRV_TOUCH_XPT2046_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the XPT2046 driver module.

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

void DRV_TOUCH_XPT2046_Deinitialize ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_XPT2046_OBJECT * pDrvInstance = (DRV_TOUCH_XPT2046_OBJECT *)object;
    DRV_TOUCH_XPT2046_CLIENT_OBJECT *pClient = &sXPT2046ClientInstances[0];

    if( pDrvInstance == NULL )
    {
        SYS_ASSERT(false, "XPT2046 Driver: Attempting to deinitialize a NULL object");
        return;
    }

    if ( pDrvInstance->inUse == false )
    {
        SYS_ASSERT(false, "XPT2046 Driver: Attempting to deinitialize a driver instance that is not in use");
        return;
    }

    if( pClient->driverObject == (DRV_TOUCH_XPT2046_OBJECT * )pDrvInstance)
    {
        pClient->driverObject = NULL;
    }

    pDrvInstance->touchId               = 0xFF;
    pDrvInstance->verticalResolution    = 0;
    pDrvInstance->horizontalResolution  = 0;
    pDrvInstance->inUse                 = false;
    pDrvInstance->status                = SYS_STATUS_UNINITIALIZED;
    pDrvInstance->readRequest           = 0;

    return;
}

/**************************************************************************
  Function:
       SYS_STATUS DRV_TOUCH_XPT2046_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the XPT2046 driver module.

  Description:
    This function provides the current status of the XPT2046 driver module.

  Precondition:
    The DRV_TOUCH_XPT2046_Initialize function must have been called before
    calling this function.
*/

SYS_STATUS DRV_TOUCH_XPT2046_Status ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_XPT2046_OBJECT * pDrvInstance = ( DRV_TOUCH_XPT2046_OBJECT *)object;

    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        //SYS_ASSERT( " Handle is invalid " );
        return SYS_MODULE_OBJ_INVALID;
    }
    
    return pDrvInstance->status;
}

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_XPT2046_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified XPT2046 driver instance and returns a handle to it.

  Description:
    This routine opens the specified XPT2046 driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Precondition:
    The DRV_TOUCH_XPT2046_Initialize function must have been called before
    calling this function.

  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_TOUCH_XPT2046_Initialize().

    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver

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

DRV_HANDLE DRV_TOUCH_XPT2046_Open ( const SYS_MODULE_INDEX index,  const DRV_IO_INTENT intent )
{
    if (index >= DRV_TOUCH_XPT2046_INDEX_COUNT)
    {
        SYS_ASSERT(false, "XPT2046 Driver: Attempting to open an instance" \
                          "number greater than the max");
        return DRV_HANDLE_INVALID;
    }
    
    DRV_TOUCH_XPT2046_OBJECT * pDrvInstance =
                 ( DRV_TOUCH_XPT2046_OBJECT *)&sXPT2046DriverInstances[index];

    DRV_TOUCH_XPT2046_CLIENT_OBJECT *pClient = &sXPT2046ClientInstances[0];
    if (pClient == NULL)
    {
        SYS_ASSERT(false, "XPT2046 Driver: Couldn't find a free client to open");
        return DRV_HANDLE_INVALID;
    }

    pClient->driverObject = pDrvInstance;

    /* Open SPI driver */
    pDrvInstance->drvSPIHandle = DRV_SPI_Open(DRV_TOUCH_XPT2046_SPI_MODULE_INDEX, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING );
    if(pDrvInstance->drvSPIHandle == DRV_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "XPT2046 Driver: Failed to open SPI Driver");
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /*Default calibration data*/
    CalibrationData.An = 0x66F0;
    CalibrationData.Bn = 0x41F40;
    CalibrationData.Cn = 0xBD1B79BC;
    CalibrationData.Dn = 0x5ACC6;
    CalibrationData.En = 0x1CCA;
    CalibrationData.Fn = 0xA7450C8A;
    CalibrationData.Divider = 0xFFBF9A56;
    
    pClient->intent       = intent;
    pClient->pNext        = NULL;
    if ((intent & DRV_IO_INTENT_EXCLUSIVE) == DRV_IO_INTENT_EXCLUSIVE)
    {
        pDrvInstance->isExclusive = true;
    }
    
    pDrvInstance->numClients++;

    return (DRV_HANDLE)pClient;
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_XPT2046_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the XPT2046 driver

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

void DRV_TOUCH_XPT2046_Close ( DRV_HANDLE handle )
{
    DRV_TOUCH_XPT2046_CLIENT_OBJECT * pClient =
                                (DRV_TOUCH_XPT2046_CLIENT_OBJECT *)handle;
    DRV_TOUCH_XPT2046_OBJECT * pDrvObject = pClient->driverObject;

    if( pDrvObject == NULL )
    {
        SYS_ASSERT(false, "XPT2046 Driver: Trying to close a client with invalid driver object")
         return;
    }

    if (pDrvObject->numClients == 0)
    {
         SYS_ASSERT(false, "XPT2046 Driver: Trying to close a client which does not exist")
         return;
    }

    pDrvObject->numClients--;
    
    return;
}


/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_XPT2046_TouchStatus( )

  Summary:
    Returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_XPT2046_TouchStatus( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_XPT2046_OBJECT * pDrvInstance =
        (DRV_TOUCH_XPT2046_OBJECT *)&sXPT2046DriverInstances[index];
    return (pDrvInstance->touchStatus);
}


/*********************************************************************
  Function:
    void DRV_TOUCH_XPT2046_TouchDataRead( )

  Summary:
    Notify the driver that the current touch data has been read

*/
void DRV_TOUCH_XPT2046_TouchDataRead( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_XPT2046_OBJECT * pDrvInstance =
        (DRV_TOUCH_XPT2046_OBJECT *)&sXPT2046DriverInstances[index];
    pDrvInstance->touchStatus = DRV_TOUCH_POSITION_NONE;
}


/*********************************************************************
  Function:
    short DRV_TOUCH_XPT2046_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.

  Description:
    It returns the x coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the x coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_XPT2046_TouchGetX( uint8_t touchNumber )
{
    return (PCapX[touchNumber]);
}

/*********************************************************************
  Function:
    short DRV_TOUCH_XPT2046_TouchGetY( uint8 touchNumber )

  Summary:
    Returns the y coordinate of touch input.

  Description:
    It returns the y coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the y coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_XPT2046_TouchGetY( uint8_t touchNumber )
{
    return PCapY[touchNumber];
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_XPT2046_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. This routine decodes the touch input data
        available in drvI2CReadFrameData.

  Precondition:
    The DRV_TOUCH_XPT2046_Initialize routine must have been called for the
    specified XPT2046 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_XPT2046_Initialize)

  Returns:
    None.

*/

void DRV_TOUCH_XPT2046_Tasks ( SYS_MODULE_OBJ object )
{
    
   
    DRV_TOUCH_XPT2046_OBJECT * pDrvObject = (DRV_TOUCH_XPT2046_OBJECT *)object;
    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        return;
    }
    
    if(PORTDbits.RD0 != 0)
    {
        PCapX[0] = -1;
        PCapY[0] = -1;
        pDrvObject->touchStatus = DRV_TOUCH_POSITION_SINGLE;
        return;
    }

    POINT pt;
    POINT coord;
    char i;
    pt.x = 0;
    pt.y = 0;
    int x = XPT2046_Read16(object, 0x90);
    int y = XPT2046_Read16(object, 0xD0);
    int ReadedX;
    int ReadedY;
    //Add some averaging
    for(i = 0; i < 16; i++)
    {
        ReadedX = XPT2046_Read16(object, 0x90);
        ReadedY = XPT2046_Read16(object, 0xD0);
        //SYS_DEBUG_Print("Touch X: %ld \r\n", ReadedX);
        if(abs(ReadedX - x) > 100)
            return;
        if(abs(ReadedY - y) > 100)
            return;
        pt.x += ReadedX;
        pt.y += ReadedY;
    }
    pt.x = pt.x >> 4;
    pt.y = pt.y >> 4;

    //Translate ADC values to coordinates
    getDisplayPoint(&coord, &pt, &CalibrationData);
    if(coord.x < 240 && coord.x > 0 && coord.y < 320 && coord.y > 0)
    {
        switch(pDrvObject->orientation)
        {
            case 90:
                PCapX[0] = coord.x;
                PCapY[0] = coord.y;    
                break;
            case 180:
                PCapX[0] = coord.y;
                PCapY[0] = pDrvObject->verticalResolution - coord.x;    
                break;
            case 270:
                PCapX[0] = coord.x;
                PCapY[0] = pDrvObject->horizontalResolution - coord.y;    
                break;
            default:
                PCapX[0] = pDrvObject->horizontalResolution - coord.y;
                PCapY[0] = coord.x;    
                break;
        }
        
    }
    else
    {
        PCapX[0] = -1;
        PCapY[0] = -1;
    }
    pDrvObject->touchStatus = DRV_TOUCH_POSITION_SINGLE;
    return;
}


int16_t XPT2046_Read16(SYS_MODULE_OBJ object, unsigned char Command)
{
    DRV_TOUCH_XPT2046_OBJECT * pDrvObject = (DRV_TOUCH_XPT2046_OBJECT *)object;
    uint16_t val;
    unsigned char send[3] = {0,0,0};
    unsigned char readed[3];
    send[0] = Command;
    DRV_SPI_BUFFER_HANDLE spiBufferHandle = DRV_SPI_BufferAddWriteRead(pDrvObject->drvSPIHandle, send, 3,readed, 3, 0, 0);
    while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus (spiBufferHandle));

    val = readed[1];	// MSB
	val <<= 8;
	val |= readed[2];	// LSB
	val >>= 3;		
	if(val < 0)
		val = 0;
	if(val >4095)
		val = 4095;
    LATDbits.LATD4 = 1;
    return val;
}


//Get exact coordinates
char getDisplayPoint( POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr )
{
    char  retValue = 0 ;
    if( matrixPtr->Divider != 0 )
    {
        displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                          (matrixPtr->Bn * screenPtr->y) + 
                           matrixPtr->Cn 
                        ) / matrixPtr->Divider ;

        displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                          (matrixPtr->En * screenPtr->y) + 
                           matrixPtr->Fn 
                        ) / matrixPtr->Divider ;
    }
    else
    {
        retValue = -1 ;
    }

    return( retValue ) ;

}

char setCalibrationMatrix( POINT * displayPtr, POINT * screenPtr, MATRIX * matrixPtr)
{
    char  retValue = 0 ;
    matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -  ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
    if( matrixPtr->Divider == 0 )
    {
        retValue = -1 ;
    }
    else
    {
        matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

        matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
						(screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;

        matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) -  ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
    
        matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) -  ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                        (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
    }
 
    return( retValue ) ;

}
