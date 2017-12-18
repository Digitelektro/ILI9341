/*******************************************************************************
 Module for Microchip Graphics Library

  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_ili9341.c

  Summary:
    This module initializes the ILI9341 Timing Controller (TCON) Module.
*******************************************************************************/


#include "../drv_gfx_ili9341.h"
#include "system/tmr/sys_tmr.h"
#include "driver/gfx/gfx_common.h"
#include "driver/spi/drv_spi.h"

// *****************************************************************************
/* ILI9341 Driver task states

  Summary
    Lists the different states that ILI9341 task routine can have.

  Description
    This enumeration lists the different states that ILI9341 task routine can have.

  Remarks:
    None.
*/

typedef enum
{
    /* Process queue */
    DRV_GFX_ILI9341_INITIALIZE_START,

    DRV_GFX_ILI9341_INITIALIZE_DONE,

} DRV_GFX_ILI9341_OBJECT_TASK;


// *****************************************************************************
/* GFX ILI9341 Driver Instance Object

Summary:
Defines the object required for the maintenance of the hardware instance.

Description:
This defines the object required for the maintenance of the hardware
instance. This object exists once per hardware instance of the peripheral.

Remarks:
None.
*/

typedef struct _DRV_GFX_ILI9341_OBJ
{
	/* Flag to indicate in use  */
	bool                                        inUse;
	/* Save the index of the driver */
	SYS_MODULE_INDEX                            drvIndex;
	/* ILI9341 machine state */
	DRV_GFX_STATES                              state;
	/* Status of this driver instance */
	SYS_STATUS                                  status;
	/* Number of clients */
	uint32_t                                    nClients;
    /* SPI Driver Handle  */
    DRV_HANDLE                                  drvSPIHandle;
	/* Client of this driver */
	DRV_GFX_CLIENT_OBJ *                        pDrvILI9341ClientObj;
	/* State of the task */
	DRV_GFX_ILI9341_OBJECT_TASK                 task;
	DRV_GFX_INIT *                              initData;
	uint16_t                                    maxY;
	uint16_t                                    maxX;
} DRV_GFX_ILI9341_OBJ;

static DRV_GFX_ILI9341_OBJ        drvILI9341Obj;

static DRV_GFX_CLIENT_OBJ drvILI9341Clients;

void GFX_TMR_DelayMS(uint16_t delayMs);
void GFX_TMR_DelayuS(uint16_t delayus);

#define  LCD_DataPort LATB    



/*
Function: ILI9341_Write_Bus(uint16_t data)  

Summary:
 Send data to ILI9341
 
Input:
 Data to be written

*/
void ILI9341_Write_Bus(uint16_t data)   
{
    //PLIB_PORTS_PinClear(PORTS_ID_0, LCD_CS_PORT, LCD_CS_BIT_POS);
#ifdef GFX_DRV_ILI9341_MODE_SPI
    //DRV_SPI_BUFFER_HANDLE spiBufferHandle = DRV_SPI_BufferAddWrite(drvILI9341Obj.drvSPIHandle, &data, 1, 0,0);
    //while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus (spiBufferHandle));
    uint8_t received;
    //Use Faster Blocking function to send data
    switch(DRV_ILI9341_SPI_MODULE_INDEX)
    {
        case DRV_SPI_INDEX_0:
        {
            DRV_SPI0_BufferAddWriteRead(&data, &received, 1);
            break;
        }
        case DRV_SPI_INDEX_1:
        {
            DRV_SPI1_BufferAddWriteRead(&data, &received, 1);
            break;
        }
        case DRV_SPI_INDEX_2:
        {
            DRV_SPI2_BufferAddWriteRead(&data, &received, 1);
            break;
        }
        case DRV_SPI_INDEX_3:
        {
            DRV_SPI3_BufferAddWriteRead(&data, &received, 1);
            break;
        }
        case DRV_SPI_INDEX_4:
        {
            DRV_SPI4_BufferAddWriteRead(&data, &received, 1);
            break;
        }
        case DRV_SPI_INDEX_5:
        {
            DRV_SPI5_BufferAddWriteRead(&data, &received, 1);
            break;
        }
        default:
        {
            break;
        }
    }
    
#endif
    
#ifdef GFX_DRV_ILI9341_MODE_16BIT
	LCD_DataPort = data;
    /*Uncomment the following three lines if you want to use debug pins, and connect the ILI9341 two LSB bit to the defined I/O PIN*/
    //LATEbits.LATE3 = data & 0b01; //Use RE3 pin insted of RB0 pin
    //data = data >> 1; 
    //LATEbits.LATE4 = data & 0b01; //Use RE4 pin insted of RB1 pin
	PLIB_PORTS_PinClear(PORTS_ID_0, LCD_WR_PORT, LCD_WR_BIT_POS);
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_WR_PORT, LCD_WR_BIT_POS);
#endif
    //PLIB_PORTS_PinSet(PORTS_ID_0, LCD_CS_PORT, LCD_CS_BIT_POS);
}


/*
Function: ILI9341_WR_DATA(uint16_t data)

Summary:
 Write ILI9341 data register
 
Input:
 Data to be written

*/
void ILI9341_WR_DATA(uint16_t data)
{
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_RS_PORT, LCD_RS_BIT_POS);
    ILI9341_Write_Bus(data);
}	
 
/*
Function: ILI9341_WR_REG(uint16_t data)

Summary:
 Write ILI9341 command register
 
Input:
 Data to be written

*/
void ILI9341_WR_REG(uint16_t data)	 
{	
	PLIB_PORTS_PinClear(PORTS_ID_0, LCD_RS_PORT, LCD_RS_BIT_POS);
	ILI9341_Write_Bus(data);
}

/*
Function: ILI9341_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)

Summary:
 Set ILI9341 drawing window area
 
Input:
 Window coordinates

*/
void ILI9341_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{ 
	ILI9341_WR_REG(0x2A);
	ILI9341_WR_DATA(x1>>8);
	ILI9341_WR_DATA(x1 & 0xFF);
	ILI9341_WR_DATA(x2>>8);
	ILI9341_WR_DATA(x2 & 0xFF);
	
	ILI9341_WR_REG(0x2B);
	ILI9341_WR_DATA(y1>>8);
	ILI9341_WR_DATA(y1 & 0xFF);
	ILI9341_WR_DATA(y2>>8);
	ILI9341_WR_DATA(y2 & 0xFF);	
	ILI9341_WR_REG(0x2c);		
}

/*
Function: ILI9341_Color_Set(uint16_t Color)

Summary:
 Set pixel color
 
Input:
    Color

*/
void ILI9341_Color_Set(uint16_t Color)
{
#ifdef GFX_DRV_ILI9341_MODE_SPI
    ILI9341_WR_DATA(Color>>8);
#endif
	ILI9341_WR_DATA(Color);
}

// *****************************************************************************
/*
Function: ILI9341_Init()

Summary:
    Initialize ILI9341 LCD controller

*/
void ILI9341_Init()
{
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_CS_PORT, LCD_CS_BIT_POS);
#ifdef GFX_DRV_ILI9341_MODE_16BIT
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_RD_PORT, LCD_RD_BIT_POS);
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_WR_PORT, LCD_WR_BIT_POS);
#endif
	PLIB_PORTS_PinClear(PORTS_ID_0, LCD_REST_PORT, LCD_REST_BIT_POS);
	GFX_TMR_DelayMS(20);	
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_REST_PORT, LCD_REST_BIT_POS);
	GFX_TMR_DelayMS(20);	
	PLIB_PORTS_PinClear(PORTS_ID_0, LCD_CS_PORT, LCD_CS_BIT_POS);
    
	//************* Start Initial Sequence **********//
	ILI9341_WR_REG(0xcf); 
	ILI9341_WR_DATA(0x00);
	ILI9341_WR_DATA(0xc1);
	ILI9341_WR_DATA(0x30);
	
	ILI9341_WR_REG(0xed); 
	ILI9341_WR_DATA(0x64);
	ILI9341_WR_DATA(0x03);
	ILI9341_WR_DATA(0x12);
	ILI9341_WR_DATA(0x81);
	
	ILI9341_WR_REG(0xcb); 
	ILI9341_WR_DATA(0x39);
	ILI9341_WR_DATA(0x2c);
	ILI9341_WR_DATA(0x00);
	ILI9341_WR_DATA(0x34);
	ILI9341_WR_DATA(0x02);
	
	ILI9341_WR_REG(0xea); 
	ILI9341_WR_DATA(0x00);
	ILI9341_WR_DATA(0x00);
	
	ILI9341_WR_REG(0xe8); 
	ILI9341_WR_DATA(0x85);
	ILI9341_WR_DATA(0x10);
	ILI9341_WR_DATA(0x79);
	
	ILI9341_WR_REG(0xC0); //Power control
	ILI9341_WR_DATA(0x23); //VRH[5:0]
	
	ILI9341_WR_REG(0xC1); //Power control
	ILI9341_WR_DATA(0x11); //SAP[2:0];BT[3:0]
	
	ILI9341_WR_REG(0xC2);
	ILI9341_WR_DATA(0x11);
	
	ILI9341_WR_REG(0xC5); //VCM control
	ILI9341_WR_DATA(0x3d);
	ILI9341_WR_DATA(0x30);
	
	ILI9341_WR_REG(0xc7); 
	ILI9341_WR_DATA(0xaa);
	
	ILI9341_WR_REG(0x3A); 
	ILI9341_WR_DATA(0x55);
	
	ILI9341_WR_REG(0x36); // Memory Access Control, screen rotation
    switch(drvILI9341Obj.initData->orientation)
    {
        case 90:
            ILI9341_WR_DATA(0x08);
            break;
        case 180:
            ILI9341_WR_DATA(0x68);
            break;
        case 270:
            ILI9341_WR_DATA(0x88);
            break;
        default:
            ILI9341_WR_DATA(0xA8);
            break;
                
    }

	ILI9341_WR_REG(0xB1); // Frame Rate Control
	ILI9341_WR_DATA(0x00);
	ILI9341_WR_DATA(0x11);
	
	ILI9341_WR_REG(0xB6); // Display Function Control
	ILI9341_WR_DATA(0x0a);
	ILI9341_WR_DATA(0xa2);
	
	ILI9341_WR_REG(0xF2); // 3Gamma Function Disable
	ILI9341_WR_DATA(0x00);
	
	ILI9341_WR_REG(0xF7);
	ILI9341_WR_DATA(0x20);
	
	ILI9341_WR_REG(0xF1);
	ILI9341_WR_DATA(0x01);
	ILI9341_WR_DATA(0x30);
	
	ILI9341_WR_REG(0x26); //Gamma curve selected
	ILI9341_WR_DATA(0x01);
	
	ILI9341_WR_REG(0xE0); //Set Gamma
	ILI9341_WR_DATA(0x0f);
	ILI9341_WR_DATA(0x3f);
	ILI9341_WR_DATA(0x2f);
	ILI9341_WR_DATA(0x0c);
	ILI9341_WR_DATA(0x10);
	ILI9341_WR_DATA(0x0a);
	ILI9341_WR_DATA(0x53);
	ILI9341_WR_DATA(0xd5);
	ILI9341_WR_DATA(0x40);
	ILI9341_WR_DATA(0x0a);
	ILI9341_WR_DATA(0x13);
	ILI9341_WR_DATA(0x03);
	ILI9341_WR_DATA(0x08);
	ILI9341_WR_DATA(0x03);
	ILI9341_WR_DATA(0x00);
	
	ILI9341_WR_REG(0xE1); //Set Gamma
	ILI9341_WR_DATA(0x00);
	ILI9341_WR_DATA(0x00);
	ILI9341_WR_DATA(0x10);
	ILI9341_WR_DATA(0x03);
	ILI9341_WR_DATA(0x0f);
	ILI9341_WR_DATA(0x05);
	ILI9341_WR_DATA(0x2c);
	ILI9341_WR_DATA(0xa2);
	ILI9341_WR_DATA(0x3f);
	ILI9341_WR_DATA(0x05);
	ILI9341_WR_DATA(0x0e);
	ILI9341_WR_DATA(0x0c);
	ILI9341_WR_DATA(0x37);
	ILI9341_WR_DATA(0x3c);
	ILI9341_WR_DATA(0x0F);
	ILI9341_WR_REG(0x11); 
	GFX_TMR_DelayMS(80);
	ILI9341_WR_REG(0x29); //display on
    
}

// *****************************************************************************
/*
Function: DRV_GFX_ILI9341_Open(uint8_t instance)

Summary:
opens an instance of the graphics controller

Description:
none

Input:
instance of the driver

Output:
1 - driver not initialied
2 - instance doesn't exist
3 - instance already open
instance to driver when successful
*/
DRV_GFX_HANDLE DRV_GFX_ILI9341_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent)
{
	DRV_GFX_CLIENT_OBJ * client = (DRV_GFX_CLIENT_OBJ *)DRV_HANDLE_INVALID;

	/* Check if instance object is ready*/
	if (drvILI9341Obj.status != SYS_STATUS_READY)
	{
		/* The ILI9341 module should be ready */
		//        SYS_DEBUG(0, "GFX_ILI9341 Driver: Was the driver initialized?");
	}
	else if (intent != DRV_IO_INTENT_EXCLUSIVE)
	{
		/* The driver only supports this mode */
		//SYS_DEBUG(0, "GFX_ILI9341 Driver: IO intent mode not supported");
	}
	else if (drvILI9341Obj.nClients > 0)
	{
		/* Driver supports exclusive open only */
		//SYS_DEBUG(0, "GFX_ILI9341 already opened once. Cannot open again");
	}
	else
	{
        ILI9341_Init();
		client = &drvILI9341Clients;
		client->inUse = true;
		client->drvObj = &drvILI9341Obj;

		/* Increment the client number for the specific driver instance*/
		drvILI9341Obj.nClients++;
	}

	/* Return invalid handle */
	return ((DRV_HANDLE)client);
}

// *****************************************************************************
/* Function:
void DRV_GFX_ILI9341_Close( DRV_HANDLE handle )

Summary:
closes an instance of the graphics controller

Description:
This is closes the instance of the driver specified by handle.
*/
void DRV_GFX_ILI9341_Close(DRV_HANDLE handle)
{
	/* Start of local variable */
	DRV_GFX_CLIENT_OBJ * client = (DRV_GFX_CLIENT_OBJ *)NULL;
	DRV_GFX_ILI9341_OBJ * drvObj = (DRV_GFX_ILI9341_OBJ *)NULL;
	/* End of local variable */

	/* Check if the handle is valid */
	if (handle == DRV_HANDLE_INVALID)
	{
		//SYS_DEBUG(0, "Bad Client Handle");
	}
	else
	{
		client = (DRV_GFX_CLIENT_OBJ *)handle;

		if (client->inUse)
		{
			client->inUse = false;
			drvObj = (DRV_GFX_ILI9341_OBJ *)client->drvObj;

			/* Remove this client from the driver client table */
			drvObj->nClients--;
		}
		else
		{
			//SYS_DEBUG(0, "Client Handle no inuse");
		}
	}
	return;
}

// *****************************************************************************
/*
Function:
	void DRV_GFX_ILI9341_MaxXGet()
Summary:
	Returns x extent of the display.
*/
uint16_t DRV_GFX_ILI9341_MaxXGet()
{
	return drvILI9341Obj.maxX;
}

// *****************************************************************************
/*
Function:
void GFX_MaxYGet()

Summary:
Returns y extent of the display.
*/
uint16_t DRV_GFX_ILI9341_MaxYGet()
{
	return drvILI9341Obj.maxY;
}

/*********************************************************************
Function:
DRV_GFX_INTEFACE DRV_GFX_ILI9341_InterfaceGet( DRV_HANDLE handle )

Summary:
Returns the API of the graphics controller

Description:
none

Return:

*********************************************************************/
void DRV_GFX_ILI9341_InterfaceSet(DRV_HANDLE handle, DRV_GFX_INTERFACE * interface)
{
	interface->BarFill = DRV_GFX_ILI9341_BarFill;
	interface->PixelArrayPut = DRV_GFX_ILI9341_PixelArrayPut;
	interface->PixelArrayGet = DRV_GFX_ILI9341_PixelArrayGet;
	interface->PixelPut = DRV_GFX_ILI9341_PixelPut;
	interface->ColorSet = DRV_GFX_ILI9341_SetColor;
	interface->InstanceSet = DRV_GFX_ILI9341_SetInstance;
	interface->MaxXGet = DRV_GFX_ILI9341_MaxXGet;
	interface->MaxYGet = DRV_GFX_ILI9341_MaxYGet;

}

// *****************************************************************************
/*
Function: uint16_t DRV_GFX_ILI9341_Initialize(uint8_t instance)

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
SYS_MODULE_OBJ DRV_GFX_ILI9341_Initialize(const SYS_MODULE_INDEX   index, const SYS_MODULE_INIT * const init)
{
    static uint16_t  horizontalSize, verticalSize;

	/* Validate the driver index */
	if (index >= GFX_CONFIG_NUMBER_OF_MODULES)
	{
		return SYS_MODULE_OBJ_INVALID;
	}

	DRV_GFX_ILI9341_OBJ *dObj = &drvILI9341Obj;
    
    /* Object is valid, set it in use */
	dObj->inUse = true;
	dObj->state = SYS_STATUS_BUSY;
	dObj->initData = (DRV_GFX_INIT *)init;
#ifdef GFX_DRV_ILI9341_MODE_SPI
        drvILI9341Obj.drvSPIHandle = DRV_SPI_Open(DRV_ILI9341_SPI_MODULE_INDEX, DRV_IO_INTENT_READWRITE);
        if(drvILI9341Obj.drvSPIHandle == DRV_HANDLE_INVALID)
        {
            SYS_ASSERT(false, "ILI9341 Driver: Failed to open SPI Driver");
            return SYS_MODULE_OBJ_INVALID;
        }
#endif
    
    if((drvILI9341Obj.initData->orientation == 90) || (drvILI9341Obj.initData->orientation == 270))
    {
        horizontalSize = drvILI9341Obj.initData->verticalResolution;
        verticalSize   = drvILI9341Obj.initData->horizontalResolution;
        drvILI9341Obj.maxX = horizontalSize - 1;
        drvILI9341Obj.maxY = verticalSize - 1;
    }
    else
    {
        horizontalSize = drvILI9341Obj.initData->horizontalResolution;
        verticalSize   = drvILI9341Obj.initData->verticalResolution;
        drvILI9341Obj.maxX = horizontalSize - 1;
        drvILI9341Obj.maxY = verticalSize - 1;
    }
    
    

	/* Save the index of the driver. Important to know this
	as we are using reference based accessing */
	dObj->drvIndex = index;

	dObj->task = DRV_GFX_ILI9341_INITIALIZE_DONE;

	dObj->nClients = 0;
	dObj->status = SYS_STATUS_READY;

	/* Return the driver handle */
	return (SYS_MODULE_OBJ)dObj;

}

// *****************************************************************************
/*
Function: void DRV_GFX_ILI9341_SetColor(GFX_COLOR color)

Summary: Sets the color for the driver instance

Description:

Output: none

*/

void DRV_GFX_ILI9341_SetColor(GFX_COLOR color)
{

	drvILI9341Obj.initData->color = color;
}

// *****************************************************************************
/*
Function: void DRV_GFX_ILI9341_SetInstance(uint8_t instance)

Summary: Sets the instance for the driver

Description:

Output: none

*/

void DRV_GFX_ILI9341_SetInstance(uint8_t instance)
{
	instance = instance;
}


// *****************************************************************************
/*
Function: void  DRV_GFX_ILI9341_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)

Summary:
Barfill function

Description:

*/
void  DRV_GFX_ILI9341_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
    uint16_t i,j; 
	ILI9341_Address_Set(left,top,right,bottom);      
	for(i=top;i<=bottom;i++)
	{													   	 	
		for(j=left;j<=right;j++)
        {
            ILI9341_Color_Set(drvILI9341Obj.initData->color); 	   
        }
        Nop();
	}

}

// *****************************************************************************
/*
Function: void DRV_GFX_ILI9341_PixelPut(uint16_t x, uint16_t y)

Summary:
outputs one pixel into the frame buffer at the x,y coordinate given

Description:
none

Input:
instance - driver instance
color - color to output
x,y - pixel coordinates
Output:
1 - call not successful (ILI9341 driver busy)
0 - call successful
*/
void DRV_GFX_ILI9341_PixelPut(uint16_t x, uint16_t y)
{
    ILI9341_Address_Set(x,y,x,y);
	ILI9341_Color_Set(drvILI9341Obj.initData->color);

}

// *****************************************************************************
/*
Function: void  DRV_GFX_ILI9341_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)

Summary:
outputs an array of pixels of length count starting at *color

Description:
none

Input:
instance - driver instance
*color - start of the array
x - x coordinate of the start point.
y - y coordinate of the end point.
count - number of pixels
Output:
handle to the number of pixels remaining
*/
void  DRV_GFX_ILI9341_PixelArrayPut(uint16_t *color, uint16_t x, uint16_t y, uint16_t count, uint16_t lineCount)
{
    uint16_t i,j; 
	ILI9341_Address_Set(x,y,x + count,y + lineCount);      
	for(i=0;i<=lineCount;i++)
	{													   	 	
		for(j=0;j<=count;j++)
            ILI9341_Color_Set(color[j]); 	    
	} 		
}

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
1 - call not successful (ILI9341 driver busy)
0 - call successful
*/
uint16_t*  DRV_GFX_ILI9341_PixelArrayGet(uint16_t *color, uint16_t x, uint16_t y, uint16_t count)
{
	/*Not yet supported*/
	return(NULL);
}

// *****************************************************************************
/*
Function: uint16_t  DRV_GFX_ILI9341_Busy(uint8_t instance)

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
uint16_t  DRV_GFX_ILI9341_Busy(uint8_t instance)
{
	uint8_t status = 0x00;
	//Get status
	return((bool)(!status));
}

// *****************************************************************************
/* Function:
SYS_STATUS DRV_GFX_ILI9341_Status ( SYS_MODULE_OBJ object )

Summary:
Returns status of the specific module instance of the Driver module.

Description:
This function returns the status of the specific module instance disabling its
operation (and any hardware for driver modules).

PreCondition:
The DRV_GFX_ILI9341_Initialize function should have been called before calling
this function.

Parameters:
object          - DRV_GFX_ILI9341 object handle, returned from DRV_GFX_ILI9341_Initialize

Returns:
SYS_STATUS_READY    Indicates that any previous module operation for the
specified module has completed

SYS_STATUS_BUSY     Indicates that a previous module operation for the
specified module has not yet completed

SYS_STATUS_ERROR    Indicates that the specified module is in an error state
*/

SYS_STATUS DRV_GFX_ILI9341_Status(SYS_MODULE_OBJ object)
{
	DRV_GFX_ILI9341_OBJ *dObj = (DRV_GFX_ILI9341_OBJ*)object;
	return (dObj->state);

} /* SYS_TMR_Status */



// *****************************************************************************
/*
Function: void  GFX_TMR_DelayMS(uint16_t delayMs)

Summary:
 * Blocking delay function

*/
void GFX_TMR_DelayMS(uint16_t delayMs)
{
    if (delayMs)
    {
        uint32_t sysClk = SYS_CLK_FREQ;
        uint32_t t0;
        t0 = _CP0_GET_COUNT();
        while (_CP0_GET_COUNT() - t0 < (sysClk / 2000)*delayMs);
    }
}

// *****************************************************************************
/*
Function: void  GFX_TMR_DelayuS(uint16_t delayus)

Summary:
 * Blocking delay function

*/
void GFX_TMR_DelayuS(uint16_t delayus)
{
	if (delayus)
    {
        uint32_t sysClk = SYS_CLK_FREQ;
        uint32_t t0;
        t0 = _CP0_GET_COUNT();
        while (_CP0_GET_COUNT() - t0 < (sysClk / 2000000)*delayus);
    }
}





