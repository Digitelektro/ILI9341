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

void GFX_TMR_DelayMS(unsigned int delayMs);

#define  LCD_DataPort LATB    
/*  
#define LCD_RS		LATEbits.LATE7  		 
#define LCD_WR		LATEbits.LATE6  		  
#define LCD_RD		LATEbits.LATE5  			     
#define LCD_CS		LATGbits.LATG7  			
#define LCD_REST	LATGbits.LATG8	  
*/

void LCD_Writ_Bus(uint16_t data)   
{
#ifdef GFX_DRV_ILI9341_MODE_SPI
    DRV_SPI_BUFFER_HANDLE spiBufferHandle = DRV_SPI_BufferAddWrite(drvILI9341Obj.drvSPIHandle, &data, 2, 0,0);
    while(DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus (spiBufferHandle));
#endif
    
#ifdef GFX_DRV_ILI9341_MODE_16BIT
	LCD_DataPort = data;	
    LATEbits.LATE3 = data & 0b01;
    data = data >> 1;
    LATEbits.LATE4 = data & 0b01;
	//LCD_WR=0;
	//LCD_WR=1; 
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_WR_PORT, LCD_WR_BIT_POS);
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_WR_PORT, LCD_WR_BIT_POS);
#endif
}



void LCD_WR_DATA(uint16_t da)
{
    //LCD_RS=1;
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_RS_PORT, LCD_RS_BIT_POS);
	LCD_Writ_Bus(da);
}	
  
void LCD_WR_REG(uint16_t da)	 
{	
    //LCD_RS=0;
	PLIB_PORTS_PinClear(PORTS_ID_0, LCD_RS_PORT, LCD_RS_BIT_POS);
	LCD_Writ_Bus(da);
}

void LCD_WR_REG_DATA(uint16_t reg,uint16_t da)
{
    LCD_WR_REG(reg);
	LCD_WR_DATA(da);
}

void Address_set( uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{ 
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(x1>>8);
	LCD_WR_DATA(x1);
	LCD_WR_DATA(x2>>8);
	LCD_WR_DATA(x2);
	
	LCD_WR_REG(0x2B);
	LCD_WR_DATA(y1>>8);
	LCD_WR_DATA(y1);
	LCD_WR_DATA(y2>>8);
	LCD_WR_DATA(y2);	
	LCD_WR_REG(0x2c);					 						 
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
			//            SYS_DEBUG(0, "Client Handle no inuse");
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
Description:
Precondition:
Parameters:
Returns:
Example:
<code>
<code>
Remarks:
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

Description:

Precondition:

Parameters:

Returns:

Example:
<code>
<code>

Remarks:
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
        drvILI9341Obj.drvSPIHandle = DRV_SPI_Open(DRV_ILI9341_SPI_MODULE_INDEX, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING );
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
    
	/*
    LCD_CS =1; 
	LCD_RD=1;
	LCD_WR=1;
	LCD_REST=0;
	GFX_TMR_DelayMS(20);	
	LCD_REST=1;	
	GFX_TMR_DelayMS(20);			
	LCD_CS =0;  
	*/
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_CS_PORT, LCD_CS_BIT_POS);
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_RD_PORT, LCD_RD_BIT_POS);
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_WR_PORT, LCD_WR_BIT_POS);
	PLIB_PORTS_PinClear(PORTS_ID_0, LCD_REST_PORT, LCD_REST_BIT_POS);
	GFX_TMR_DelayMS(20);	
	PLIB_PORTS_PinSet(PORTS_ID_0, LCD_REST_PORT, LCD_REST_BIT_POS);
	GFX_TMR_DelayMS(20);			
	PLIB_PORTS_PinClear(PORTS_ID_0, LCD_CS_PORT, LCD_CS_BIT_POS);

	//************* Start Initial Sequence **********//
	LCD_WR_REG(0xcf); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0xc1);
	LCD_WR_DATA(0x30);
	
	LCD_WR_REG(0xed); 
	LCD_WR_DATA(0x64);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0x81);
	
	LCD_WR_REG(0xcb); 
	LCD_WR_DATA(0x39);
	LCD_WR_DATA(0x2c);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x34);
	LCD_WR_DATA(0x02);
	
	LCD_WR_REG(0xea); 
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	
	LCD_WR_REG(0xe8); 
	LCD_WR_DATA(0x85);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x79);
	
	LCD_WR_REG(0xC0); //Power control
	LCD_WR_DATA(0x23); //VRH[5:0]
	
	LCD_WR_REG(0xC1); //Power control
	LCD_WR_DATA(0x11); //SAP[2:0];BT[3:0]
	
	LCD_WR_REG(0xC2);
	LCD_WR_DATA(0x11);
	
	LCD_WR_REG(0xC5); //VCM control
	LCD_WR_DATA(0x3d);
	LCD_WR_DATA(0x30);
	
	LCD_WR_REG(0xc7); 
	LCD_WR_DATA(0xaa);
	
	LCD_WR_REG(0x3A); 
	LCD_WR_DATA(0x55);
	
	LCD_WR_REG(0x36); // Memory Access Control, screen rotation
    switch(drvILI9341Obj.initData->orientation)
    {
        case 90:
            LCD_WR_DATA(0x08);
            break;
        case 180:
            LCD_WR_DATA(0x68);
            break;
        case 270:
            LCD_WR_DATA(0x88);
            break;
        default:
            LCD_WR_DATA(0xA8);
            break;
                
    }
    

	LCD_WR_REG(0xB1); // Frame Rate Control
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x11);
	
	LCD_WR_REG(0xB6); // Display Function Control
	LCD_WR_DATA(0x0a);
	LCD_WR_DATA(0xa2);
	
	LCD_WR_REG(0xF2); // 3Gamma Function Disable
	LCD_WR_DATA(0x00);
	
	LCD_WR_REG(0xF7);
	LCD_WR_DATA(0x20);
	
	LCD_WR_REG(0xF1);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x30);
	
	LCD_WR_REG(0x26); //Gamma curve selected
	LCD_WR_DATA(0x01);
	
	LCD_WR_REG(0xE0); //Set Gamma
	LCD_WR_DATA(0x0f);
	LCD_WR_DATA(0x3f);
	LCD_WR_DATA(0x2f);
	LCD_WR_DATA(0x0c);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x0a);
	LCD_WR_DATA(0x53);
	LCD_WR_DATA(0xd5);
	LCD_WR_DATA(0x40);
	LCD_WR_DATA(0x0a);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x00);
	
	LCD_WR_REG(0xE1); //Set Gamma
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x0f);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x2c);
	LCD_WR_DATA(0xa2);
	LCD_WR_DATA(0x3f);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x0e);
	LCD_WR_DATA(0x0c);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x3c);
	LCD_WR_DATA(0x0F);
	LCD_WR_REG(0x11); 
	GFX_TMR_DelayMS(80);
	LCD_WR_REG(0x29); //display on
    
    
    
	

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

void  DRV_GFX_ILI9341_LineDraw(short x1, short y1, short x2, short y2)
{
	uint16_t t; 
	int16_t xerr=0,yerr=0,delta_x,delta_y,distance; 
	int16_t incx,incy,uRow,uCol; 

	delta_x=x2-x1; 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 
	else if(delta_x==0)incx=0;
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )
	{  
		DRV_GFX_ILI9341_PixelPut(uRow,uCol);
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}

// *****************************************************************************
/*
Function: void  DRV_GFX_ILI9341_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)

Summary:
Hardware accelerated barfill function

Description:
see primitive BarDraw

Output:
1 - call not successful (PMP driver busy)
0 - call successful
*/
void  DRV_GFX_ILI9341_BarFill(uint16_t left, uint16_t top, uint16_t right, uint16_t bottom)
{
    uint16_t i,j; 
	Address_set(left,top,right,bottom);      
	for(i=top;i<=bottom;i++)
	{													   	 	
		for(j=left;j<=right;j++)
            LCD_WR_DATA(drvILI9341Obj.initData->color); 	    
	}
    /*if(left == 0 && top == 0 && right == 0 && bottom == 0)
    {
        Address_set(0,0,240,320);      
        for(i=0;i<=320;i++)
        {													   	 	
            for(j=0;j<=240;j++)
                LCD_WR_DATA(drvILI9341Obj.initData->color); 	    
        }
    }*/

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
    Address_set(x,y,x,y);
	LCD_WR_DATA(drvILI9341Obj.initData->color);

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
	Address_set(x,y,x + count,y + lineCount);      
	for(i=0;i<=lineCount;i++)
	{													   	 	
		for(j=0;j<=count;j++)
            LCD_WR_DATA(color[j]); 	    
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




void GFX_TMR_DelayMS(unsigned int delayMs)
{
	#if (true == SYS_TMR_INTERRUPT_MODE)

	#else      //Primitive Blocking Mode
		if (delayMs)
		{
			uint32_t sysClk = SYS_CLK_FREQ;
			uint32_t t0;
	#if   defined (__C32__)
			t0 = _CP0_GET_COUNT();
			while (_CP0_GET_COUNT() - t0 < (sysClk / 2000)*delayMs);
	#elif defined (__C30__)
			t0 = ReadTimer23();
			while (ReadTimer23() - t0 < (sysClk / 2000)*mSec);
	#else
	#error "Neither __C32__ nor __C30__ is defined!"
	#endif
		}
	#endif
}





