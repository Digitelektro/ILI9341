<#--
/*******************************************************************************
  XPT2046 Driver Initialization File

  File Name:
    drv_xpt2046_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
 *******************************************************************************/
 -->
<#if CONFIG_USE_DRV_TOUCH_XPT2046 == true>
    sysObj.drvXPT2046 = DRV_TOUCH_XPT2046_Initialize(DRV_TOUCH_XPT2046_INDEX_0, (SYS_MODULE_INIT *)&drvTouchInitData);
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
