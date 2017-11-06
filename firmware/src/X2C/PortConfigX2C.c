/* This file is part of X2C. http://www.mechatronic-simulation.org/                                                   */

#include "system_definitions.h"
#include "X2C.h"
#include "PortConfigX2C.h"

#define ENABLE_LED1     (SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0))
#define DISABLE_LED1    (SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0))
#define ENABLE_LED2     (SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1))
#define DISABLE_LED2    (SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1))
#define ENABLE_LED3     (SYS_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_2))
#define DISABLE_LED3    (SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_2))

#define BUTTON1_STATE   (SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12))
#define BUTTON2_STATE   (SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13))
#define BUTTON3_STATE   (SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14))

void readInports(void)
{
    Nop();
	/* TODO add linkage hardware-inputs -> X2C inports here */
    if (BUTTON1_STATE)
    {
        Inports.Button1 = INT16_MAX;
    }
    else
    {
        Inports.Button1 = 0;
    }
    
    if (BUTTON2_STATE)
    {
        Inports.Button2 = INT16_MAX;
    }
    else
    {
        Inports.Button2 = 0;
    }
    
    if (BUTTON3_STATE)
    {
        Inports.Button3 = INT16_MAX;
    }
    else
    {
        Inports.Button3 = 0;
    }
}

void writeOutports(void)
{
	/* TODO add linkage X2C outports -> hardware-outputs here */
    if (*(Outports.pLED1) > 0)
    {
        ENABLE_LED1;
    }
    else
    {
        DISABLE_LED1;
    }
    
    if (*(Outports.pLED2) > 0)
    {
        ENABLE_LED2;
    }
    else
    {
        DISABLE_LED2;
    }
    
    if (*(Outports.pLED3) > 0)
    {
        ENABLE_LED3;
    }
    else
    {
        DISABLE_LED3;
    }
}
