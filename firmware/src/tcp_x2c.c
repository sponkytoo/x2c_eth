/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    tcp.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "tcp.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

#define TCP_PORT ((uint16)12666)

TCP_X2C_DATA tcp_x2cData;

#define RX_BUF_SIZE (16*1024)
#define TX_BUF_SIZE (16*1024)

#define LNET_BUF_SIZE (255)

static tTcpIp tcpip;
static uint8 rxBuffer[RX_BUF_SIZE];
static uint8 txBuffer[TX_BUF_SIZE];


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
uint32_t initTcpIp(tTcpIp* tcpip, uint8_t* rxBuffer, uint16_t rxSize, int8_t* txBuffer, uint16_t txSize);


/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void TCP_X2C_Initialize ( void )

  Remarks:
    See prototype in tcp.h.
 */

void TCP_X2C_Initialize(void) {
    /* Place the App state machine in its initial state. */
    tcp_x2cData.state = TCP_X2C_STATE_INIT;
    SYS_CONSOLE_MESSAGE(" TCP/IP Server Task Initialization start\r\n");

    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void TCP_X2C_Tasks ( void )

  Remarks:
    See prototype in tcp.h.
 */

void TCP_X2C_Tasks(void) {
    SYS_STATUS tcpipStat;
    const char *netName, *netBiosName;
    static IPV4_ADDR dwLastIP[2] = {
        {-1},
        {-1}
    };
    IPV4_ADDR ipAddr;
    int i, nNets;
    TCPIP_NET_HANDLE netH;

    SYS_CMD_READY_TO_READ();
    /* Check the application's current state. */
    switch (tcp_x2cData.state) {
            /* Application's initial state. */
        case TCP_X2C_STATE_INIT:
            tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if (tcpipStat < 0) { // some error occurred
                SYS_CONSOLE_MESSAGE(" APP: TCP/IP stack initialization failed!\r\n");
                tcp_x2cData.state = TCP_X2C_STATE_ERROR;
            } else if (tcpipStat == SYS_STATUS_READY) {
                // now that the stack is ready we can check the
                // available interfaces
                nNets = TCPIP_STACK_NumberOfNetworksGet();
                for (i = 0; i < nNets; i++) {

                    netH = TCPIP_STACK_IndexToNet(i);
                    netName = TCPIP_STACK_NetNameGet(netH);
                    netBiosName = TCPIP_STACK_NetBIOSName(netH);

#if defined(TCPIP_STACK_USE_NBNS)
                    SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
#else
                    SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS disabled\r\n", netName, netBiosName);
#endif  // defined(TCPIP_STACK_USE_NBNS)

                }
                tcp_x2cData.state = TCP_X2C_STATE_WAIT_FOR_IP;

            }
            break;

        case TCP_X2C_STATE_WAIT_FOR_IP:

            // if the IP address of an interface has changed
            // display the new value on the system console
            nNets = TCPIP_STACK_NumberOfNetworksGet();

            for (i = 0; i < nNets; i++) {
                netH = TCPIP_STACK_IndexToNet(i);
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                if (dwLastIP[i].Val != ipAddr.Val) {
                    dwLastIP[i].Val = ipAddr.Val;

                    SYS_CONSOLE_MESSAGE(TCPIP_STACK_NetNameGet(netH));
                    SYS_CONSOLE_MESSAGE(" IP Address: ");
                    SYS_CONSOLE_PRINT("%d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                    if (ipAddr.v[0] != 0 && ipAddr.v[0] != 169) // Wait for a Valid IP
                    {
                        tcp_x2cData.state = TCP_X2C_STATE_OPENING_SERVER;
                    }
                }
            }
            break;
        case TCP_X2C_STATE_OPENING_SERVER:
        {
            SYS_CONSOLE_PRINT("Waiting for Client Connection on port: %d\r\n", TCP_PORT);
            tcp_x2cData.socket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, TCP_PORT, 0);
            if (tcp_x2cData.socket == INVALID_SOCKET) {
                SYS_CONSOLE_MESSAGE("Couldn't open server socket\r\n");
                break;
            }
            tcp_x2cData.state = TCP_X2C_STATE_WAIT_FOR_CONNECTION;
        }
            break;

        case TCP_X2C_STATE_WAIT_FOR_CONNECTION:
        {
            if (!TCPIP_TCP_IsConnected(tcp_x2cData.socket)) {
                return;
            } else {
                // We got a connection
                tcp_x2cData.state = TCP_X2C_STATE_SERVING_CONNECTION;
                initTcpIp(&tcpip, rxBuffer, RX_BUF_SIZE, txBuffer, TX_BUF_SIZE);
                SYS_CONSOLE_MESSAGE("Received a connection\r\n");
            }
        }
            break;

        case TCP_X2C_STATE_SERVING_CONNECTION:
        {
            if (!TCPIP_TCP_IsConnected(tcp_x2cData.socket)) {
                tcp_x2cData.state = TCP_X2C_STATE_CLOSING_CONNECTION;
                SYS_CONSOLE_MESSAGE("Connection was closed\r\n");
                break;
            }
        }
            break;
        case TCP_X2C_STATE_CLOSING_CONNECTION:
        {
            // Close the socket connection.
            TCPIP_TCP_Close(tcp_x2cData.socket);

            tcp_x2cData.state = TCP_X2C_STATE_OPENING_SERVER;

        }
            break;
            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

uint32_t initTcpIp(tTcpIp* tcpip, uint8_t* rxBuffer, uint16_t rxSize, int8_t* txBuffer, uint16_t txSize) {
    tcpip->receive = (uint8(*)(tTcpIp*))readByte;
    tcpip->send = (uint8(*)(tTcpIp*, uint8))sendByte;
    tcpip->isReceiveDataAvailable = (uint8(*)(tTcpIp*))isDataAvailable;
    tcpip->isSendReady = (uint8(*)(tTcpIp*))isReadyToSend;
    tcpip->flush = (void(*)(tTcpIp*))flush;

    initBuffer(&tcpip->txBuffer, txBuffer, txSize);
    initBuffer(&tcpip->rxBuffer, rxBuffer, rxSize);

    return 0;
}

void initCommunication(void) {

}

void linkCommunication(tProtocol* protocol) {
    protocol->hwInterface = (tInterface*) & tcpip;
}

uint8 sendByte(tTcpIp* tcpip, uint8 data) {
    TCPIP_TCP_Put(tcp_x2cData.socket, data);
}

uint8 readByte(tTcpIp* tcpip) {
    uint8_t data;
    TCPIP_TCP_Get(tcp_x2cData.socket, &data);
    return data;
}

uint8 isReadyToSend(tTcpIp* tcpip) {
    uint16_t data;
    data = TCPIP_TCP_PutIsReady(tcp_x2cData.socket);
    if (data)
        return 1;
    else
        return 0;
}

uint8 isDataAvailable(tTcpIp* tcpip) {
    uint16_t data;
    data = TCPIP_TCP_GetIsReady(tcp_x2cData.socket);
    if (data)
        return 1;
    else
        return 0;
}

void flush(tTcpIp* tcpip) {
    TCPIP_TCP_Flush(tcp_x2cData.socket);
}

uint8 getTxFifoFree(tTcpIp* tcpip) {
    return ((uint8) 0);
}

uint8 initCommunicationHardware(void) {
    uint8 success;

    if (tcp_x2cData.state == TCP_X2C_STATE_SERVING_CONNECTION)
        success = 1;
    else
        success = 0;

    return (success);
}

uint8 isCommunicationConfigured(void) {
    uint8 success;

    if (tcp_x2cData.state == TCP_X2C_STATE_SERVING_CONNECTION)
        success = 1;
    else
        success = 0;
    return (success);
}

/*******************************************************************************
 End of File
 */
