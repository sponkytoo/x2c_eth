/**
 * @file
 * @brief Generated model file.

 * Date:  2016-04-11 16:20
 * X2C-Version: 910
 */
/* This file is part of X2C. http://www.mechatronic-simulation.org/                                                   */

/* Model: DemoApplication                                                                                             */
/* Date:  2016-04-11 16:20                                                                                            */

/* X2C-Version: 910                                                                                                   */

#ifndef __X2C_H__
#define __X2C_H__

/**********************************************************************************************************************/
/**     Includes                                                                                                     **/
/**********************************************************************************************************************/
#include "Sin3Gen_FiP16.h"
#include "Constant_FiP16.h"
#include "SinGen_FiP16.h"
#include "AutoSwitch_FiP16.h"
#include "Scope_Main.h"
#include "CommonFcts.h"

/**********************************************************************************************************************/
/**     Defines                                                                                                      **/
/**********************************************************************************************************************/
#define FUNCTIONS SIN3GEN_FIP16_FUNCTIONS , \
                  CONSTANT_FIP16_FUNCTIONS , \
                  SINGEN_FIP16_FUNCTIONS , \
                  AUTOSWITCH_FIP16_FUNCTIONS , \
                  SCOPE_MAIN_FUNCTIONS 

#define PARAMETER_TABLE { (uint16)1, &TAmplitude } , \
                        { (uint16)2, &TAutoSwitch } , \
                        { (uint16)3, &TFrequency } , \
                        { (uint16)4, &TLED_off } , \
                        { (uint16)5, &TLED_on } , \
                        { (uint16)6, &TLED2Control1 } , \
                        { (uint16)7, &TSin3Gen } , \
                        { (uint16)8, &TSinGen } , \
                        { (uint16)9, &TScope } , \
                        { (uint16)10, &Inports.Button1 } , \
                        { (uint16)11, &Inports.Button2 } , \
                        { (uint16)12, &Inports.Button3 } , \
                        { (uint16)13, &Outports.pLED1 } , \
                        { (uint16)14, &Outports.pLED3 } , \
                        { (uint16)15, &Outports.pLED2 } 

/**********************************************************************************************************************/
/**     Typdefs                                                                                                      **/
/**********************************************************************************************************************/

/* Inport structure                                                                                                   */
typedef struct 
{
    int16 Button2;
    int16 Button3;
    int16 Button1;
} TInports;

/* Outport structure                                                                                                  */
typedef struct 
{
    int16* pLED1;
    int16* pLED3;
    int16* pLED2;
} TOutports;

/**********************************************************************************************************************/
/**     Externals                                                                                                    **/
/**********************************************************************************************************************/
extern TInports Inports;
extern TOutports Outports;
extern const tBlockFunctions blockFunctionTable[];
extern const tParameterTable parameterIdTable[];


/**********************************************************************************************************************/
/**     Prototypes                                                                                                   **/
/**********************************************************************************************************************/
void X2C_Init(void);
void X2C_Update(void);
void X2C_Update_1(void);


#endif
