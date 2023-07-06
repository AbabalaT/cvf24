/*
 * presSen.c
 *
 *  Created on: 2022Äê6ÔÂ11ÈÕ
 *      Author: SystemUser
 */

#include "presSen.h"
#include "main.h"
#include "stdbool.h"

/* The address of the sensor is actually 0x28. But the address used in the
 * program */



static void init(void);

/*Using get_SenVal() to call get_SenVal_unit() to make sure the structure preesSen
 * is placed in the .text section by the linker.*/
extern uint16_t asm_Func_presSen_getVal(void);

const PresSen_Def presSen = {
	.init	=	init,
	.get_SenVal	=	asm_Func_presSen_getVal,
};

void init(void){
}
