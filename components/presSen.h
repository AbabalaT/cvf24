/*
* 	* presSen.h
*/
#ifndef PRESSEN_INC_PRESSEN_H_
#define PRESSEN_INC_PRESSEN_H_

#include "stdint.h"

typedef struct _PresSen_Def{
	void (*init)(void);
	uint16_t (*get_SenVal)(void);

}PresSen_Def;

extern const PresSen_Def presSen;
#endif /* PRESSEN_INC_PRESSEN_H_ */
