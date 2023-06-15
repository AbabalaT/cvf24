#ifndef _CONTROL_H_
#define _CONTROL_H_
//#include "ano.h"
void upload_data(void);

void get_target_quaternion(void);
void get_target_angular_velocity(void);
void angular_rate_ctrl(void);
void height_rate_ctrl(float dt);
void ctrl_init(void);



#endif

