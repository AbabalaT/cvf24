/*
 * presSen_asm.s
 *
 *  Created on: 2022爛6堎17��
 *      Author: SystemUser
 */

.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

.global asm_Func_presSen_getVal

.set 	I2C2_BaseAddr,	 	0x40005800
.set 	CR1,				0x00
.set 	DR,					0x10
.set 	SR1,				0x14
.set 	SR2,				0x18

.set 	I2C_CR1_START,		0x100
.set 	I2C_CR1_STOP,		0x200
.set 	I2C_CR1_ACK,		0x400
.set	I2C_CR1_POS,		0x800
.set	I2C_CR1_PE,			0x01
.set	I2C_CR1_POS_ACK,	I2C_CR1_POS | I2C_CR1_ACK
.set 	I2C_SR1_ADDR, 		0x02
.set 	I2C_SR1_SB, 		0x01
.set 	I2C_SR1_BTF, 		0x04

.set 	sensor_addr,		0x51

	.section .text.asm_Func_presSen_getVal/*,"ax",%progbits*/
	.type asm_Func_presSen_getVal,%function
/*
*	Accroding to the manual, Case of two bytes to be received:
*	每 Set POS and ACK
*	每 Wait for the ADDR flag to be set
*	每 Clear ADDR
*	每 Clear ACK
*	每 Wait for BTF to be set
*	每 Program STOP
*	每 Read DR twice
*/

/*r3 is used to hold CR1, r4 for I2C2_BaseAddr, r5 for SR1, R6 for SR2*/
asm_Func_presSen_getVal:
	push	{r4-r7, lr}
/*for test*/
/*test over*/
	ldr	 	r4, =I2C2_BaseAddr; /* load the address of the I2C2 base register*/
	ldr  	r3, [r4, #CR1];

Enable_the_I2C2:
	orr 	r3, #I2C_CR1_PE	/*Set the PE*/
	str		r3, [r4, CR1];
Send_a_Start:
	orr		r3, #I2C_CR1_START  /*Send a start*/
	str		r3, [r4, CR1];

Check_if_the_i2c_start_isSent:
	ldr 	r5, [r4, #SR1];	/*Keep reading the sr1 register*/
	and		r5, #I2C_SR1_SB
	cmp		r5, #I2C_SR1_SB
	bne		Check_if_the_i2c_start_isSent;
Send_the_Addr:
	ldr 	r5, [r4, #SR1];
	mov		r3, #sensor_addr;
	str		r3, [r4, DR];

Set_POS_and_ACK:
	ldr  	r3, [r4, #CR1];
	orr 	r3, #I2C_CR1_POS_ACK	/*Set the ACK and POS*/
	str		r3, [r4, CR1];

Wait_for_the_ADDR_flag_to_be_set:
	ldr  	r5, [r4, #SR1];
	and		r5, #I2C_SR1_ADDR
	cmp		r5, #I2C_SR1_ADDR
	bne		Wait_for_the_ADDR_flag_to_be_set;

Clear_Addr:
	ldr		r5, [r4, #SR1];
	ldr		r6, [r4, #SR2];

Clear_ACK:
	ldr 	r3, [r4, #CR1];
	bic 	r3, I2C_CR1_ACK;
	str		r3, [r4, CR1];

Wait_for_BTF_to_be_set:
	ldr  	r5, [r4, #SR1];
	and		r5, #I2C_SR1_BTF
	cmp		r5, #I2C_SR1_BTF
	bne		Wait_for_BTF_to_be_set;

Program_STOP:
	ldr 	r3, [r4, #CR1];
	orr 	r3, I2C_CR1_STOP;
	str		r3, [r4, CR1];

Read_DR_twice:
	ldrb	r2, [r4, #DR]
	ldrb	r1, [r4, #DR]
Form_the_return_value:
	add 	r0, r1, r2, lsl 8;
Disable_I2C2:
	mov		r3, #0;
	str		r3, [r4, #CR1];
End_the_function:
	pop	 	{r4-r7, lr}
	bx 	 	lr
.size asm_Func_presSen_getVal, .-asm_Func_presSen_getVal
