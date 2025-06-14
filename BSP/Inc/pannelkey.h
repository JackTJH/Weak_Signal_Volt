#ifndef __PANNELKEY_H
#define __PANNELKEY_H


#include "main.h"


/* ----------------------------------//
//		按键排布与对应键值
//
// 		k13 	k14 	k15 	k16
// 		k9 		k10 	k11 	k12
// 		k5 		k6		k7		k8
// 		k1		k2		k3		k4
//
//---------------------------------- */
#define	Key_Val_NoKey		 0
#define	Key_Val_K1			 1
#define	Key_Val_K2			 2
#define	Key_Val_K3			 3
#define	Key_Val_K4			 4
#define	Key_Val_K5			 5
#define	Key_Val_K6			 6
#define	Key_Val_K7			 7
#define	Key_Val_K8			 8
#define	Key_Val_K9			 9
#define	Key_Val_K10			10
#define	Key_Val_K11			11
#define	Key_Val_K12			12
#define	Key_Val_K13			13
#define	Key_Val_K14			14
#define	Key_Val_K15			15
#define	Key_Val_K16			16



extern uint8_t Global_Key_Val;


void Key_Detect_Last(void);
void Key_Action_Last(uint8_t key);
void Key_Detect(void);
void Key_Action_Main(void);


#endif
