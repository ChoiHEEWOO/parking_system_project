/*
 * timer_lib.h
 *
 * Created: 2020-08-18 오후 3:06:18
 *  Author: CHOI
 */ 
#ifndef TIMER_LIB_H
#define TIMER_LIB_H
#include <avr/io.h>

extern void timer0_init();
//about timer3 initialization
extern void timer3_init();
//about Timer3 set data
extern void setOCR3A(int num);
extern void setOCR3B(int num);
extern void setOCR3C(int num);
extern void setICR3(int num);

#endif