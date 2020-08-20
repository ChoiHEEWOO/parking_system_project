/*
 * timer_lib.h
 *
 * Created: 2020-08-18 오후 3:06:18
 *  Author: CHOI
 */ 
#ifndef TIMER_LIB_H
#define TIMER_LIB_H
#include <avr/io.h>
void timer0_init();
//about timer3 initialization
void timer3_init();
//about Timer3 set data
void setOCR3A(int num);
void setOCR3B(int num);
void setOCR3C(int num);
void setICR3(int num);

#endif