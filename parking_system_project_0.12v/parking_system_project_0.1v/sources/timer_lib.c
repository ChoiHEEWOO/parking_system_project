/*
 * timer_lib.c
 *
 * Created: 2020-08-18 오후 3:06:34
 *  Author: CHOI
 */ 
#include "timer_lib.h"

void timer0_init()
{
	//16Mhz > 64 prescailing(CS:100) >>0.25MHz/count 
	//CTC mode , OC0 disconnected
	DDRB |= (1<<DDRB4);
	TCCR0 = ((1<<CS02) | (0<< CS01) | (0<<CS00) | (1<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00));
	OCR0=250;
	TIMSK |= (1<<OCIE0); //to using tick TIMER
	
}


void timer3_init(){
	//TIM3
	// prescailing : 8
	// if ICR==4000 : 62.5Hz > 실제 주파수 31.25Hz
	//
	//COM : 01 : Toggle (in CTC)
	//WGM : 1100 : CTC : TOP : ICR3
	
	DDRE |= (1<<DDRE3);
	
	TCCR3A = ((0<<COM3A1)|(1<<COM3A0)|(0<<COM3B1)|(0<<COM3B0)|(0<<COM3C1)|(0<<COM3C0)|(0<<WGM31)|(0<<WGM30));
	TCCR3B = ((0<<CS32)|(1<<CS31)|(0<<CS30)|(1<<WGM33)|(1<<WGM32) );
	TCNT3H = 0;
	TCNT3L = 0;
	setICR3(0);
}
void setOCR3A(int num)
{
	OCR3AH = (unsigned char)(num>>8);
	OCR3AL = (unsigned char)(num&0xff);
	
}

void setOCR3B(int num)
{
	OCR3BH = (unsigned char)(num>>8);
	OCR3BL = (unsigned char)(num&0xff);
	
}

void setOCR3C(int num)
{
	OCR3CH = (unsigned char)(num>>8);
	OCR3CL = (unsigned char)(num&0xff);
	
}
void setICR3(int num)
{
	//high write first
	/**8MHz의 경우*/
	//num=(int)(num*0.5);
	
	ICR3H = (unsigned char)(num>>8);
	ICR3L = (unsigned char)(num&0xff);
	

}