/*
 * uart_lib.h
 *
 * Created: 2020-07-16 오전 2:27:50
 *  Author: 최희우
 */ 

#ifndef UART_LIB_H
#define UART_LIB_H

#define F_CPU 16000000UL
#define MYUBRR(BAUD) (F_CPU/16/(BAUD)-1)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//채널 선택.
#define uart(N) uart##N

#define BAUD_4800 4800
#define BAUD_9600 9600
#define BAUD_14400 14400
#define BAUD_19200 19200
#define BAUD_38400 38400
#define BAUD_57600 57600
#define BAUD_115200 115200
#define BAUD_230400 230400
#define BAUD_460800 460800
#define BAUD_921600 921600

struct{
	char itos_buff[64];
	char esp_buff[64];
}buff;

struct{
	
	volatile uint8_t tx0Buf[100];
	volatile uint8_t tx0Cnt,tx0CntMax;
	volatile uint8_t buf;
	//char s[20];
}uart0;

struct{
	
	volatile uint8_t tx1Buf[100];
	volatile uint8_t tx1Cnt,tx1CntMax;
	//char s[20];
}uart1;


extern void uart_init(int select_ch,unsigned int ubrr);
extern void uart0_tx_char(unsigned char data);
extern void uart0_tx_string(char * data);

extern void uart1_tx_char(unsigned char data);
extern void uart1_tx_string(char * data);


//this function have to enable global interrupt. and called ISR.
//해당 함수는 긴 문장을 송신할 경우, 딜레이가 없으면 중간에 다른 처리를 해버림.
extern void uart0_tx_string_IT(char * data);
extern void uart1_tx_string_IT(char * data);


extern char * IntToString(int num);
extern char * HexToString(char num);
extern void _uart_tx_string_in_isr(void);

extern char* connect_to_AP(char * cmd, char * ssid, char*password);
extern char* TCP_connect(char * ip, char* port);

#endif