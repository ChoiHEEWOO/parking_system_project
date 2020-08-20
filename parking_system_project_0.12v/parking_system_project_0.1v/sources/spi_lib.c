/*
 * spi.c
 *
 * Created: 2020-07-16 오전 2:27:59
 *  Author: 최희우
 */ 
#include "spi_lib.h"
#include <stdarg.h>
#include <avr/io.h> //소스파일을 분리했을 때 마찬가지로 해당 파일에서 사용하는 라이브러리는 가져와야 함.
#include <avr/interrupt.h>

/* SPCR Register 
	(Check atmel atmega128 Datasheet p166 SPCR 레지스터)
		대충 SPI 초기 설정해주는 레지스터
		SPIE(bit-7): SPI Interrupt Enable : 외부 인터럽트를  
		SPE (bit-6): SPI Enable :
		DORD(bit-5): Data Order :
		MSTR(bit-4): Master/Slave Select :
		CPOL(bit-3): Clock Polarity
		CPHA(bit-2): Clock Phase
		SPR1(bit-1): SPI Clock rate 1 
		SPR0(bit-0): SPI Clock rate 0
*/

/* SPSR Register
	(Check atmel atmega128 Datasheet p168 SPSR 레지스터)
		대충 SPI 통신중, 상태에 대해 알려주는 레지스터
		SPIF (bit-7): SPI Interrupt Flag   : 시리얼 전송이 완료됐을 때, 해당 비트가 set된다. 자세한 설명은 데이터시트에서 확인.
		WCOL (bit-6): Write Collision Flag : 시리얼 데이터 전송 중에 SPDR 레지스터가 write되었을 때 set된다. 자세한 설명은 데이터시트로
		SPI2X(bit-0): Double SPI Speed bit : SPCR레지스터에서 설정했던 SPI 클럭속도의 2배로 set하고자 할때 사용하는 비트.
*/

/* SPDR Register
	데이터 Read/Write하는 레지스터임. MSB/LSB를 유념해둬야 한다.
*/


//spi intialization function. ver1.0
//Parameter :
//if) spi master mode	: (spi_mode, spi_click_prescailing, spi_clock_and_edge)
//if) spi slave  mode	: (spi_mode)
//if) spi master mode (advance) : (spi_mode, spi_click_prescailing, spi_clock_and_edge, spi_double_speed)
void spi_init(unsigned char spi_mode, ...){

	if ((spi_mode != _SPI_MASTER_MODE) && (spi_mode != _SPI_SLAVE_MODE)) return;// this line is exception code when if user insert the wrong parameter
	
	unsigned char spcr_buff;
	unsigned char param_buff[3];//[0]: 분주, [1]: 엣지검출, [2]: 주파수2배		
	
	va_list ap;
	va_start(ap, spi_mode); //spi_mode : 0 or 2
	
	//spi_mode 변수 = 표면적으론 spi 기능 선택여부, 내면적으로는 가변인자의 갯수를 정해주는 역할을 함.
	for (int i = 0; i < spi_mode; i++) {
		param_buff[i]=(unsigned char)va_arg(ap, int);
	} 
	if(param_buff[2]!=0&&param_buff[2]!=1) param_buff[2]=0;//exception care code about trash value at buffer
	
	
	SPCR=0; //clear CPCR register
	
	if(spi_mode==_SPI_MASTER_MODE){
		
		/* Set MOSI and SCK output, all others input */
		SPI_PORT |= (1 << SS);
		SPI_DDR |= ((1<<MOSI)|(1<<SCK)|(1 << SS));
		
		spcr_buff=((1<<SPE)|(1<<MSTR)|(param_buff[0])|(param_buff[1]));
		SPSR|=(param_buff[2]<<0);//double speed set
		SPCR=spcr_buff;
 
	}
	else if(spi_mode==_SPI_SLAVE_MODE){
		/* Set MISO output, all others input */
		SPI_DDR |= (1<<MISO);
		/* Enable SPI */
		SPCR = (1<<SPE);
	}
	else {
		//error
		return;
	}
}


void spi_master_tx(unsigned char data){
	SPDR = data;
	while(!(SPSR&(1<<SPIF)));
}

unsigned char spi_master_rx(void){
	SPDR=0x00; //dummy code. 수신 받기 위해 더미데이터를 슬레이브로 전송시킴.
	while(!(SPSR&(1<<SPIF)));
	
	return SPDR;
}



/**이전 버전 함수 ver0.2 
//check the library(spi_lib.h). And search _SPI_...something to fill this parameter
void spi_init(unsigned char spi_mode, unsigned char spi_clk_presc, unsigned char spi_clock_and_edge){
	
	unsigned char spcr_buff;
	SPCR=0; //clear this register
	
	if(spi_mode==_SPI_MASTER_MODE){
		
		// Set MOSI and SCK output, all others input 
		SPI_PORT |= (1 << SS);
		SPI_DDR |= ((1<<MOSI)|(1<<SCK));
		
		spcr_buff=((1<<SPE)|(spi_mode<<MSTR)|(spi_clk_presc)|(spi_clock_and_edge));
		
		SPCR=spcr_buff;
		
	}
	else if(spi_mode==_SPI_SLAVE_MODE){
		//Set MISO output, all others input 
		SPI_DDR |= (1<<MISO);
		// Enable SPI 
		SPCR = (1<<SPE);
	}
	else {
		//error 
		return;
	}
}
*/
//======================don't use.. maybe=============//
/**이전 버전 함수 ver0.1
void SPI_Master_Init(void)
{

	SPI_PORT |= (1 << SS);
	SPI_DDR |= ((1<<MOSI)|(1<<SCK));
	//SPI Mode: master mode
	//Data order: the MSB of the data word is transmitted first.
	//polarity, phase: Leading edge=(Sample, Rising)
	//16MHz
	SPCR |= ((0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(1<<SPR0));
	SPSR |= ((0<<SPIF)|(0<<WCOL)|(0<<SPI2X));
	
}

void SPI_Slave_Init(void)
{

	SPI_DDR |= (1<<MISO);
	SPCR = (1<<SPE);
}
*/