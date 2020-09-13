/*
 * spi_lib.h
 *
 * Created: 2020-07-16 오전 2:28:16
 *  Author: 최희우
 */ 
#ifndef SPI_LIB_H
#define SPI_LIB_H

#include <stdint.h>

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define SS 0
#define SCK 1
#define MOSI 2
#define MISO 3

#define _SPI_MASTER_MODE 3
#define _SPI_SLAVE_MODE 0

#define _SPI_CLK_PRESC_4	(0b00)
#define _SPI_CLK_PRESC_16	(0b01)
#define _SPI_CLK_PRESC_64	(0b10)
#define _SPI_CLK_PRESC_128	(0b11)

#define _SPI_CLK_LO_LEADING		(0b00<<2)
#define _SPI_CLK_LO_TRAILING	(0b01<<2)
#define _SPI_CLK_HI_LEADING		(0b10<<2)
#define _SPI_CLK_HI_TRAILING	(0b11<<2)

#define _SPI_DOUBLE_SPEED 1


//spi intialization function. ver1.0
//Parameter :
//if) spi master mode	: (spi_mode, spi_click_prescailing, spi_clock_and_edge)
//if) spi slave  mode	: (spi_mode)
extern void spi_init(uint8_t spi_mode, ...);
extern void spi_master_tx(uint8_t data);

extern uint8_t spi_master_rx(void);


#endif