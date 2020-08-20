/*
 * spi_lib.h
 *
 * Created: 2020-07-16 오전 2:28:16
 *  Author: 최희우
 */ 

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
void spi_init(unsigned char spi_mode, ...);
void spi_master_tx(unsigned char data);

unsigned char spi_master_rx(void);
