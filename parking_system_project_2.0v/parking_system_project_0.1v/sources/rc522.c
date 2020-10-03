/*
 * rc522.c
 *
 * Created: 2020-07-19 오후 7:07:23
 *  Author: 최희우
 */ 
#include "rc522.h"
#include "spi_lib.h"
#include "uart_lib.h"
#include "clcd_i2c.h"
#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>




static void mfrc522_write(unsigned char reg, unsigned char value, unsigned char channel)
{
	
	//spi 통신 시작 (굳이 없어도 될듯)
	//RC522와 연결된 SS핀 LOW출력
	SS_PORT &= ~(1<<(channel));
	//SPI 라인을 통해 regisger 주소 전송 (MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.)
	spi_master_tx((reg<<1)&0x7E);
	//이후 레지스터 내부 값 전송
	spi_master_tx(value);
	//RC522와 연결된 SS핀 HIGH출력
	SS_PORT |= (1<<channel);
	//spi 통신 종료	 (굳이 없어도 될듯)
	
	
}

static void mfrc522_reset(unsigned char ch){ //장치의 데이터를 모두 리셋시킴.
	//내부 버퍼는 유지되며, 레지스터의 데이터들은 reset values로 set된다.
	
	mfrc522_write(CommandReg,MFRC522_CMD_SoftReset,ch); // 0x01<<1 , 0x0F

}

static unsigned char mfrc522_read(unsigned char reg, unsigned char channel){
	unsigned char value=0;
	//RC522와 연결된 SS핀 LOW출력
	SS_PORT &= ~(1<<channel);
	//SPI 라인을 통해 regisger 주소 전송 ( MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.)
	spi_master_tx((((reg<<1)&0x7E)|0x80));
	value = spi_master_rx(); //send dummy data and receive data.
	//RC522와 연결된 SS핀 HIGH출력
	SS_PORT |= (1<<channel);
	
	return value;
}

static unsigned char mfrc522_to_card(unsigned char cmd, unsigned char *send_data, unsigned char send_data_len, unsigned char *back_data, uint32_t *back_data_len, unsigned char ch)
{
	uint8_t status = ERROR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;
	uint8_t n;
	uint8_t	tmp;
	uint32_t i;

	switch (cmd)
	{
		case MFRC522_CMD_MFAuthent:		//Certification cards close
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case MFRC522_CMD_Transceive:	//Transmit FIFO data
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
		break;
	}
	
	///////////////////////////////////////////============여기-==========//////////
	mfrc522_write(ComIEnReg, irqEn|0x80,ch);	//Interrupt request 원래 주석으로 달려있었떤 곳임.
	n=mfrc522_read(ComIrqReg,ch);
	mfrc522_write(ComIrqReg,n&(~0x80),ch);//clear all interrupt bits
	n=mfrc522_read(FIFOLevelReg,ch);
	mfrc522_write(FIFOLevelReg,n|0x80,ch);//flush FIFO data
	
	mfrc522_write(CommandReg, MFRC522_CMD_Idle,ch);	//NO action; Cancel the current cmd???

	//Writing data to the FIFO
	for (i=0; i<send_data_len; i++)
	{
		mfrc522_write(FIFODataReg, send_data[i],ch);
	}

	//Execute the cmd
	mfrc522_write(CommandReg, cmd,ch);
	if (cmd == MFRC522_CMD_Transceive)
	{
		n=mfrc522_read(BitFramingReg,ch);
		mfrc522_write(BitFramingReg,n|0x80,ch);
	}
	
	//Waiting to receive data to complete
	i = 2000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
	do
	{
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = mfrc522_read(ComIrqReg,ch);
		i--;
	}
	while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	tmp=mfrc522_read(BitFramingReg,ch);
	mfrc522_write(BitFramingReg,tmp&(~0x80),ch);
	
	if (i != 0)
	{
		if(!(mfrc522_read(ErrorReg,ch) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
		{
			status = CARD_FOUND;
			if (n & irqEn & 0x01)
			{
				status = CARD_NOT_FOUND;			//??
			}

			if (cmd == MFRC522_CMD_Transceive)
			{
				n = mfrc522_read(FIFOLevelReg,ch);
				lastBits = mfrc522_read(ControlReg,ch) & 0x07;
				if (lastBits)
				{
					*back_data_len = (n-1)*8 + lastBits;
				}
				else
				{
					*back_data_len = n*8;
				}

				if (n == 0)
				{
					n = 1;
				}
				if (n > MAX_LEN)
				{
					n = MAX_LEN;
				}
				
				//Reading the received data in FIFO
				for (i=0; i<n; i++)
				{
					back_data[i] = mfrc522_read(FIFODataReg,ch);
				}
			}
		}
		else
		{
			status = ERROR;
		}
		
	}
	
	// 	SetBitMask(ControlReg,0x80);           //timer stops
	//mfrc522_write(CommandReg, MFRC522_CMD_Idle);

	return status;
}

void mfrc522_init(unsigned char ch)
{
	unsigned char byte;
	
	spi_init(_SPI_MASTER_MODE,_SPI_CLK_PRESC_16,_SPI_CLK_LO_LEADING);
	
	//SS핀 출력모드 및 디폴트 하이로 설정해두어야 함.
	SS_DDR|=(0x03); // C포트 0번핀을 SS핀으로 사용
	SS_PORT=0; //입력핀이 전원에 영향 끼치는 행위를 막기 위함
	//이렇게 케어해두니, 이상 동작하지 않아서 좋음
	_delay_ms(20);
	SS_PORT|=(0x03); //Default HIGH
	mfrc522_reset(ch);
	
	
	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	// 통신을 할 때, 잘못되는 상황을 대비하여 timeout을 걸어놓아야 한다.
	mfrc522_write(TModeReg, 0x8D,ch); //Timer setting and prescalingH
	mfrc522_write(TPrescalerReg, 0x3E,ch); //prescalingL  ==> pre:0xD3E=3390
	mfrc522_write(TReloadRegH, 30,ch); //16-bit timer reload value high.
	mfrc522_write(TReloadRegL, 0,ch); //16-bit timer reload value low. ==>Reload Timer :0x1E0=480  ==> (ie 120ms before timeout.
	
	
	mfrc522_write(TxASKReg, 0x40,ch); //Controls transmit modulation settings. (Force100ASK)
	mfrc522_write(ModeReg, 0x3D,ch); //Defines general mode settings for transmitting and receiving.
	
	
	//AntennaOn. reset 시 해당 핀은 다시 disable된다.
	byte = mfrc522_read(TxControlReg,ch);
	if(!(byte&0x03)) //0번비트 1번비트 둘다 모두 0으로 클리어 되어있는 경우
	{
		mfrc522_write(TxControlReg,byte|0x03,ch);
	}
}




void mfrc522_version_check(unsigned char ch){
	
	unsigned char byte;
	
	byte = mfrc522_read(VersionReg, ch);
	//uart0_tx_string_IT(IntToString(byte));
	_delay_ms(20); //전송시 여유가 있어야 함...
	///
	if(ch==CH0)uart0_tx_string("\n[channel 0");
	else uart0_tx_string("\n[channel 1");
	
	if(byte == 0x92)
	{
		uart0_tx_string(" Detected Version]: MIFARE RC522v2\n");
		if(ch==CH0) i2c_lcd_string(2,0,"     O_________     ");
		else i2c_lcd_string(2,0,"     OO________     ");
		
	}else if(byte == 0x91 || byte==0x90)
	{
		uart0_tx_string(" Detected Version]: MIFARE RC522v1\n");
		if(ch==CH0) i2c_lcd_string(2,0,"     O_________     ");
		else i2c_lcd_string(2,0,"     OO________     ");
	}else
	{
		uart0_tx_string(" No reader found]\n");
		i2c_lcd_string(2,0,"Error, Plz CHK RFID");
		while(1);
	}
	
}

void mfrc522_IRQ_enable(unsigned char ch)
{
	unsigned char byte;
	
	byte=mfrc522_read(ComIEnReg, ch);
	mfrc522_write(ComIEnReg,byte|0x20, ch); //RxInterrupt Enable
	byte=mfrc522_read(DivIEnReg, ch);
	mfrc522_write(DivIEnReg,byte|0x80, ch); //IRQPushPull
}


unsigned char mfrc522_request(unsigned char req_mode, unsigned char * tag_type, unsigned char ch)
{
	uint8_t  status;
	uint32_t backBits;//The received data bits

	//Adjustments for bit-oriented frames.
	mfrc522_write(BitFramingReg, 0x07, ch);//TxLastBists = BitFramingReg[2..0]	???
	
	tag_type[0] = req_mode;
	status = mfrc522_to_card(MFRC522_CMD_Transceive, tag_type, 1, tag_type, &backBits, ch);

	if ((status != CARD_FOUND) || (backBits != 0x10))
	{
		status = ERROR;
	}
	
	return status;
}



unsigned char mfrc522_get_card_serial(unsigned char * serial_out, unsigned char ch)
{
	uint8_t status;
	uint8_t i;
	uint8_t serNumCheck=0;
	uint32_t unLen;
	
	mfrc522_write(BitFramingReg, 0x00,ch);		//TxLastBists = BitFramingReg[2..0]
	
	serial_out[0] = PICC_ANTICOLL;
	serial_out[1] = 0x20;
	status = mfrc522_to_card(MFRC522_CMD_Transceive, serial_out, 2, serial_out, &unLen,ch);

	if (status == CARD_FOUND)
	{
		//Check card serial number
		for (i=0; i<4; i++)
		{
			serNumCheck ^= serial_out[i];
		}
		if (serNumCheck != serial_out[i])
		{
			status = ERROR;
		}
	}
	return status;
}

