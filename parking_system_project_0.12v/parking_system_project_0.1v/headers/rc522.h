/*
 * rc522.h
 *
 * Created: 2020-07-19 오후 7:00:05
 *  Author: 최희우
 */ 
#include "spi_lib.h"
#include <avr/io.h>

#define SS_DDR	DDRC
#define SS_PORT PORTC 
#define CH0 0
#define CH1 1

#define		_MFRC_SPI_MSBFIRST		 1
#define		_MFRC_SPI_MODE0			 0		

#define CARD_FOUND		1
#define CARD_NOT_FOUND	2
#define ERROR			3
#define MAX_LEN			16

//Card types (don't use)
#define Mifare_UltraLight 	0x4400
#define Mifare_One_S50		0x0400
#define Mifare_One_S70		0x0200
#define Mifare_Pro_X		0x0800
#define Mifare_DESFire		0x4403

// Mifare_One card command word
# define PICC_REQIDL          0x26               // find the antenna area does not enter hibernation
# define PICC_REQALL          0x52               // find all the cards antenna area
# define PICC_ANTICOLL        0x93               // anti-collision
# define PICC_SElECTTAG       0x93               // election card
# define PICC_AUTHENT1A       0x60               // authentication key A
# define PICC_AUTHENT1B       0x61               // authentication key B
# define PICC_READ            0x30               // Read Block
# define PICC_WRITE           0xA0               // write block
# define PICC_DECREMENT       0xC0               // debit
# define PICC_INCREMENT       0xC1               // recharge
# define PICC_RESTORE         0xC2               // transfer block data to the buffer
# define PICC_TRANSFER        0xB0               // save the data in the buffer
# define PICC_HALT            0x50               // Sleep

//===========================register list===========================//  check datasheet p36 확인
#define		CommandReg				(0x01)// starts and stops command execution
#define		ComIEnReg				(0x02)	// enable and disable interrupt request control bits
#define		DivIEnReg				(0x03)	// enable and disable interrupt request control bits
#define		ComIrqReg				(0x04)	// interrupt request bits
#define		DivIrqReg				(0x05)	// interrupt request bits
#define		ErrorReg				(0x06)	// error bits showing the error status of the last command executed
#define		Status1Reg				(0x07)	// communication status bits
#define		Status2Reg				(0x08)	// receiver and transmitter status bits
#define		FIFODataReg				(0x09)	// input and output of 64 byte FIFO buffer
#define		FIFOLevelReg			(0x0A)	// number of bytes stored in the FIFO buffer
#define		WaterLevelReg			(0x0B)	// level for FIFO underflow and overflow warning
#define		ControlReg				(0x0C)	// miscellaneous control registers
#define		BitFramingReg			(0x0D)	// adjustments for bit-oriented frames
#define		CollReg					(0x0E)	// bit position of the first bit-collision detected on the RF interface
//						  0x0F			// reserved for future use
		
// Page 1: Command
// 						  0x10			// reserved for future use
#define		ModeReg					(0x11)	// defines general modes for transmitting and receiving
#define		TxModeReg				(0x12)	// defines transmission data rate and framing
#define		RxModeReg				(0x13)	// defines reception data rate and framing
#define		TxControlReg			(0x14)	// controls the logical behavior of the antenna driver pins TX1 and TX2
#define		TxASKReg				(0x15)	// controls the setting of the transmission modulation
#define		TxSelReg				(0x16)	// selects the internal sources for the antenna driver
#define		RxSelReg				(0x17)	// selects internal receiver settings
#define		RxThresholdReg			(0x18)	// selects thresholds for the bit decoder
#define		DemodReg				(0x19)	// defines demodulator settings
// 						  0x1A			// reserved for future use
// 						  0x1B			// reserved for future use
#define		MfTxReg					(0x1C)	// controls some MIFARE communication transmit parameters
#define		MfRxReg					(0x1D)	// controls some MIFARE communication receive parameters
// 						  0x1E			// reserved for future use
#define		SerialSpeedReg			(0x1F)	// selects the speed of the serial UART interface	
// Page 2: Configuration
// 						  0x20			// reserved for future use
#define		CRCResultRegH			(0x21)	// shows the MSB and LSB values of the CRC calculation
#define		CRCResultRegL			(0x22)
// 						  0x23			// reserved for future use
#define		ModWidthReg				(0x24)	// controls the ModWidth setting?
// 						  0x25			// reserved for future use
#define		RFCfgReg				(0x26)	// configures the receiver gain
#define		GsNReg					(0x27)	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
#define		CWGsPReg				(0x28)	// defines the conductance of the p-driver output during periods of no modulation
#define		TModeReg				(0x2A)	// defines settings for the internal timer
#define		TPrescalerReg			(0x2B)	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
#define		TReloadRegH				(0x2C)	// defines the 16-bit timer reload value
#define		TReloadRegL				(0x2D)
#define		TCounterValueRegH		(0x2E)	// shows the 16-bit timer value
#define		TCounterValueRegL		(0x2F)
// Page 3: Test Registers
// 						  0x30			// reserved for future use
#define		TestSel1Reg				(0x31)	// general test signal configuration
#define		TestSel2Reg				(0x32)	// general test signal configuration
#define		TestPinEnReg			(0x33)	// enables pin output driver on pins D1 to D7
#define		TestPinValueReg			(0x34)	// defines the values for D1 to D7 when it is used as an I/O bus
#define		TestBusReg				(0x35)	// shows the status of the internal test bus
#define		AutoTestReg				(0x36)	// controls the digital self-test
//#define		VersionReg				(0x37 << 1)	// shows the software version
#define		VersionReg				(0x37)	// shows the software version
#define		AnalogTestReg			(0x38)	// controls the pins AUX1 and AUX2
#define		TestDAC1Reg				(0x39)	// defines the test value for TestDAC1
#define		TestDAC2Reg				(0x3A)	// defines the test value for TestDAC2
#define		TestADCReg				(0x3B)		// shows the value of ADC I and Q channels
//===========================register list===========================//

//===========================command list===========================// check datasheet p70 확인
#define 	MFRC522_CMD_Idle				 0x00		// no action, cancels current command execution
#define		MFRC522_CMD_Mem					 0x01		// stores 25 bytes into the internal buffer
#define		MFRC522_CMD_GenerateRandomID	 0x02		// generates a 10-byte random ID number
#define		MFRC522_CMD_CalcCRC				 0x03		// activates the CRC coprocessor or performs a self-test
#define		MFRC522_CMD_Transmit			 0x04		// transmits data from the FIFO buffer
#define		MFRC522_CMD_NoCmdChange			 0x07		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define		MFRC522_CMD_Receive				 0x08		// activates the receiver circuits
#define		MFRC522_CMD_Transceive 			 0x0C		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
#define		MFRC522_CMD_MFAuthent 			 0x0E		// performs the MIFARE standard authentication as a reader
#define		MFRC522_CMD_SoftReset			 0x0F		// resets the MFRC522

//===========================command list===========================//





void mfrc522_init(unsigned char ch);
void mfrc522_write(unsigned char reg, unsigned char value,unsigned char channel);
unsigned char mfrc522_read(unsigned char reg, unsigned char channel);
void mfrc522_reset(unsigned char ch);
void mfrc522_version_check(unsigned char ch);
void mfrc522_IRQ_enable(unsigned char ch);
unsigned char mfrc522_request(unsigned char req_mode, unsigned char * tag_type,unsigned char ch);
unsigned char mfrc522_to_card(unsigned char cmd, unsigned char *send_data, unsigned char send_data_len, unsigned char *back_data, uint32_t *back_data_len,unsigned char ch);
unsigned char mfrc522_get_card_serial(unsigned char * serial_out,unsigned char ch);
