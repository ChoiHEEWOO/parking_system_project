/*
 * parking_system_project_0.1v.c
 *
 * Created: 2020-08-18 오후 1:13:40
 * Author : CHOI
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include "spi_lib.h"
#include "uart_lib.h"
#include "rc522.h"
#include "buzzer_cmd.h"
#include "timer_lib.h"

#include <avr/interrupt.h>
#include <util/delay.h>

#define SSID "ggsg-2"
#define PASSWORD "0318565300"


typedef uint32_t u32;

typedef uint32_t u8;

 struct{
	 volatile u32 tick_1ms;
	 volatile u32 buzz_1ms;
 }TICK;
 
 volatile u8 music_flag=0; //volatile 붙여줘야 되나? 모르겠음.
 
 
 //RFID 관련.
unsigned char byte;
unsigned char detected_flag=0;
uint8_t str[MAX_LEN];

void mfrc_print_serial(int _type);
 //===========================RFID 데이터 형식 출력==========================//
 #define ASCII_TYPE 0
 #define DECIMAL_TYPE 1
 #define HEXDECIMAL_TYPE 2
 //===========================RFID 데이터 형식 출력==========================//
char mfrc_serial_data_receive(void);
 //===========================수신 여부 리턴==========================//
 #define RECEIVE_NONE 0
 #define RECEIVE_SUCC 1
 #define RECEIVE_FAIL -1
 //===========================수신 여부 리턴==========================//



//about buzzer
void setSoundClip(char clip);
void buzz_play();
void buzz_MUTE();
void setSoundNote(int note);

ISR(TIMER0_COMP_vect) // 1khz 속도로 ISR 진입
{
	//dummy code to check 
	PORTA ^=0x02;
// 	static u32 ticks=0;
// 	ticks++;
// 	if(ticks%10==0){//1khz마다 증가
	TICK.buzz_1ms++;
	buzz_play();
}

ISR(USART1_RX_vect)
{//esp8266으로 부터 받아오는 데이터
	u8 buff=UDR1;
	
	//바로 터미널창에서 확인시도.
	uart0_tx_char(buff);
	
}
int main(void)
{
    /* Replace with your application code */
	sei();
	DDRA|=0x03;
	
	
	//사용하는 기능들 초기화 작업
	spi_init(_SPI_MASTER_MODE,_SPI_CLK_PRESC_16,_SPI_CLK_LO_LEADING);
	//spi_master_tx(0x67);
	mfrc522_init();
	uart_init(0,9600);
	uart_init(1,9600);//esp8266() : Rx:PD2, Tx:PD3
	
	mfrc522_version_check();
	//mfrc522_IRQ_enable();
	
	byte=mfrc522_read(ComIEnReg);
	mfrc522_write(ComIEnReg,byte|0x20); //RxInterrupt Enable
	byte=mfrc522_read(DivIEnReg);
	mfrc522_write(DivIEnReg,byte|0x80); //IRQPushPull
	cli();
	setSoundClip(BUZZ_SUCCESS);
	timer0_init();
	timer3_init();
	sei();
	_delay_ms(1000);
	
// 	while(TICK.buzz_1ms<1500)//전원 준비 끝날 때 까지
// 	{//ISR에 buzz_play함수를 넣어주면 이게 필요할까 싶다?
// 		//buzz_play();
// 		//전원 켜지는 소리
// 	}
	
	
	//main loop start.
    while (1) 
    {//절대 루프 안에 delay가 길게 걸리면 않도록 주의해야 함.
		//_delay_ms(20);
		//_delay_ms(1000);
		
		setSoundClip(BUZZ_FAIL);
		_delay_ms(1000);
		PORTA^=0x01;
		uart0_tx_string(send_SSID_TEST(SSID,PASSWORD)); //31ms나 소요됨.
		PORTA^=0x01;
		if(mfrc_serial_data_receive()==RECEIVE_SUCC); //받은 데이터 처리 루틴
		
	// buzz_play();이 함수가 타이밍에 영향을 크게 안준다면, ISR에 넣는것도 고려해봄.	
	//	
    }
}

char mfrc_serial_data_receive(void){
	
	byte = mfrc522_request(PICC_REQALL,str);//
	//uart0_tx_string(IntToString(byte));
	
	if(byte==CARD_FOUND&&detected_flag==0){//&& detected_flag==0
		//카드 인식이 된 경우.
		detected_flag=1;
		byte=mfrc522_get_card_serial(str);
		
		if(byte==CARD_FOUND){
			setSoundClip(BUZZ_SUCCESS);
			//uart0_tx_string_IT("\nuid: ");
			uart0_tx_string("[CHECK UID]: ");
			//_delay_ms(20);
			mfrc_print_serial(ASCII_TYPE);
			mfrc_print_serial(DECIMAL_TYPE);
			mfrc_print_serial(HEXDECIMAL_TYPE);
			uart0_tx_char('\n');
			return RECEIVE_SUCC;
		}
		else {
			setSoundClip(BUZZ_FAIL);
			uart0_tx_string("\nerror\n");
			//uart0_tx_string_IT("error\n");
			return RECEIVE_FAIL;
		}
	}
	else
	{
		detected_flag=0;
		return RECEIVE_NONE;
	}
}

void mfrc_print_serial(int _type)
{
	switch(_type)
	{
		case ASCII_TYPE:
		
		uart0_tx_string("\n\tascii: ");
		for(int i=0;i<4;i++){
			//uart0_tx_string_IT(IntToString(str[i]));
			uart0_tx_char(str[i]);
			//_delay_ms(10);
		}
		
		break;
		case DECIMAL_TYPE:
		
		uart0_tx_string("\n\tdec: ");
		for(int i=0;i<4;i++){
			//uart0_tx_string_IT(IntToString(str[i]));
			//uart0_tx_char(str[i]);
			uart0_tx_string(IntToString(str[i]));
			//_delay_ms(10);
		}
		
		
		break;
		case HEXDECIMAL_TYPE:
		
		uart0_tx_string("\n\thex: ");
		for(int i=0;i<4;i++){
			//uart0_tx_string_IT(IntToString(str[i]));
			//uart0_tx_char(str[i]);
			uart0_tx_string(HexToString(str[i]));
			//_delay_ms(10);
		}
		
		break;
		
	}
	
}

void setSoundClip(char clip){
	 // 부저 관련 tick.clear
	 switch(clip)
	 {
		 
		   case BUZZ_MUTE: music_flag=BUZZ_MUTE; break;
		   case BUZZ_SUCCESS: music_flag=BUZZ_SUCCESS; break;
		   case BUZZ_FAIL: music_flag=BUZZ_FAIL; break;
		   
// 		 case BUZZ_MUTE: music_flag=BUZZ_MUTE; break;
// 		 case BUZZ_BEEP: music_flag=BUZZ_BEEP; break;
// 		 case BUZZ_FAIL: music_flag=BUZZ_FAIL; break;
// 		 case BUZZ_POWERON: music_flag=BUZZ_POWERON; break;
// 		 case BUZZ_DOOR_OPEN: music_flag=BUZZ_DOOR_OPEN; break;
	 }
	 TICK.buzz_1ms=0;
}


void buzz_play(){
	 //재생이 끝났으면 music_flag는 확실하게 MUTE로 들어가야 함. 안그러면 꼬이는 것 같다.
	  switch(music_flag)
	  {
		  case BUZZ_MUTE:  buzz_MUTE(); break; //setICR3(0);. buzz_MUTE() 안에 music_flag=MUTE 넣어주는 명령 들어있음.
		  
		  case BUZZ_SUCCESS:
		  //TCCR3A |= (1<<COM3A0); //재생 시 타이머카운터 3번 채널 A채널 고유 핀 토글모드로 출력 설정.
		  if(TICK.buzz_1ms<200)setSoundNote(Ca);
		  else if(TICK.buzz_1ms==210) setSoundNote(BUZZ_MUTE);
		  else if(TICK.buzz_1ms==220) setSoundNote(E);
		  else if(TICK.buzz_1ms==400) setSoundNote(BUZZ_MUTE);
		  else if(TICK.buzz_1ms==430) setSoundNote(A);
		  else if(TICK.buzz_1ms==600) buzz_MUTE();
		  break;
		  
		  case BUZZ_FAIL:
		  if(TICK.buzz_1ms<100)setSoundNote(_960Hz);
		  else if(TICK.buzz_1ms<200)setSoundNote(BUZZ_MUTE);
		  else if(TICK.buzz_1ms<300)setSoundNote(_960Hz);
		  else if(TICK.buzz_1ms==450) buzz_MUTE(); //buzz_MUTE();
		  break;
	  }

}
void buzz_MUTE(){
	 TCCR3A &= ~(1<<COM3A0); // 타이머카운터3번 A채널 고유 핀 출력 X
	 music_flag = BUZZ_MUTE;
	 /*setSoundClip(BUZZ_MUTE);*/
}
void setSoundNote(int note){
	 if(BUZZ_MUTE!=note){
		 TCCR3A |= (1<<COM3A0);setICR3(note);
	 }
	 else {TCCR3A &= ~(1<<COM3A0);}
	 
}