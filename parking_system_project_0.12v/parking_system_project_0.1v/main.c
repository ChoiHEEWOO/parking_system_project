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
 #define BUMMY_TEST_SERIAL 1  //if 1: dummy test, 0: no test at terminal
unsigned char byte;
unsigned char detected_flag_ch0='X';
unsigned char detected_flag_ch1='X';
uint8_t rfid_uid_ch0[MAX_LEN];
uint8_t rfid_uid_ch1[MAX_LEN];
char received_state;
uint8_t esp8266_received_data[50];
 //===========================RFID 입력 flag==========================//
 #define DETECED 'O'
 #define NON_DETECTED 'X'

 //===========================RFID 입력 flag==========================//

 //===========================RFID 입/출구==========================//
#define ENTRANCE_GATE 0
#define EXIT_GATE 1
 //===========================RFID 입/출구==========================//




void mfrc_print_serial(int _type, unsigned char ch);
 //===========================RFID 데이터 형식 출력==========================//
 #define ASCII_TYPE 0
 #define DECIMAL_TYPE 1
 #define HEXDECIMAL_TYPE 2
 //===========================RFID 데이터 형식 출력==========================//
char mfrc_check_and_data_receive_ch0(void); //여기선 카드 인식 자체의 성공 실패 여부만 체크.
char mfrc_check_and_data_receive_ch1(void); //여기선 카드 인식 자체의 성공 실패 여부만 체크.
//esp8266에는 성공 후 무조건 단 한번만 넘기기만 하면 됨. 실패하면 안넘기면 되는거고

 //===========================수신 여부 리턴==========================//
 #define RECEIVE_NONE 0
 #define RECEIVE_SUCCESS 1
 #define RECEIVE_FAIL -1
 //===========================수신 여부 리턴==========================//
//about buzzer
void setSoundClip(char clip);
void buzz_play();
void buzz_MUTE();
void setSoundNote(int note);

void esp8266_init(unsigned char* ssid, unsigned char* pw, unsigned char * ip, unsigned char* port);

ISR(TIMER0_COMP_vect) // 1khz 속도로 ISR 진입
{
	//dummy code to check 
	//PORTA ^=0x02;
	
// 	static u32 ticks=0;
// 	ticks++;
// 	if(ticks%10==0){//0.1khz마다 증가
	TICK.buzz_1ms++;
	TICK.tick_1ms++;
	buzz_play(); //
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
	DDRA|=0x03; //test Port
	
	
	//사용하는 기능들 초기화 작업
	spi_init(_SPI_MASTER_MODE,_SPI_CLK_PRESC_16,_SPI_CLK_LO_LEADING);
	//spi_master_tx(0x67);
	mfrc522_init(CH0);
	_delay_ms(100);
	mfrc522_init(CH1);
	uart_init(0,9600);
	uart_init(1,9600);//esp8266() : Rx:PD2, Tx:PD3
	
	_delay_ms(300);
	mfrc522_version_check(CH0);
	mfrc522_IRQ_enable(CH0);
	_delay_ms(300);
	mfrc522_version_check(CH1);
	mfrc522_IRQ_enable(CH1);

// 	byte=mfrc522_read(ComIEnReg);
// 	mfrc522_write(ComIEnReg,byte|0x20); //RxInterrupt Enable
// 	byte=mfrc522_read(DivIEnReg);
// 	mfrc522_write(DivIEnReg,byte|0x80); //IRQPushPull
	cli();
	setSoundClip(BUZZ_ON);
	timer0_init();
	timer3_init();
	sei();
	 TICK.tick_1ms=0;
	_delay_ms(2500);
	uart0_tx_string("AT\r\n");
	

	//main loop start.
    while (1) 
    {//절대 루프 안에 delay가 길게 걸리면 않도록 주의해야 함.
		//PORTA^=0x01;
		
		//_delay_ms(100);
		//uart0_tx_string(send_SSID_TEST(SSID,PASSWORD)); //31ms나 소요됨.
		//PORTA^=0x01;
		static char toggle=0;
		if((TICK.tick_1ms % 100) ==0) {
			//toggle^=0x01; //start toggling :
			//toggle = 0 : entrance gate 
			//toggle = 1 : exit gate
			if(toggle==0)received_state = mfrc_check_and_data_receive_ch0(); //RFID check and receive UID data per 100ms
			else received_state = mfrc_check_and_data_receive_ch1();  //UID values are in 'rfid_uid_chX[]'
		}
		if(received_state==RECEIVE_NONE); //do nothing
		else if(received_state==RECEIVE_SUCCESS){//Received data service routine.
			//send to esp8266 and receive result data.
			//esp8266으로 uid값과 출/입 여부를 전송
			if(toggle==ENTRANCE_GATE){
				//esp8266에 uid와 입구게이트 정보 전송 함수
				//while(전송 완료될 때 까지 대기)???
				uart1_tx_string("AT+CIPSEND=4\r\n"); //4byte길이 데이터 전송 예정
				// '>' 문자가 확인될 때까지 대기
				//rx 완료 대기 관련 레지스터 찾아봐야함.
				for(int i=0; i<4; i++)
				{
					uart1_tx_string(HexToString(rfid_uid_ch0[i]));
				}
				uart1_tx_string("\r\n");
				
				
				
				
				
				//전송 후 
				
				strcpy(esp8266_received_data,"SUCCESS,CHOI HEE WOO"); //결과 데이터 저장.
				//LCD 뷰어 및 5초 카운트 후 다시 리셋
			}
			else if(toggle==EXIT_GATE){
				//esp8266에 uid와 출구게이트 정보 전송 함수		
				//구현안하기로 함			
			}
					
			/*
			if(esp수신데이터)
			else if(esp 수신 데이터)
			*/
			setSoundClip(BUZZ_SUCCESS);
		}
		else if(received_state==RECEIVE_FAIL){
			setSoundClip(BUZZ_FAIL);
		}
		received_state=RECEIVE_NONE;
		//dummy code
		//else if(received_state==RECEIVE_FAIL); 
		
	// buzz_play();이 함수가 타이밍에 영향을 크게 안준다면, ISR에 넣는것도 고려해봄.	
	//	
    }
}

char mfrc_check_and_data_receive_ch0(void){ 
	//하... 이 복병을 해결하는 방법은 detect_flag를 다른 곳에서 돌아오도록 처리해주는 방법밖에 안떠오른다. 기모띵 
	
	//원인 모를 버그를 해결하기 위한 용도로 쓰는 flag : 카드 인식 request 시, return 할 때 oxoxoxoxox이짓거리 하는 버그 발생	
	static char noise_flag=0;
	static char toggle_flag=0;
	static char _byte=0;
	if(noise_flag==0){ //CARD_FOUND로 리턴될 떄
		_byte = mfrc522_request(PICC_REQALL,rfid_uid_ch0,CH0);
	}
	else { //인식 성공 이후 인식(ERROR로 리턴될 때) 
		
		
		if(toggle_flag) mfrc522_request(PICC_REQALL,rfid_uid_ch0,CH0); //이상한 데이터 가져올 때
		else{ //정상적인 데이터 가져올 때
			_byte=mfrc522_request(PICC_REQALL,rfid_uid_ch0,CH0);
			if(_byte==ERROR) noise_flag=0;
		}
			
		toggle_flag^=0x01;
	}
	
	/*dummy code///////////////////////////////////////*/
// 	if(byte==CARD_FOUND)uart0_tx_char('O');
// 	else if(byte==CARD_NOT_FOUND)uart0_tx_char('N');
// 	else if(byte==ERROR)uart0_tx_char('X');
	///////////////////////////////////////////////////
	
	
	if(_byte!=CARD_FOUND) //카드 인식이 안되어 있는 경우
	{
		detected_flag_ch0=NON_DETECTED;  
		return RECEIVE_NONE;
	}
	else if((_byte==CARD_FOUND)&&(detected_flag_ch0==NON_DETECTED)) //카드를 계속 대고 있다면, 첫 순간만 인정
	{
		detected_flag_ch0=DETECED;
		noise_flag=1; //얘가 첫 순간임.
		toggle_flag=1;
		_byte=mfrc522_get_card_serial(rfid_uid_ch0,CH0);
		if(_byte==CARD_FOUND){//카드가 인식됐을 때 
			
		/*dummy code///////////////////////////////////////*/
// 		if(byte==CARD_FOUND)uart0_tx_char('O');
// 		else if(byte==CARD_NOT_FOUND)uart0_tx_char('N');
// 		else if(byte==ERROR)uart0_tx_char('X');
	///////////////////////////////////////////////////

			//
			//dummy code
			//setSoundClip(BUZZ_SUCCESS);
			if(BUMMY_TEST_SERIAL){
				uart0_tx_string("[CHECK UID(CH0)]: ");
				mfrc_print_serial(ASCII_TYPE,CH0);
				mfrc_print_serial(DECIMAL_TYPE,CH0);
				mfrc_print_serial(HEXDECIMAL_TYPE,CH0);
				uart0_tx_char('\n');
			}
			//////////////////////////
			
			return RECEIVE_SUCCESS;
		}
		else {//카드는 인식됐지만 식별되지 않았을 때 
			//dummy code////////////////
			//uart0_tx_string("\nerror\n");
			////////////////////////////
			
			return RECEIVE_FAIL;
		}
	}
	else {  //카드를 계속 대고 있을 때 (byte==CARD_FOUND && detected_flag==1)
		
		return RECEIVE_NONE;
	}
	

}

char mfrc_check_and_data_receive_ch1(void){ 
	static char noise_flag=0;
	static char toggle_flag=0;
	static char _byte=0;
	if(noise_flag==0){ //CARD_FOUND로 리턴될 떄
		_byte = mfrc522_request(PICC_REQALL,rfid_uid_ch1,CH1);
	}
	else { //인식 성공 이후 인식(ERROR로 리턴될 때) 
		if(toggle_flag) mfrc522_request(PICC_REQALL,rfid_uid_ch1,CH1); //이상한 데이터 가져올 때
		else{ //정상적인 데이터 가져올 때
			_byte=mfrc522_request(PICC_REQALL,rfid_uid_ch1,CH1);
			if(_byte==ERROR) noise_flag=0;
		}
		toggle_flag^=0x01;
	}
	if(_byte!=CARD_FOUND) //카드 인식이 안되어 있는 경우
	{
		detected_flag_ch1=NON_DETECTED;  
		return RECEIVE_NONE;
	}
	else if((_byte==CARD_FOUND)&&(detected_flag_ch1==NON_DETECTED)) //카드를 계속 대고 있다면, 첫 순간만 인정
	{
		detected_flag_ch1=DETECED;
		noise_flag=1; //얘가 첫 순간임.
		toggle_flag=1;
		_byte=mfrc522_get_card_serial(rfid_uid_ch1,CH1);
		if(_byte==CARD_FOUND){//카드가 인식됐을 때 
			if(BUMMY_TEST_SERIAL){
						uart0_tx_string("[CHECK UID(CH1)]: ");
						mfrc_print_serial(ASCII_TYPE,CH1);
						mfrc_print_serial(DECIMAL_TYPE,CH1);
						mfrc_print_serial(HEXDECIMAL_TYPE,CH1);
						uart0_tx_char('\n');
			}
			return RECEIVE_SUCCESS;
		}
		else {//카드는 인식됐지만 식별되지 않았을 때 
				return RECEIVE_FAIL;
		}
	}
	else {  //카드를 계속 대고 있을 때 (byte==CARD_FOUND && detected_flag==1)
		return RECEIVE_NONE;
	}
	

}

void mfrc_print_serial(int _type, unsigned char ch)
{
	switch(_type)
	{
		case ASCII_TYPE:
		
		uart0_tx_string("\n\tascii: ");
		for(int i=0;i<4;i++){
			//uart0_tx_string_IT(IntToString(str[i]));
			if(ch==CH0)uart0_tx_char(rfid_uid_ch0[i]);
			else uart0_tx_char(rfid_uid_ch1[i]);
			//_delay_ms(10);
		}
		
		break;
		case DECIMAL_TYPE:
		
		uart0_tx_string("\n\tdec: ");
		for(int i=0;i<4;i++){
			//uart0_tx_string_IT(IntToString(str[i]));
			//uart0_tx_char(str[i]);
			if(ch==CH0)uart0_tx_string(IntToString(rfid_uid_ch0[i]));
			else uart0_tx_string(IntToString(rfid_uid_ch1[i]));
			//_delay_ms(10);
		}
		
		
		break;
		case HEXDECIMAL_TYPE:
		
		uart0_tx_string("\n\thex: ");
		for(int i=0;i<4;i++){
			//uart0_tx_string_IT(IntToString(str[i]));
			//uart0_tx_char(str[i]);
			if(ch==CH0)uart0_tx_string(HexToString(rfid_uid_ch0[i]));
			else uart0_tx_string(HexToString(rfid_uid_ch1[i]));
			//_delay_ms(10);
		}
		
		break;
		
	}
	
}


void esp8266_init(unsigned char* ssid, unsigned char* pw, unsigned char * ip, unsigned char* port)
{
	uart1_tx_string("AT+RST\r\n");
	_delay_ms(100);
	uart1_tx_string("AT+CWMODE=3\r\n");
	uart1_tx_string(send_SSID_TEST("AT+CWJAP=\"",ssid,pw));
	
}



void setSoundClip(char clip){
	 // 부저 관련 tick.clear
	 switch(clip)
	 {
		 
		   case BUZZ_MUTE: music_flag=BUZZ_MUTE; break;
		   case BUZZ_ON: music_flag=BUZZ_ON; break;
		   case BUZZ_SUCCESS: music_flag=BUZZ_SUCCESS; break;
		   case BUZZ_FAIL: music_flag=BUZZ_FAIL; break;
	 }
	 TICK.buzz_1ms=0;
}


void buzz_play(){
	 //재생이 끝났으면 music_flag는 확실하게 MUTE로 들어가야 함. 안그러면 꼬이는 것 같다.
	  switch(music_flag)
	  {
		  case BUZZ_MUTE:  buzz_MUTE(); break; //setICR3(0);. buzz_MUTE() 안에 music_flag=MUTE 넣어주는 명령 들어있음.
		  
		  case BUZZ_ON: 
		   if(TICK.buzz_1ms<200)setSoundNote(Ca);
		   else if(TICK.buzz_1ms==200) setSoundNote(E);
		   else if(TICK.buzz_1ms==400) setSoundNote(A);
		   else if(TICK.buzz_1ms==600) setSoundNote(BUZZ_B);
		   else if(TICK.buzz_1ms==800) setSoundNote(BUZZ_Cs);
		   else if(TICK.buzz_1ms==1200) buzz_MUTE();
		   break;

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