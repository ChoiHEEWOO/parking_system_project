/*
 * parking_system_project_0.1v.c
 *
 * Created: 2020-08-18 오후 1:13:40
 * Author : CHOI
 */ 

/*

-부저 set함수는 반드시 다른 통신들 다 처리한 뒤에 호출해줘야 한다. ex. lcd출력함수를 사용한 다음에 사용해야 소리가 안깨짐.


*/
#define FIRMWARE_VERSION "1.90v"
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#include "spi_lib.h" 
#include "uart_lib.h"
#include "rc522.h"//여기 안에 spi라이브러리 선언해두었음.
#include "buzzer_cmd.h"
#include "timer_lib.h"
#include "clcd_i2c.h"

#define USING_MY_HOTSPOT
///#define NON_USING_MY_HOTSPOT

//버퍼를 생성해 여석 여부를 배열을 통해 확인하는 방식
#define USER_STATE_DEBUG 1
//RFID 시리얼 데이터 UART0를 통해 출력할 때 사용
#define DUMMY_TEST_SERIAL 0  //if 1: dummy test, 0: no test at terminal
//다른 장치들 연결 안된 상태에서 테스트해야 할 경우 0으로 설정
#define MOTOR_DEBUG_WITHOUT_ANOTHER_SENSOR 1 //if 1: default, 0: Motor Test.

//본인 핸드폰 핫스팟 환경일 때
#ifdef USING_MY_HOTSPOT
	#define SSID "ChoiHW"
	#define PASSWORD "hwhwhwhw0000"
	#define IP "172.20.10.3" //공유기에서 할당해준 사설아이피
	#define PORT "23"
#endif

//팀원이 만든 가상환경에서의 서버
#ifdef NON_USING_MY_HOTSPOT
	#define SSID "abcde"
	#define PASSWORD "19990305"
	#define IP "192.168.43.27" //가상서버에서 할당받은 사설아이피
	#define PORT "9000"
#endif

//==================== Initialize Timeout Error Code=================//
#define TIMEOUT_ATRST		"AT+RST"
#define TIMEOUT_ATGMR		"AT+GMR"
#define TIMEOUT_ATCWMODE	"AT+CWMODE"
#define TIMEOUT_ATCWMODE_	"AT+CWMODE?"
#define TIMEOUT_ATCWLAP		"AT+CWLAP"
#define TIMEOUT_ATCWJAP		"AT+CWJAP"
#define TIMEOUT_ATCIFSR		"AT+CIFSR"
#define TIMEOUT_ATCIPSTART	"AT+CIPSTART"
//==================== Initialize Timeout Error Code=================//


//==================== start_timer_flag관련 define  =================//
//start_timer_flag
#define AFTER_VERIFIED_EVENT 1
#define AFTER_EXIT_USER_EVENT 2
#define AFTER_NON_REGISTERED_EVENT -1
#define STOP_TIMER 0

//==================== start_timer_flag관련 define  =================//

//===========================RFID 입력 flag==========================//
#define DETECED 'O'
#define NON_DETECTED 'X'

//===========================RFID 입력 flag==========================//

//===========================RFID 입/출구==========================//
#define ENTRANCE_GATE 0
#define EXIT_GATE 1
#define MAX_USER_COUNT 20 //주차장 칸수
//===========================RFID 입/출구==========================//
typedef uint32_t u32;
typedef uint8_t u8;

 struct{
	 volatile uint32_t tick_1ms;
	 volatile uint32_t buzz_1ms;
	 volatile uint32_t logojector_tick_1ms;
	 volatile uint32_t lcd_tick_1ms;
	 volatile uint32_t exit_gate_tick_1ms;
	 volatile uint32_t entrance_gate_tick_1ms;
	 volatile uint32_t timeout_tick_1ms;
 }TICK;
 
 
 volatile uint8_t music_flag=0; //volatile 붙여줘야 되나? >>optimize 옵션에 따라 다르겠다만,,, 붙여주는게 정석이지

 
 //RFID 관련.
unsigned char byte;
unsigned char detected_flag_ch0=NON_DETECTED;
unsigned char detected_flag_ch1=NON_DETECTED;
//리더기로부터 받아온 uid를 저장하는 버퍼
uint8_t rfid_uid_ch0[MAX_LEN];
uint8_t rfid_uid_ch1[MAX_LEN];
char received_state;
//0번채널로 인식 시, 해당 카드 정보를 차곡차곡 저장해주는 버퍼
uint8_t rfid_user_uid_buffer[MAX_USER_COUNT][5]={0,}; //최대 MAX_USER_COUNT명까지 입장여부 기억
int rfid_user_count_pointer=0;	
int rfid_user_flag=0;
uint8_t esp8266_received_data[50];
uint8_t step_motor_rot[4]={0x05, 0x06, 0x0a, 0x09};


void systems_init(void);
void mfrc_print_serial(int _type, unsigned char ch);
 //===========================RFID 데이터 형식 출력==========================//
 #define ASCII_TYPE 0
 #define DECIMAL_TYPE 1
 #define HEXDECIMAL_TYPE 2
 //===========================RFID 데이터 형식 출력==========================//
char mfrc_check_and_data_receive_ch0(void); //여기선 카드 인식 자체의 성공 실패 여부만 체크.
char mfrc_check_and_data_receive_ch1(void); //여기선 카드 인식 자체의 성공 실패 여부만 체크.
//esp8266에는 성공 후 무조건 단 한번만 넘기기만 하면 됨. 실패하면 안넘기면 되는거고
void rfid_user_uid_buffer_init(void);
void RC522_data_request_per_100ms(char* tggl);
void RC522_data_state_check_and_actuate(char *tggl);
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
volatile unsigned char esp8266_send_ready_flag=0;
volatile unsigned char esp8266_receiving_flag=0;
volatile uint8_t esp8266_return_result_flag=0;
volatile uint8_t receive_length=0;
volatile int receive_length_int;


void start_timer();
void start_timeout_count();

void timeout_check(char* state);
void request_reset_to_admin(char* state);

void logojector_ON();
void logojector_OFF();

void flag_switch(int flag);


#define STEP_MOTOR_CW 1
#define STEP_MOTOR_CCW -1
#define STEP_MOTOR_DIABLE 0

#define GATE_ENT_OPEN 1
#define GATE_CLOSE 0
#define GATE_EXT_OPEN -1

void set_step_rot(int dir);
int set_step_speed(int _spd);
void motor_drive();
void set_step_dir_and_angle(int dir,int angle);
void set_gate_motor_state(int state);
volatile int set_motor_flag=0;
volatile int set_angle;

int logojector_timer_flag=0;
int start_after_verified_timer_flag=0;
int start_after_no_registered_timer_flag=0;
int start_after_exit_user_timer_flag=0;
int lcd_timer_flag=0;

int gate_busy_flag=0;
int gate_busy_buffer=0; //A사용자가 카드를 찍은 뒤 동작 중인 상황에서, B사용자가 추가로 카드를 찍었을 때 
//해당 동작에 대한 정보를 기억해뒀다가 A동작이 모두 끝난 뒤 B동작 수행하기 위해 만들어진 버퍼


int start_timeout_count_flag=1;
ISR(TIMER0_COMP_vect) // 1khz 속도로 ISR 진입
{
	//dummy code to check 
	//PORTA ^=0x02;
	
// 	static u32 ticks=0;
// 	ticks++;
// 	if(ticks%10==0){//0.1khz마다 증가
	TICK.buzz_1ms++;
	TICK.tick_1ms++;
	TICK.logojector_tick_1ms++;
	TICK.lcd_tick_1ms++;
	TICK.exit_gate_tick_1ms++;
	TICK.entrance_gate_tick_1ms++;
// 	TICK.verified_tick_1ms++;
// 	TICK.no_registered_tick_1ms++;
	TICK.timeout_tick_1ms++;
	
	buzz_play(); //
	
	motor_drive();
}


volatile int dir=0;
volatile unsigned char spd=9; //9 is default ! high speed & low current
volatile int steps=0;
volatile int set_step=0;
//esp8266 테스트
ISR(USART0_RX_vect)
{
	uint8_t buff=UDR0;
	uart0.buf=buff;
// 	if((buff=='a')||(buff=='s')||(buff=='d'))dir=buff;
// 	else if(('0'<=buff)&&(buff<='9')){
// 		spd=buff-'0';
// 	}
	//uart1_tx_char(buff);
}

//여기에 들어가있는 코드는 완전 뒤죽박죽임. 수정해야할 상황이 생긴다면, 차라리 새로짜는게 더 낫습니다.
ISR(USART1_RX_vect)
{//esp8266으로 부터 받아오는 데이터
	static uint8_t cnt=0;
	//static uint8_t receive_length=0;
	static uint8_t parse_cnt=0;
	static uint8_t data_cnt=0;
	static uint8_t parse_data_flag=0;
	uint8_t buff=UDR1;
	if(buff=='>') esp8266_send_ready_flag=1;//서버로 UID정보 송신 준비 완료 flag
	
	else if(buff=='O') cnt=1;
	else if(buff=='K'&&cnt==1) {esp8266_return_result_flag=1;cnt=0;}
	else cnt=0;
	
	if(buff=='+')parse_cnt=1;//
	else if(buff=='I'&&parse_cnt==1)parse_cnt++; //cnt=2
	else if(buff=='P'&&parse_cnt==2)parse_cnt++; //cnt=3
	else if(buff=='D'&&parse_cnt==3)parse_cnt++; //cnt=4
	else if(buff==','&&parse_cnt==4)parse_cnt++;	
	else if(parse_cnt==5)// 자리수가 일의 자리로 들어왔을 때
	{
		receive_length=buff; parse_cnt++; //
		char buf_1[2]= {receive_length,0};
		receive_length_int=atoi((char*)buf_1);
		memset(buf_1,0,sizeof(buf_1));

	}
	else if(parse_cnt==6&&buff!=':') // :가 들어오지 않고 10의 자리 숫자의 길이가 들어왔을 때
	{
		char buf_2[3]={receive_length,buff,0};
		receive_length_int = atoi((char*)buf_2);
		memset(buf_2,0,sizeof(buf_2));
	}
	else if(parse_cnt==6) //:가 들어왔을 때 
	{
		//esp8266_receiving_flag=1;
		parse_cnt=0; data_cnt=0; parse_data_flag=1;
	}
	else if(parse_data_flag){ //길이가 4인 데이터
		esp8266_received_data[data_cnt]=buff;
		data_cnt++;
		if(data_cnt==receive_length_int) {esp8266_receiving_flag=1; parse_data_flag=0; }
	}
	else parse_cnt=0;
	
	
	//바로 터미널창에서 확인시도.
	uart0_tx_char(buff); //1ms 소요되기 떄문에 동작에 장애가 생길수도 있음 분명
	
}
int main(void)
{
    /* Replace with your application code */
	
	
	systems_init();
	
	//dummy
	//DDRF|=0x01;
	
	//dummycode
// 	_delay_ms(1000);
// 	set_step_dir_and_angle(STEP_MOTOR_CW,360);
// 	_delay_ms(2000);
// 	set_step_dir_and_angle(STEP_MOTOR_CCW,180);
// 	_delay_ms(2000);
//	set_step_dir_and_angle(STEP_MOTOR_CW,90);

	while (1) 
    {//가급적 루프 안에 delay가 길게 걸리면 않도록 주의해야 함.
		//dummy code
		//PORTA^=0x01;
		//setSoundClip(BUZZ_ON);
		
		//to use 2 RFID channels
		static char toggle=0; 
		//every 100ms, return RFID Reader state
		RC522_data_request_per_100ms(&toggle);
		RC522_data_state_check_and_actuate(&toggle);
		
		//입장 시, 확인이 성공된 유저의 경우
		if(logojector_timer_flag)
		{
				if(TICK.logojector_tick_1ms>30000)
				{
					//로고젝터 오프
					logojector_OFF();
					logojector_timer_flag=STOP_TIMER;
				}
		}
		//명령이 동시에 발생할 때, 백라이트 끄는 함수가 호출되지 않는 상황이 생겼다. 이에 대한 처리코드
		//그냥 구문을 아예 따로 lcd관련해서 timer를 분리하였다.
		if(lcd_timer_flag)
		{
			if(TICK.lcd_tick_1ms==12000)
			{
				i2c_lcd_noBacklight();
				lcd_timer_flag=STOP_TIMER;
			}
		}

		//입구에서 등록된 유저가 카드를 찍었을 때 해당 구문을 돈다.
		if(start_after_verified_timer_flag)
		{//이미 인식되었던 사람들도 마찬가지 과정을 거침
			
			//가끔 여기 문을 안들어감 뭐가 문젠지는 확인이 안됨. 
			if(TICK.tick_1ms==10000)//10초
			{
				//setSoundClip(BUZZ_ON);
				//문을 닫아주는 방향으로 모터를 돌림
				set_gate_motor_state(GATE_CLOSE);
				//set_step_dir_and_angle(STEP_MOTOR_CCW,720);
			}
			else if(TICK.tick_1ms==12000){//12초
			
				start_after_verified_timer_flag=STOP_TIMER;
				
				//발생할 버그 상황 해결을 위한 코드
				//만일 입구열림 상태 도중 출구에서 카드가 찍힌 상태라면 아래 블럭에 진입한다.
				//if(gate_busy_flag&&(gate_busy_buffer!=GATE_CLOSE))
				{
					//set_gate_motor_state(gate_busy_buffer);
					//gate_busy_buffer=GATE_CLOSE; 
					//start_timer(AFTER_EXIT_USER_EVENT);

					//출구 동작을 이후에 수행해줘야 함
					//TICK.tick_1ms=0;
					//start_after_exit_user_timer_flag=1;
					
				}
				//gate_busy_flag=0;
			}
			
		}
		if (start_after_exit_user_timer_flag)
		{
			//add some codes
			//PORTF^=0x01;
			if(TICK.tick_1ms==10000)//5초
			{
				//setSoundClip(BUZZ_ON); //전까진 소리 났음
				
				//테스트 라인임 없애도 됌 근데 정상적으로 동작하는지 확인하기 위함
				//set_step_dir_and_angle(STEP_MOTOR_CCW,720); //된다
				
				set_gate_motor_state(GATE_CLOSE);
			}
			else if(TICK.tick_1ms==12000){//10초
				//10초가 지나면 화면 클리어시키고, 백라이트 꺼줌
				//i2c_lcd_noBacklight();
				start_after_exit_user_timer_flag=STOP_TIMER;
				
				//발생할 버그 상황 해결을 위한 코드
				//만일 출구열림 상태 도중 입구에서 카드가 찍혔다면?
				//if(gate_busy_flag&&(gate_busy_buffer!=GATE_CLOSE))
				{
				//	set_gate_motor_state(gate_busy_buffer);
				//	gate_busy_buffer=GATE_CLOSE;

					//입구 동작을 이후에 수행해줘야 함
				//	TICK.tick_1ms=0;
				//	start_after_verified_timer_flag=1;
				}
				//gate_busy_flag=0;
			}
		}
		
		//입장 시, 미 등록된 유저의 경우
		//이 블럭이 있어야되나 모르겠다. 위에 다르게 구현해놓긴했다. 일단 주석처리
// 		if(start_after_no_registered_timer_flag)
// 		{
// 			// add some codes
// 			if(TICK.tick_1ms==12000){//10초
// 				//10초가 지나면 화면 클리어시키고, 백라이트 꺼줌
// 				//i2c_lcd_noBacklight();
// 				start_after_no_registered_timer_flag=STOP_TIMER;
// 			}
// 		}


		//dummy code
		//else if(received_state==RECEIVE_FAIL); 
		
    }
}

void systems_init(void){
	sei();
	//DDRA|=0x03; //test Port
	//0~3번비트	: 스테핑모터 제어
	DDRA=0x0f;
	//4번비트	: 릴레이스위치
	DDRC |= (1<<4);
	cli(); //전역 인터럽트 해제
	
	timer0_init();
	timer3_init();
	sei(); //전역 인터럽트 허용
	TICK.tick_1ms=0;
	//사용하는 기능들 초기화 작업
	
	logojector_OFF();
	uart_init(0,BAUD_9600); //debug channel
	uart_init(1,BAUD_9600);//esp8266() : Rx:PD2, Tx:PD3
	#if MOTOR_DEBUG_WITHOUT_ANOTHER_SENSOR
		mfrc522_init(CH0);
		mfrc522_init(CH1);
		i2c_lcd_init();
	
	
		i2c_lcd_string(0,0,"====================");
		i2c_lcd_string(1,0,"  SYSTEM BOOTING...");
		i2c_lcd_string(2,0,"     __________     ");
		i2c_lcd_string(3,0,"====================");
		setSoundClip(BUZZ_ON);
		_delay_ms(2500);
	
		//로딩 시작. RFID모듈체크, ESP8266 연결 체크
	
		mfrc522_version_check(CH0);
		mfrc522_IRQ_enable(CH0);
		mfrc522_version_check(CH1);
		mfrc522_IRQ_enable(CH1);
		esp8266_init((unsigned char*)SSID,(unsigned char*)PASSWORD,(unsigned char*)IP,(unsigned char*)PORT);
		rfid_user_uid_buffer_init();
	
		char version_buf[20] = " Firmware Ver ";
		strcat(version_buf,(const char*)FIRMWARE_VERSION);
		i2c_lcd_string(0,0,"====================");
		i2c_lcd_string(1,0,"  Parking System    ");
		i2c_lcd_string(2,0, version_buf);
		i2c_lcd_string(3,0,"====================");
		setSoundClip(BUZZ_ESP8266_CONNECTED);
		//main loop start.
		_delay_ms(2000);
		i2c_lcd_clear();
		i2c_lcd_noBacklight();
	#endif
}

char mfrc_check_and_data_receive_ch0(void){ 
	//하... 이 복병을 해결하는 방법은 detect_flag를 다른 곳에서 돌아오도록 처리해주는 방법밖에 안떠오른다. 기모띵 
	
	//원인 모를 버그를 해결하기 위한 용도로 쓰는 flag : 카드 인식 request 시, return 할 때 oxoxoxoxox 이짓 하는 버그 발생	
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
	////////////////////////////////////////////
	///////
			//
			//dummy code
			//setSoundClip(BUZZ_SUCCESS);
			#if DUMMY_TEST_SERIAL
			{
				uart0_tx_string("[CHECK UID(CH0)]: ");
				mfrc_print_serial(ASCII_TYPE,CH0);
				mfrc_print_serial(DECIMAL_TYPE,CH0);
				mfrc_print_serial(HEXDECIMAL_TYPE,CH0);
				uart0_tx_char('\n');
			}
			#endif
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
			#if DUMMY_TEST_SERIAL 
			{
						uart0_tx_string("[CHECK UID(CH1)]: ");
						mfrc_print_serial(ASCII_TYPE,CH1);
						mfrc_print_serial(DECIMAL_TYPE,CH1);
						mfrc_print_serial(HEXDECIMAL_TYPE,CH1);
						uart0_tx_char('\n');
			}
			#endif
			
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


void RC522_data_request_per_100ms(char* tggl)
{
	
	if((TICK.tick_1ms % 100) ==0) {
		
		//toggle = 0 : entrance gate
		//toggle = 1 : exit gate
		(*tggl)^=0x01; //start toggling :
		
		if((*tggl)==0){
			received_state = mfrc_check_and_data_receive_ch0();
			rfid_uid_ch0[4]=0; //배열을 문자열처럼 사용하기 위해 (문자 끝에 null을 넣어주기 위함. "abcd")
		}//RFID check and receive UID data per 100ms
		else {
			received_state = mfrc_check_and_data_receive_ch1();  //UID values are in 'rfid_uid_chX[]'
			rfid_uid_ch1[4]=0;
		}
	}
}

void RC522_data_state_check_and_actuate(char *tggl)
{
	if(received_state==RECEIVE_NONE); //do nothing
	else if(received_state==RECEIVE_SUCCESS)
	{//Received data service routine.
		//send to esp8266 and receive result data.
		
		static int user_count=0;//입구,출구 둘다 사용해야 하는 변수이므로 일단 여기다가 선언함.
		if((*tggl)==ENTRANCE_GATE){
			//esp8266에 uid와 입구게이트 정보 전송 함수
			//while(전송 완료될 때 까지 대기)???
			
			
			/*이부분은 esp8266 구현한 뒤에 넣어야 된다 */
			uart1_tx_string("AT+CIPSEND=11\r\n"); //4byte길이 데이터 전송 예정
			// '>' 문자가 확인될 때까지 대기
			while(!esp8266_send_ready_flag);//'>'문자 들어왔는지 검사
			esp8266_send_ready_flag=0;
			
			//esp8266으로 uid데이터 전송
			
			//미리 받을 사전 준비 시작.
			memset(esp8266_received_data,0,sizeof(esp8266_received_data)); 
			
			//uid 데이터 전송
			for(int i=0; i<4; i++)
			{
				uart1_tx_string(HexToString(rfid_uid_ch0[i]));
				uart1_tx_char(' ');
			}
			uart1_tx_string("\r\n");

			/*이 부분은 esp8266 구현한 뒤에 넣어야 된다.*/
			//전송 후, 서버에서 결과물을 다시 전송해주기까지 대기
			while(!esp8266_receiving_flag); //ISR내에서 버퍼에 모두 담을때 까지 대기 esp8266_received_data[] 에 저장
			esp8266_receiving_flag=0;
			//esp8266_receive_complete_flag=0;
			if(esp8266_received_data[0]=='O'){
				//DB 테이블에 존재하는 uid일 경우 해당 구문을 들어옴

				strncpy((char*)esp8266_received_data,"  ",2);
				
				//현재 입장객 버퍼 비어있는 인덱스 체크
				rfid_user_flag=0;
				for(int i=0; i<MAX_USER_COUNT;i++)
				{
					
					//인덱스를 모두 체크해줘서 한번 인식이 유저의 경우
					// 다시 카드 인식시키지 않도록 구현
					if(strcmp((char*)rfid_user_uid_buffer[i],"0000")==0){
						//해당 위치의 버퍼가 비어있는 것이 확인된다면
						rfid_user_count_pointer=i;
						rfid_user_flag=1;
						//i=MAX_USER_COUNT;//루프를 나오기 위함
					}
					else if(strcmp((char*)rfid_user_uid_buffer[i],(char*)rfid_uid_ch0)==0)
					{
						//만일 버퍼 안에 기존 유저가 들어있는 것이 확인됐을 때 
						i=MAX_USER_COUNT; //그 이후는 의미 없기 때문에 그냥 빠져나옴
						rfid_user_flag=0;
					}
					//else rfid_user_flag=0;//모두 꽉 차 있음.
					
				}
				
				
				if(rfid_user_flag){//DB에 uid가 존재할뿐더러, 최초 입장시에만 해당 구문을 들어감. 이후에는 인식안됨.
					strcpy((char*)rfid_user_uid_buffer[rfid_user_count_pointer],(char*)rfid_uid_ch0);
					
					//사용자 인식이 정상적으로 되면 1회에 한해서 증가시킴.
					if(user_count<MAX_USER_COUNT)user_count++; //단, 주차장 최대 수용 수 보다는 작아야 함.
							
					char MAX_USER_COUNT_STR[4];
					strcpy(MAX_USER_COUNT_STR,IntToString((int)MAX_USER_COUNT));
					
					char USER_COUNT_STR[4];
					//char dummy_value=1;
					strcpy(USER_COUNT_STR,IntToString((int)MAX_USER_COUNT-user_count));
					
					char empty_space_str[20]="Empty Space=[";
					strcat((char*)empty_space_str,USER_COUNT_STR);
					strcat((char*)empty_space_str,"/");
					strcat((char*)empty_space_str,MAX_USER_COUNT_STR);
					strcat((char*)empty_space_str,"]");
					//start_timer(); //ticktim을 0으로 클리어시킴.
					//LCD ON
					i2c_lcd_clear();
					i2c_lcd_string(0,0,"Welcome,");
					i2c_lcd_string(1,0,(char*)esp8266_received_data);
					i2c_lcd_string(2,0,(char*)empty_space_str);
					setSoundClip(BUZZ_SUCCESS);
					start_timer(AFTER_VERIFIED_EVENT); //ticktim을 0으로 클리어시킴.
					
					//set_step_dir_and_angle(STEP_MOTOR_CW,720);
					//gate_busy_flag=1;
					
					//if(gate_busy_flag==0)
					{
						set_gate_motor_state(GATE_ENT_OPEN); //한번 선언되면 gate_busy_flag가 활성화된다.
					//	gate_busy_flag=1;
					}
					//명령 동작 중에 선언되면 모터 동작하지 않고 busy buffer에 저장된다	
					//else gate_busy_buffer=GATE_ENT_OPEN;
					
					logojector_ON();
				}
				else {//한 번 초과로 인식시켰을 때 지나는 구문
					i2c_lcd_clear();  
					i2c_lcd_string(0,0,"Welcome,");
					i2c_lcd_string(1,2,(char*)esp8266_received_data);
					i2c_lcd_string(2,0,"Already Recognized");
					setSoundClip(BUZZ_SUCCESS);
					start_timer(AFTER_VERIFIED_EVENT); //ticktim을 0으로 클리어시킴.
					//set_step_dir_and_angle(STEP_MOTOR_CW,720);
					//gate_busy_flag=1;	
					//if(gate_busy_flag==0)
					{
						set_gate_motor_state(GATE_ENT_OPEN); //한번 선언되면 gate_busy_flag가 활성화된다.
					//	gate_busy_flag=1;
					}
					//타이머 동작 중에 들어오는 상황
					//else gate_busy_buffer=GATE_ENT_OPEN; //명령 동작 중에 선언되면 모터 동작하지 않고 busy buffer에 저장된다	
					
					logojector_ON();
					
				}
			}//if(esp8266_received_data[0]=='O') end
			else if(esp8266_received_data[0]!='O') 
			{
				i2c_lcd_clear();
				i2c_lcd_string(0,0,"Sorry,");
				i2c_lcd_string(1,2,"This card is");
				i2c_lcd_string(2,2,"not registered.");
				start_timer(AFTER_NON_REGISTERED_EVENT);
				setSoundClip(BUZZ_NOT_REGISTERED);
			}
			//_delay_ms(20);
			//dummy test code (서버로부터 결과 값 수신 결과 확인)
			
			#if DUMMY_TEST_SERIAL
				uart0_tx_char('\n');
				uart0_tx_string("From server : ");
				uart0_tx_string((char*)esp8266_received_data);
				uart0_tx_char('\n');
			
				//dummy test code (이용객 저장 버퍼 상태 표시)
				for(int i=0;i<MAX_USER_COUNT;i++){
					uart0_tx_char('[');
					//uart0_tx_string((char*)rfid_user_uid_buffer[i]);
					for(int j=0;j<4;j++){
						uart0_tx_string(HexToString(rfid_user_uid_buffer[i][j]));
						if(j!=3)uart0_tx_char(' ');
						//_delay_ms(10);
					}
				
				
					uart0_tx_char(']');
					uart0_tx_char('\n');
				}
			#endif 
			
			//strcpy((char*)esp8266_received_data,"SUCCESS,CHOI HEE WOO"); //결과 데이터 저장.	
			
			
			
			//LCD 뷰어 및 5초 카운트 후 다시 리셋
			
		}//if(toggle==ENTRANCE_GATE) end
		
		else if((*tggl)==EXIT_GATE)
		{
			//esp8266에 uid와 출구게이트 정보 전송 함수
			//구현안하기로 함
			// 구현해둬야 함. ==> 사람들 나가는 것 정도는 확인할 필요가 있음.
			
			for(int i=0; i<MAX_USER_COUNT;i++)
			{
				if(strcmp((char*)rfid_user_uid_buffer[i],(char*)rfid_uid_ch1)==0){//출구에서 찍은 카드가 이용객 버퍼에 존재한다면
					strcpy((char*)rfid_user_uid_buffer[i],"0000");
					//절대 버퍼에는 중복되는 값이 들어가지 않도록 코드가 작성되어 있기 때문에 여기다가 명령구문을 넣어도 될듯
					user_count--; //이용자 카운트를 감소시킴.
					start_timer(AFTER_EXIT_USER_EVENT); //ticktim을 0으로 클리어시킴.
					//if(gate_busy_flag==0)
					{//한번 선언되면 gate_busy_flag가 활성화된다.
						set_gate_motor_state(GATE_EXT_OPEN);
					//	gate_busy_flag=1;
					} 
					//else gate_busy_buffer = GATE_EXT_OPEN; //명령 동작 중에 선언되면 모터 동작하지 않고 busy buffer에 저장된다	
					
					//gate_busy_flag=1;
					setSoundClip(BUZZ_SUCCESS);
				}//그곳 버퍼를 비움
				
			}
			//dummy test code
			#if DUMMY_TEST_SERIAL
				for(int i=0;i<MAX_USER_COUNT;i++){
				
					uart0_tx_char('[');
					//uart0_tx_string((char*)rfid_user_uid_buffer[i]);
					for(int j=0;j<4;j++){
						uart0_tx_string(HexToString(rfid_user_uid_buffer[i][j]));
						if(j!=3)uart0_tx_char(' ');
					}
					uart0_tx_char(']');
					uart0_tx_char('\n');
				
				}
			#endif
			//마찬가지로 액추에이터 동작시킴
			
		}//	else if((*tggl)==EXIT_GATE) end
		
		
	} //else if(received_state==RECEIVE_SUCCESS) end.
	
	//RFID 데이터가 정상적으로 인식되지 않았을 때
	else if(received_state==RECEIVE_FAIL){
		setSoundClip(BUZZ_FAIL);
		i2c_lcd_clear();
		//i2c_lcd_string(0,0,"Welcome,")
		//i2c_lcd_string(1,2,esp8266_received_data);
		i2c_lcd_string(2,0,"Plz, Re-tagging. ");
		
	}
	
	received_state=RECEIVE_NONE;
}


void rfid_user_uid_buffer_init(void)
{
	for(int i=0; i<MAX_USER_COUNT;i++)
	{
		
		strcpy((char*)rfid_user_uid_buffer[i],"0000");
	}
	i2c_lcd_string(2,0,"     OOOOOOOOOO     ");
	_delay_ms(500);
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
			uart0_tx_char(' ');
			//_delay_ms(10);
		}
		
		break;
		
	}
	
}

//AP 끊는함수하고, TCP끊는거, 다시연결하는거 등등 > 터미널환경에서 가능케 할수도 있어야함
//리셋, 실패시 ssid, passward 묻고, 마찬가지로 TCP연결 실패시 또한 다시 ip, port 묻는다

void esp8266_init(unsigned char* ssid, unsigned char* pw, unsigned char * ip, unsigned char* port)
{
	//타임아웃 시작
	start_timeout_count();
	uart1_tx_string("AT+RST\r\n");	 //리셋 신호 이후 추가 데이터가 들어와서, 딜레이로 강제 정지 시켜줘야 함
	_delay_ms(2500);
	timeout_check(TIMEOUT_ATRST);
	i2c_lcd_string(2,0,"     OOO_______     ");
	
	
	start_timeout_count();
	uart1_tx_string("AT+GMR\r\n");
	//기본은 0이므로 무조건 돌고, OK 사인 들어오면 1로 set되어 while문 나옴
	while(!esp8266_return_result_flag){
		timeout_check(TIMEOUT_ATGMR);
	}
	esp8266_return_result_flag=0;
	
	
	start_timeout_count();
	uart1_tx_string("AT+CWMODE=1\r\n"); // OK sign 말고도 다른 신호도 들어오기 때문에 걸어놓음
	_delay_ms(2000);
	esp8266_return_result_flag=0;
	timeout_check(TIMEOUT_ATCWMODE);
	i2c_lcd_string(2,0,"     OOOO______     ");
	
	
	//
	
	start_timeout_count();
	uart1_tx_string("AT+CWMODE?\r\n");
	//기본은 0이므로 무조건 돌고, OK 사인 들어오면 1로 set되어 while문 나옴
	while(!esp8266_return_result_flag) {
		timeout_check(TIMEOUT_ATCWMODE_);
	}
	esp8266_return_result_flag=0;
	
		
	start_timeout_count();
	uart1_tx_string("AT+CWLAP\r\n");
	//기본은 0이므로 무조건 돌고, OK 사인 들어오면 1로 set되어 while문 나옴
	while(!esp8266_return_result_flag){
		timeout_check(TIMEOUT_ATCWLAP);
	}
	esp8266_return_result_flag=0;
	i2c_lcd_string(2,0,"     OOOOO_____     ");
	
	
	start_timeout_count();
	uart1_tx_string(connect_to_AP("AT+CWJAP=\"",(char*)ssid,(char*)pw));
	 //기본은 0이므로 무조건 돌고, OK 사인 들어오면 1로 set되어 while문 나옴
	while(!esp8266_return_result_flag){
		timeout_check(TIMEOUT_ATCWJAP);
	}
	esp8266_return_result_flag=0;
	i2c_lcd_string(2,0,"     OOOOOO____     ");
	
	//AT+SWQAP  AP 접속 끊기
	
	start_timeout_count();
	uart1_tx_string("AT+CIFSR\r\n");
	//기본은 0이므로 무조건 돌고, OK 사인 들어오면 1로 set되어 while문 나옴
	while(!esp8266_return_result_flag){
		timeout_check(TIMEOUT_ATCIFSR);
	} 
	esp8266_return_result_flag=0;
	i2c_lcd_string(2,0,"     OOOOOOO___     ");
	
	
	
	start_timeout_count();
	uart1_tx_string(TCP_connect((char*)ip, (char*)port));
	//AT+CIPCLOSE 
	 //기본은 0이므로 무조건 돌고, OK 사인 들어오면 1로 set되어 while문 나옴
	while(!esp8266_return_result_flag){
		timeout_check(TIMEOUT_ATCIPSTART);
	}
	esp8266_return_result_flag=0;
	i2c_lcd_string(2,0,"     OOOOOOOO__     ");
	//1byte당 해봐야 1ms 정도밖에 소요되지 않는다.
	_delay_ms(100);// OK sign 말고도 Linked sign까지 들어온다. 이 문자까지 잡아내려면 또 구문을 추가해야되는데, 번거로워서 딜레이로 처리함.

	i2c_lcd_string(2,0,"     OOOOOOOOO_     ");
	
}



void setSoundClip(char clip){
	 // 부저 관련 tick.clear
	 TICK.buzz_1ms=0;
	 switch(clip)
	 {
		 
		   case BUZZ_MUTE: music_flag=BUZZ_MUTE; break;
		   case BUZZ_ON: music_flag=BUZZ_ON; break;
		   case BUZZ_SUCCESS: music_flag=BUZZ_SUCCESS; break;
		   case BUZZ_NOT_REGISTERED: music_flag=BUZZ_NOT_REGISTERED; break;
		   case BUZZ_FAIL: music_flag=BUZZ_FAIL; break;
		   case BUZZ_ESP8266_CONNECTED: music_flag=BUZZ_ESP8266_CONNECTED; break;
		   
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
		  case BUZZ_NOT_REGISTERED:
		  		  if(TICK.buzz_1ms<75)setSoundNote(_960Hz);
		  		  else if(TICK.buzz_1ms<150)setSoundNote(BUZZ_MUTE);
		  		  else if(TICK.buzz_1ms<225)setSoundNote(_960Hz);
		  		  else if(TICK.buzz_1ms<300)setSoundNote(BUZZ_MUTE);
		  		  else if(TICK.buzz_1ms<375)setSoundNote(_960Hz);
		  		  else if(TICK.buzz_1ms==450) buzz_MUTE(); //buzz_MUTE();
		  		  break;
		  break;
		  case BUZZ_FAIL:
		  if(TICK.buzz_1ms<100)setSoundNote(_960Hz);
		  else if(TICK.buzz_1ms<200)setSoundNote(BUZZ_MUTE);
		  else if(TICK.buzz_1ms<300)setSoundNote(_960Hz);
		  else if(TICK.buzz_1ms==450) buzz_MUTE(); //buzz_MUTE();
		  break;
		  case BUZZ_ESP8266_CONNECTED:
		  if(TICK.buzz_1ms<100)setSoundNote(BUZZ_240Hz);
		  else if(TICK.buzz_1ms<200)setSoundNote(BUZZ_MUTE);
		  else if(TICK.buzz_1ms<300)setSoundNote(BUZZ_480Hz);
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

void start_timer(int flag)
{
	
	//모터 관련된 플래그는 따로 구현해야 될듯
	//flag를 실패 성공 다 나누지 말고, 출구 입구에 대한 플래그만 나눌까?
	TICK.tick_1ms=0;
	if(flag!=AFTER_EXIT_USER_EVENT)TICK.lcd_tick_1ms=0; // LCD가 출력되는 모든 상황에서 lcd tick 초기화가 된다.
	if(flag==AFTER_VERIFIED_EVENT)TICK.logojector_tick_1ms=0;
	//셋된 플래그들에 맞게 타이머 감지를 시작함.
	flag_switch(flag);
		
}
void start_timeout_count(void){
	TICK.timeout_tick_1ms=0;
	start_timeout_count_flag=0;
}

void timeout_check(char* state){
	//timeout_tick_1ms 변수가 10000을 넘지 않는다면 if구문을 돌지 않으며, 마찬가지로 while루프도 돌지 않음.
	
	if(TICK.timeout_tick_1ms>10000) {//10초 이상 경과 한 뒤 해당 구문을 들어갈 경우.
		start_timeout_count_flag=1;request_reset_to_admin(state);
	}
	while(start_timeout_count_flag); //시스템 중지v
}
void request_reset_to_admin(char* state)
{
	
	char buf[21]="ERR Code:";
	strcat(buf,state);
	
	i2c_lcd_clear();
	i2c_lcd_string(1,0,"Initial err.");
	i2c_lcd_string(2,0,"Plz, Trying reset.");
	i2c_lcd_string(3,0,buf);//따로 전역으로 빼주지 않아도 상관없음. 함수 안에서 다 처리하므로.
}

void logojector_ON(void){
	PORTC|=(1<<4);
}
void logojector_OFF(void){
	unsigned char buff = ~(1<<4); //자료형이 확실하지 않기 때문에, 확실하게 선언해준 buff를 이용
	PORTC&=buff;
}


void flag_switch(int flag)
{
	
	/*
	#define AFTER_VERIFIED_EVENT 1
	#define AFTER_EXIT_USER_EVENT 2
	#define AFTER_NON_REGISTERED_EVENT -1
	#define STOP_TIMER 0
	*/
	
	/*
	int start_after_verified_timer_flag=1;
	int start_after_no_registered_timer_flag=1;
	int start_after_exit_user_timer_flag=1;
	*/
	switch(flag)
	{
		case  AFTER_VERIFIED_EVENT: 
			start_after_exit_user_timer_flag=0;
			start_after_no_registered_timer_flag=0;
			start_after_verified_timer_flag=1;
			logojector_timer_flag=1;
			lcd_timer_flag=1;
		break;
		case AFTER_NON_REGISTERED_EVENT:
			start_after_exit_user_timer_flag=0;
			start_after_no_registered_timer_flag=1;
			start_after_verified_timer_flag=0;
			lcd_timer_flag=1;
		break;
		
		case AFTER_EXIT_USER_EVENT:
			start_after_exit_user_timer_flag=1;
			start_after_no_registered_timer_flag=0;
			start_after_verified_timer_flag=0;
		break;
		
	}
}


void set_step_rot(int dir){
	static uint32_t i =0;
	if(dir==STEP_MOTOR_CW)i++;
	else if(dir==STEP_MOTOR_CCW)i--;
	
	if(dir)PORTA=(step_motor_rot[i%4]);
	else PORTA=STEP_MOTOR_DIABLE;
}


//0~9단계 까지 가능 
int set_step_speed(int _spd){
	return (11-_spd);
}
//0~9
//속도는 2ms 갱신이 가장 이상적이며 160mA를 소모함
// 1ms갱신의 경우 제대로 동작하지 않음
// 갱신속도를 느리게 할 수록 속도가 느려지며, 전류소모도 이상하게 더 커짐
//한바퀴는 200스텝?
void motor_drive(){
	if(set_motor_flag)
	{
		if(TICK.tick_1ms%set_step_speed(spd)==0) //103~5 (
		{
			//setSoundClip(BUZZ_ESP8266_CONNECTED);
			
			if(steps<set_step){
				if(dir==STEP_MOTOR_CW)set_step_rot(STEP_MOTOR_CW);
				else if(dir==STEP_MOTOR_CCW) set_step_rot(STEP_MOTOR_CCW);
				else set_step_rot(0);
				steps++;
			}
			else {
				set_motor_flag=0;
				steps=0;
				set_step_rot(0);
// 				dir=0;
// 				set_step=0;
			}
		} 
	}
}

void set_step_dir_and_angle(int direction,int angle){
	//
	dir=direction;
	//angle   1.8도 == 1 <==> 360도 == 200
	set_step=(int)(angle*0.556);
	set_motor_flag=1;
}

void set_gate_motor_state(int state){
	//동일 명령이 계속 들어오면 그떄는 무시하도록 
	//다른 명령이 들어올 때만 인정
	
	//
	static int current_state_flag; //직전 상태 
	
	
	//동시에 게이트 동작 명령 내리는 상황을 방지하기 위한 코드
// 	if(state!=GATE_CLOSE)
// 	{
//  		if(gate_busy_flag){ //이미 동작되어 있는 상태일 때 
//  			gate_busy_buff=state; //1또는 -1이 들어가며, 0이면 아무것도 아닌 상태
//  			return;
//  		}//그게 아니라면 첫 동작이므로 buff에 현재 상태를 저장하지 않는다.
// 		gate_busy_flag=1;
// 		
// 	}
	if(state==GATE_ENT_OPEN)
	{
		if(current_state_flag==GATE_ENT_OPEN) return; //중복으로 입력했다면 무시
		else//이전과 다른 명령이 들어왔다면
		{
			switch(current_state_flag){
				case GATE_CLOSE://닫혀있다가 > 입구 오픈 명령
					set_step_dir_and_angle(STEP_MOTOR_CW,360);
				break;
				case GATE_EXT_OPEN: //출구오픈상태 > 입구 오픈 명령
					set_step_dir_and_angle(STEP_MOTOR_CW,720);
				break;
			}	
		
		}
		current_state_flag=GATE_ENT_OPEN;
	}
	
	else if(state==GATE_CLOSE)
	{
		if(current_state_flag==GATE_CLOSE) return;
		else
		{
			switch(current_state_flag){
				case GATE_ENT_OPEN://입구오픈상태에서 닫힘명령
					set_step_dir_and_angle(STEP_MOTOR_CCW,360);
				break;
				case GATE_EXT_OPEN://출구오픈상태에서 닫힘명령
					set_step_dir_and_angle(STEP_MOTOR_CW,360);
				break;
			}
		}
		current_state_flag=GATE_CLOSE;
	}
	else if(state==GATE_EXT_OPEN)
	{
		if(current_state_flag==GATE_EXT_OPEN) return;
		else
		{
			switch(current_state_flag){
				case GATE_ENT_OPEN: //입구오픈상태에서 출구오픈명령
				set_step_dir_and_angle(STEP_MOTOR_CCW,720);
				break;
				case GATE_CLOSE: //닫힌 상태에서  출구오픈명령
				set_step_dir_and_angle(STEP_MOTOR_CCW,360);
				break;
			}
		}
		current_state_flag=GATE_EXT_OPEN;
	}
}
