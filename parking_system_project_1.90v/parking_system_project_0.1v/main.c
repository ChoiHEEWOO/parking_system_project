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
//#define NON_USING_MY_HOTSPOT


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

//===========================RFID 입력 flag==========================//
#define DETECED 'O'
#define NON_DETECTED 'X'

//===========================RFID 입력 flag==========================//

//===========================RFID 입/출구==========================//
#define ENTRANCE_GATE 0
#define EXIT_GATE 1
#define MAX_USER_COUNT 5 //주차장 칸수
//===========================RFID 입/출구==========================//
typedef uint32_t u32;
typedef uint8_t u8;

 struct{
	 volatile uint32_t tick_1ms;
	 volatile uint32_t buzz_1ms;
	 volatile uint32_t timeout_tick_1ms;
 }TICK;
 
 
 volatile uint8_t music_flag=0; //volatile 붙여줘야 되나? >>optimize 옵션에 따라 다르겠다만,,, 붙여주는게 정석이지

 
 //RFID 관련.
 #define DUMMY_TEST_SERIAL 1  //if 1: dummy test, 0: no test at terminal
unsigned char byte;
unsigned char detected_flag_ch0=NON_DETECTED;
unsigned char detected_flag_ch1=NON_DETECTED;
uint8_t rfid_uid_ch0[MAX_LEN];
uint8_t rfid_uid_ch1[MAX_LEN];
char received_state;
uint8_t rfid_user_uid_buffer[MAX_USER_COUNT][5]={0,}; //최대 MAX_USER_COUNT명까지 입장여부 기억
int rfid_user_count_pointer=0;	
int rfid_user_flag=0;
uint8_t esp8266_received_data[50];



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

int start_timer_flag=1;
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
	TICK.timeout_tick_1ms++;
	buzz_play(); //
}

//esp8266 테스트
ISR(USART0_RX_vect)
{
	uint8_t buff=UDR0;
	uart0.buf=buff;
	
	uart1_tx_char(buff);
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
	
	while (1) 
    {//가급적 루프 안에 delay가 길게 걸리면 않도록 주의해야 함.
		//dummy code
		//PORTA^=0x01;
		
		//to use RFID channels
		static char toggle=0; 
		//every 100ms, return RFID Reader state
		RC522_data_request_per_100ms(&toggle);
		RC522_data_state_check_and_actuate(&toggle);
		if(start_timer_flag==1)
		{
			
			if(TICK.tick_1ms==7000)//5초
			{
				//문을 닫아주는 동시에 백라이트 꺼줌
				setSoundClip(BUZZ_FAIL);
				
			}
			else if(TICK.tick_1ms==10000){//10초
				//10초가 지나면 화면 클리어시키고, 백라이트 꺼줌
				i2c_lcd_noBacklight();
			}
			else if(TICK.tick_1ms>12000)
			{
				//로고젝터 오프 
				logojector_OFF();
				start_timer_flag=0;
			}
		}
		
		//dummy code
		//else if(received_state==RECEIVE_FAIL); 
		
	// buzz_play();이 함수가 타이밍에 영향을 크게 안준다면, ISR에 넣는것도 고려해봄.	
	//	
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
	
	mfrc522_init(CH0);
	mfrc522_init(CH1);
	uart_init(0,BAUD_9600); //debug channel
	uart_init(1,BAUD_9600);//esp8266() : Rx:PD2, Tx:PD3
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
	
	i2c_lcd_string(0,0,"====================");
	i2c_lcd_string(1,0,"  Parking System    ");
	i2c_lcd_string(2,0," Firmware Ver 1.90  ");
	i2c_lcd_string(3,0,"====================");
	setSoundClip(BUZZ_ESP8266_CONNECTED);
	//main loop start.
	_delay_ms(2000);
	i2c_lcd_clear();
	i2c_lcd_noBacklight();
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
			if(DUMMY_TEST_SERIAL){
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
			if(DUMMY_TEST_SERIAL){
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


void RC522_data_request_per_100ms(char* tggl){
	
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
	else if(received_state==RECEIVE_SUCCESS){//Received data service routine.
		//send to esp8266 and receive result data.
		if((*tggl)==ENTRANCE_GATE){
			//esp8266에 uid와 입구게이트 정보 전송 함수
			//while(전송 완료될 때 까지 대기)???
			
			
			/*이부분은 esp8266 구현한 뒤에 넣어야 된다 */
			uart1_tx_string("AT+CIPSEND=11\r\n"); //4byte길이 데이터 전송 예정
			//_delay_ms(20); //위 데이터 다 보낼때 까지 대기해야 하는데, 사실 없어도 되는 라인
			// '>' 문자가 확인될 때까지 대기
			while(!esp8266_send_ready_flag);//'>'문자 들어왔는지 검사
			esp8266_send_ready_flag=0;
			
			//esp8266으로 uid데이터 전송
			
			//미리 받을 준비 시작
			memset(esp8266_received_data,0,sizeof(esp8266_received_data)); 
			for(int i=0; i<4; i++)
			{
				uart1_tx_string(HexToString(rfid_uid_ch0[i]));
				uart1_tx_char(' ');
			}
			uart1_tx_string("\r\n");

			//uart0_tx_string("\nline:304\n");
			/*이 부분은 esp8266 구현한 뒤에 넣어야 된다.*/
			//전송 후, 서버에서 결과물을 다시 전송해주기까지 대기
			while(!esp8266_receiving_flag); //ISR내에서 버퍼에 모두 담을때 까지 대기 esp8266_received_data[] 에 저장
			esp8266_receiving_flag=0;
			//uart0_tx_string("\nline:309\n");
			//esp8266_receive_complete_flag=0;
			if(esp8266_received_data[0]=='O'){
				//DB 테이블에 존재하는 uid일 경우 해당 구문을 무조건 돌음
				//uart0_tx_string("\nline:313\n");
				start_timer(); //ticktim을 0으로 클리어시킴.
				logojector_ON();
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
						}else if(strcmp((char*)rfid_user_uid_buffer[i],(char*)rfid_uid_ch0)==0){
						//만일 버퍼 안에 기존 유저가 들어있다면
						i=MAX_USER_COUNT; //그 이후는 의미 없기 때문에 그냥 빠져나옴
						rfid_user_flag=0;
					}
					//else rfid_user_flag=0;//모두 꽉 차 있음.
					
				}
				//uart0_tx_string("\nline:336\n");
				if(rfid_user_flag){//DB에 uid가 존재할뿐더러, 최초 입장시에만 해당 구문을 들어감. 이후에는 인식안됨.
					strcpy((char*)rfid_user_uid_buffer[rfid_user_count_pointer],(char*)rfid_uid_ch0);
					//LCD ON
					
					
					//start_timer(); //ticktim을 0으로 클리어시킴.
					
					//uart0_tx_string("\nline:344\n");
					i2c_lcd_clear();
					i2c_lcd_string(0,0,"Welcome,");
					i2c_lcd_string(1,2,(char*)esp8266_received_data);
					i2c_lcd_string(2,0,"Empty Space=[00 /42]");
					i2c_lcd_string(2,13,"40");
					setSoundClip(BUZZ_SUCCESS);
				}
				else {//한 번 초과로 인식시켰을 때 지나는 구문
					i2c_lcd_clear();
					i2c_lcd_string(0,0,"Welcome,");
					i2c_lcd_string(1,2,(char*)esp8266_received_data);
					i2c_lcd_string(2,0,"Already Recognized");
					
				}
			}//if(esp8266_received_data[0]=='O') end
			else if(esp8266_received_data[0]!='O') setSoundClip(BUZZ_FAIL);
			
			_delay_ms(20);
			//dummy test code (서버로부터 결과 값 수신 결과 확인)
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
			
			//_delay_ms(100);
			
			//strcpy((char*)esp8266_received_data,"SUCCESS,CHOI HEE WOO"); //결과 데이터 저장.
			
			
			
			//LCD 뷰어 및 5초 카운트 후 다시 리셋
			
		}//if(toggle==ENTRANCE_GATE) end
		
		else if((*tggl)==EXIT_GATE){
			//esp8266에 uid와 출구게이트 정보 전송 함수
			//구현안하기로 함
			// 구현해둬야 함. ==> 사람들 나가는 것 정도는 확인할 필요가 있음.
			
			
			//흠.... 등록되어있는사람일 경우에 무조건 열어주는방식으로 할까		: esp8266으로부터 데이터 받은 뒤에 그냥 열어줌
			//입장한 사람에 한정해서만 나갈 수 있도록 제한하는 방식으로 할까		:  >> 이게 타당하다 :
			for(int i=0; i<MAX_USER_COUNT;i++)
			{
				if(strcmp((char*)rfid_user_uid_buffer[i],(char*)rfid_uid_ch1)==0){//출구에서 찍은 카드가 이용객 버퍼에 존재한다면
					strcpy((char*)rfid_user_uid_buffer[i],"0000");
					//절대 버퍼에는 중복되는 값이 들어가지 않도록 코드가 작성되어 있기 때문에 여기다가 명령구문을 넣어도 될듯
					start_timer(); //ticktim을 0으로 클리어시킴.
					setSoundClip(BUZZ_SUCCESS);
				}//그곳 버퍼를 비움
				
			}
			//dummy test code
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
			//마찬가지로 액추에이터 동작시킴
			
		}
		
		
	}
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
	_delay_ms(100);// OK sign 말고도 Linked sign까지 들어온다. 이 문자까지 잡아내려면 또 구문을 추가해야되는데, 번거로워서 일단 딜레이로 처리함.

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
		   case BUZZ_UNENROLLED: music_flag=BUZZ_UNENROLLED; break;
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
		  case BUZZ_UNENROLLED:
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

void start_timer(void)
{
	TICK.tick_1ms=0;
	start_timer_flag=1;
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
	PORTC&=~(1<<4);
}