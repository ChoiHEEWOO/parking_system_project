# parking_system_project
![](놀릴때.gif)

 >parking system, RFID, server, DB, etc...




# 현재 진행 상황


## 수정중...

2020.09.21 수정 요청 사항 : 

-로고젝터 끄는 시간 10 -> 30초로 변경

-등록안된사람 LCD 안뜸 

-들어갈 때 가끔 인식안되고 버그발생함

-주차하고나서걸어나가는사람 잘 보이게 고려

-릴레이스위치 절연시키기, 전원 및 제어라인 선 길게 빼기





------------------------------------------------

스테핑모터 구현
------------------------------------------------
GPIO로 손 쉽게 제어가 가능하며, 다만 모터에 공급되는 전력에 대한 케어가 되어야 하며, 이를 위해 모터 드라이버를 이용하는 것이 좋다.
모터드라이버는 L289 모듈을 이용하며, drive current는 2A까지 보장한다. peak의 경우 3A까지 버틴다고 함.
현재 개발 중에 사용하고 있는 모터의 경우 동작 전류가 0.5A내지 1A정도 된다.
현재 전원 공급원으로 사용하고 있는 어댑터의 경우 최대 2A까지 출력이 가능하다. 어느정도 여유가 있으므로 충분히 구동할 수 있을 것으로 예상된다.

 - 해당 모터의 특징을 확인해야 함
 
 - 펄스 신호 등을 분석하고, 타이밍에 맞춰 GPIO핀을 통해 해당 펄스들을 출력해줘야 한다.
 
 - 해당 펄스 신호의 경우, 모터를 충분히 드라이빙 할 수 있는 전력이 충족된 신호여야 하므로, 모터드라이버를 이용한다.
 
 - 사전에 탈조현상에 대한 개념을 확인한 뒤, 이를 고려하여 코딩을 할 것

 - 모터드라이버에 공급하는 전원(5V)은 L298N의 12V쪽에 넣어줘야 된다. 왜 그런지는 나도 이해가 안가지만 하여튼 그렇다. 5V핀에 공급하면 동작 안한다. 
 

__[ 현재 동작 ]__

 - 손으로 직접 명령 내리는 방식을 통해 한 스텝당 1.8도 씩 도는 것을 확인함.
 
 - 스텝 갱신을 5ms마다 할 때가 가장 빠른 것으로 확인. (그것 보다 빠르게 하면 이상 동작을 한다.)
 
 - 제어는 ISR에서 하는게 효과적임. 이유? >> 메인 내에서 폴링으로 짜니 헛돈다. 일부 명령을 놓치는 것 같다는 느낌이 든다.


```c
//PA0~PA3핀에 스테핑 모터 제어 라인이 연결되어 있음. 각각 A, C(\A), B, D(\B)임
uint8_t STEP_BUFFER[4]={0x05,0x06,0x0a,0x09};
```
__[ 추후 진행 ]__ 

 - 원하는 속도 및 각도 만큼 회전할 수 있도록 적당히 함수화. 각도 기준을 정해줘야 할 것 같다.
 
 - 입장 시 카드를 찍었을 때 등록된 유저라면, 문 개방하기. 이후 10초 뒤 다시 문 닫음

 - 퇴장 시 카드를 찍었을 때, 문 개방하기. 10초뒤에 다시 문 닫음

 - 새벽>아침이 되었을 때 문 아예 개방해두고 나서 시스템 종료



------------------------------------------------

릴레이스위치
------------------------------------------------

릴레이 스위치를 통해 로고젝터의 전원부를 제어한다.
로고젝터의 소모 전력을 알 필요가 있는데, 제품 메뉴얼에 언급된 바로는 릴레이스위치로 구동하기 충분한 것 같다.(직접 확인된 사항은 아님)

 - 2020.09.21 : 해당 제품은 소비전력 31W로 확인되었으며, 해당 릴레이가 충분히 커버 가능하다.
 
 

__[ 현재 진행 ]__

  - 전선 양단을 릴레이 스위치의 두 단자 (COM, NO)에 연결해두었음.
 
  - 길이 계산을 잘못해서 너무 짧음. 이 부분에 대해선 아래와 같은 고민을 해봄
 
     - 릴레이 모듈의 전원, 제어 라인을 길게 뽑을 것.
  
 -> ㅇ



__[ 추후 진행 ]__


 - 입구 쪽 RFID 리더기에 카드를 인식시킬 때, 카드 인식 실패를 제외하고 모든 상황에서 로고젝터를 킨다.

 - 로고젝터가 켜질 때마다 5분의 타이머가 갱신되고 작동된다.

 - 타이머가 끝나면 릴레이를 통해 로고젝터의 전원라인을 끊는다.




------------------------------------------------


RC-522 모듈 2개 사용.
------------------------------------------------
CH0: 입구 리더기

CH1: 출구 리더기

SPI 통신 인터페이스를 제공하는 RC-522를 이용했음. 라이브러리로 만들어두었으며, SPI라이브러리 또한 만들어 두었으니, 이 두개를 가져다 사용하면 바로 이용이 가능하다.

SPI통신 특성 상, SPI을 지원하는 모든 디바이스를 MCU에 동시에 연결할 수 있다. 즉, RC-522를 동시에 여러 대 운용할 수가 있다는 점에서, 현재 출 입구 두 곳에 RFID 리더기가 달려있다.


__[ 현재 동작 ]__

 - 입구 리더기에서 카드 입력 시, ESP8266을 통해 서버로 데이터가 전송이 된다. 또한 DB에 등록된 회원의 경우 UID가 버퍼에 저장되어 
현재 주차장에 존재하는 차량으로서 기록이 된다.

 - 출구 리더기의 경우, 태그 시 소리만 나고 기록되어 있던 UID정보를 버퍼에서 지운다.


__[ 추후 진행 ]__

 - 아직 정해진 바 없음

------------------------------------------------

ESP8266 구현
------------------------------------------------
서버와 연결

UART1채널에 연결되어 있으며, ESP8266 펌웨어에서의 Baud는 9600으로 설정되어 있다. ESP8266은 AT Commands를 통해 명령을 주고 받는다는 점에서 장점이 있다. 

AT Command 전송하는 함수만 잘 갖추어져 있고, 서버가 제대로 갖춰진 상태에서라면 사용하기 쉽다.

ssid와 pw, ip 및 port 번호를 알고 있어야 하며, 이와 관련된 개념들을 잘 알고 있어야 빠른 개발이 가능할 것 같다

__[ 현재 동작 ]__

 - RFID를 통해 얻어낸 UID 값을 ESP8266으로 전송함. 

 - ESP8266은 현재 연결된 TCP서버로 해당 데이터를 전송 함.

 - 서버에서 보낸 데이터를 ESP8266에서 수신하며, ESP8266은 이 데이터를 패킷에 담아 다시 UART를 통해 mcu로 보내준다.


__[ 추후 진행 ]__

 - 서버로 부터 시간 데이터 수집 : (밤/아침)여부 파악을 위해

 - 


------------------------------------------------

CLCD 구현
------------------------------------------------

I2C 통신 인퍼테이스를 제공하는 CLCD모듈을 이용했음. 마찬가지로 라이브러리로 만들어두었으며, 이 두개를 가져다 사용하면 즉시 이용이 가능하다. 
SPI통신과 마찬가지로 I2C 또한 여러 디바이스를 동시에 이용이 가능하다. 즉, 추후에 또 다른 디바이스가 필요하다면 추가하면 된다.

__[ 현재 동작 ]__ 

 - 시스템 부팅 시 로딩화면을 띄움으로써, 시스템 내부 진행상황을 직접 확인할 수 있도록 UI를 구현함.

 - Initialization 시, 에러가 발생할 경우 이를 감지하여 리셋하라는 경고 창을 띄우도록 함.

 - 유저가 카드를 찍을 경우, 확인된 유저의 경우 해당 유저의 이름을 출력해주며 환영 메시지를 띄워줌. 동시에 주차장 여석 수도 함께 띄움.

 - 이미 찍었던 유저의 경우, 해당 유저의 이름과 함께 이미 찍었다는 알림을 띄움. 동시에 주차아 여석 수도 함께 띄워줌.

 - 확인되지 않은 유저의 경우, 등록되지 않은 카드라고 알림을 띄움.

 - 카드 인식이 정상적으로 되지 않은 경우, 다시 시도해달라는 알림을 띄움. 

 - 알림을 띄운 뒤, 5초나 10초뒤에 자동으로 화면을 클리어하고 백라이트를 끔.


__[ 추후 진행 ]__


------------------------------------------------

부저 구현
------------------------------------------------
Timer0와 Timer3를 통해 만들었음.

TIM0를 통해 tick TIMER를 구현하고 TIM3를 통해 주파수를 가변할 수 있는 클럭을 만듦으로써 원하는 음을 낼 수 있는 부저를 구현함.


__[ 현재 동작 ]__

-> 시스템 부팅 음 : #define BUZZ_ON 0x01

-> 시스템 연결 성공 음 : #define BUZZ_ESP8266_CONNECTED 0x05

-> 등록된 유저 카드 인식 음 : #define BUZZ_SUCCESS 0x02

-> 미 등록된 유저 카드 인식 음 : #define BUZZ_UNENROLLED 0x03

-> 카드 인식 실패 음 : #define BUZZ_FAIL 0x04


__[ 추후 진행 ]__

? 아직 계획 없음 ? 



------------------------------------------------

영상처리 
------------------------------------------------
영상처리의 경우 개발 진행 중이며, 주차장 내 차량 갯수에 대한 검출이 가능해지면, 해당 데이터를 MCU에서 UART를 통해 수신 받아 LCD로 띄우도록 한다.

__[ 추후 진행 ]__ 

-> UART로 차량 갯수 받아온 뒤 LCD에 주차공간 여석 수 출력
