# parking_system_project
![](놀릴때.gif)

parking system, RFID, server, DB, etc...


# 현재 진행 상황
================================================



RC-522 모듈 2개 사용.
------------------------------------------------
CH0: 입구 리더기

CH1: 출구 리더기

SPI 통신 인터페이스를 제공하는 RC-522를 이용했음. 라이브러리로 만들어두었으며, SPI라이브러리 또한 만들어 두었으니, 이 두개를 가져다 사용하면 바로 이용이 가능하다.

SPI통신 특성 상, SPI을 지원하는 모든 디바이스를 MCU에 동시에 연결할 수 있다. 즉, RC-522를 동시에 여러 대 운용할 수가 있다는 점에서, 현재 출 입구 두 곳에 RFID 리더기가 달려있다.


[ 현재 동작 ]

입구 리더기에서 카드 입력 시, ESP8266을 통해 서버로 데이터가 전송이 된다. 또한 DB에 등록된 회원의 경우 UID가 버퍼에 저장되어 
현재 주차장에 존재하는 차량으로서 기록이 된다.

출구 리더기의 경우, 태그 시 소리만 나고 기록되어 있던 UID정보를 버퍼에서 지운다.


ESP8266 
------------------------------------------------
서버와 연결

UART1채널에 연결되어 있으며, ESP8266 펌웨어에서의 Baud는 9600으로 설정되어 있다. ESP8266은 AT Commands를 통해 명령을 주고 받는다는 점에서 장점이 있다. 


CLCD 구현
------------------------------------------------

I2C 통신 인퍼테이스를 제공하는 CLCD모듈을 이용했음. 마찬가지로 라이브러리로 만들어두었으며, 이 두개를 가져다 사용하면 즉시 이용이 가능하다. 
SPI통신과 마찬가지로 I2C 또한 여러 디바이스를 동시에 이용이 가능하다. 즉, 추후에 또 다른 디바이스가 필요하다면 추가하면 된다.

[현재 동작]

->시스템 부팅 시 로딩화면을 띄움으로써, 시스템 내부 진행상황을 직접 확인할 수 있도록 UI를 구현함.\n
->Initialization 시, 에러가 발생할 경우 이를 감지하여 리셋하라는 경고 창을 띄우도록 함.\n
->



-부저 구현
------------------------------------------------
Timer0와 Timer3를 통해 만들었음.

TIM0를 통해 tick TIMER를 구현하고 TIM3를 통해 주파수를 가변할 수 있는 클럭을 만듦으로써 원하는 음을 낼 수 있는 부저를 구현함.





-스테핑모터 구현
------------------------------------------------
GPIO로 손 쉽게 제어가 가능하며, 다만 모터에 공급되는 전력에 대한 케어가 고려되어야 하며, 이를 위해 모터 드라이버를 이용하는 것이 좋다.

모터드라이버는 L289 모듈을 이용하며, drive current는 2A이다. peak의 경우 3A까지 버틴다고 함.

현재 개발 중에 사용하고 있는 모터의 경우 동작 전류가 0.5A내지 1A정도 된다.

현재 전원 공급원으로 사용하고 있는 어댑터의 경우 최대 2A까지 출력이 가능하다. 어느정도 여유가 있으므로 충분히 구동할 수 있을 것으로 예상된다.

[ 현재 동작 ] 





-영상처리 
------------------------------------------------
영상처리의 경우 개발 진행 중이며, 주차장 내 차량 갯수에 대한 검출이 가능해지면, 해당 데이터를 MCU에서 UART를 통해 수신 받아 LCD로 띄우도록 한다.

