/*
 * buzzer_cmd.h
 *
 * Created: 2020-08-18 오후 2:59:49
 *  Author: CHOI
 */ 
#ifndef BUZZER_CMD_H
#define BUZZER_CMD_H


//타이머 1에 대한 주파수 ICR 값
#define BUZZ_60Hz 16667
#define BUZZ_120Hz 8334
#define BUZZ_240Hz 4167
#define BUZZ_480Hz 2084
#define BUZZ_960Hz 1042
#define BUZZ_1920Hz 521
#define BUZZ_500Hz 2000


#define _960Hz 1042
#define Ca 1691
#define E 1511
#define A 1125

#define BUZZ_C 1875
#define BUZZ_D 1706
#define BUZZ_E 1511
#define BUZZ_F 1475
#define BUZZ_G 1250
#define BUZZ_A 1125
#define BUZZ_B 1021
#define BUZZ_Cs 760


/**setSoundClip 아래 효과음 리스트 추가 요망*/

#define BUZZ_MUTE 0x00 //
#define BUZZ_ON 0x01
#define BUZZ_SUCCESS 0x02
#define BUZZ_FAIL 0x03 //인식 실패 시 :: 삐~~-삐~~-삐~~

// #define BUZZ_BEEP 0x01 //인식 시 삐빕
// #define BUZZ_POWERON 0x03 //처음 전원 인가 후 로딩 중
// #define BUZZ_DOOR_OPEN 0x04 //인식 후 문 열 때 효과음

#endif