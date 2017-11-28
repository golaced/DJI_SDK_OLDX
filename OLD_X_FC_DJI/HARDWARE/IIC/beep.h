
#ifndef __MUSIC_H__
#define __MUSIC_H__
#include  "stm32f4xx.h"
void Beep_Init(u32 arr,u32 psc);
void Tone(u8 level, u8 tone);
void Play_Music_Direct(u8 sel);
u8 Play_Music_In_Task(u8 *music, u16 st,u16 ed,u8 en,float dt);
void Play_Music_Task(u8 sel,float dt);
#define MEMS_RIGHT_BEEP 0
#define MEMS_ERROR_BEEP 1
#define START_BEEP 2
#define BAT_ERO_BEEP 3
#define RC_ERO_BEEP 4
#define BEEP_ONE 5
#define BEEP_TWO 6
#define BEEP_THREE 7
#define BEEP_STATE 8
#define BEEP_GPS_SAVE 9
extern u8 fc_state_beep[3];
#define BEEP_STATE1 03
#define BEEP_STATE2 13
#define BEEP_STATE3 23
extern u8 fc_save_gps_beep;
#endif