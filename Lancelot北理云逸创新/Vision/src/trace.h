/*
 * trace.h
 *
 *  Created on: May 5, 2016
 *      Author: odroid
 */

#ifndef TRACE_H_
#define TRACE_H_

#include "CharacterRecognition.h"
#include "lancelot_functions.h"
#include <iostream>
#include <fstream>
#include <ctype.h>
#include <serial/serial.h>
#include <unistd.h>

#define COUT_TIME 0

extern int circle_check, circles_x, circles_y, circles_r, track_check, state_v;
extern int mouse_x, mouse_y;
extern unsigned char circle_control[4];
extern int target_num, state_num, delay_ms;
extern float Pitch, Yaw, Roll;
extern NumberPosition number_position_send;
extern bool is_print_character, isTerminal;
extern int flow_pix[2], uart_good, fd;
extern int char_num;
extern bool LBtnDown;
extern serial::Serial my_serial;
extern int flag_LX_target;
extern bool have_target;
extern std::vector<std::string> state_str;
extern char ignore_char[10];
extern int debug_screen;
extern int flag_LX_target;
extern std::vector<NumberPosition> target_global;

#endif /* TRACE_H_ */
