#ifndef _FLASH_H
#define _FLASH_H
#include "gd32f30x.h"
#define canshu_max_size 200
extern uint8_t mode;
extern uint8_t mesh_channel;
extern char ssid[100];
extern char pwd[100];
uint8_t canshu_get_from_Json(char *data);
void write_to_flash(uint8_t *ptr);
void read_from_flash(uint8_t *ptr);
void canshu_write_to_Jsonstr(char *data);
#endif
