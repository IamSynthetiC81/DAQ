#ifndef SERCOMM_H
#define SERCOMM_H

#include <Arduino.h>
// #include "Stream.h"
#include <string.h>
#include <stdlib.h>

void SERCOMM_handler(const char* message, size_t len);

void SERCOMM_init(Stream *stream, bool *_flag, unsigned long *_time, unsigned int *_sfreq, void (*_ExportSerial)(void), void (*_ExportSD)(void));

#endif