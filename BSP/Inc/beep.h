#ifndef __BEEP_H
#define __BEEP_H

#include "main.h"

void BEEP_ON(void);
void BEEP_OFF(void);
void BEEP_Short(void);
void BEEP_Long(void);
void BEEP_Success(void);
void BEEP_Error(void);
void BEEP_Warning(void);
void BEEP_StartUp(void);
void BEEP_Custom(uint16_t on_time_ms, uint16_t off_time_ms, uint8_t repeat_count);
void BEEP_Metronome(uint16_t bpm, uint8_t beats);

#endif
