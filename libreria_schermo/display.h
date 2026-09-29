#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
//inizializza il display
void Display_Init(void);
//renderizza lo schermo completamente
void Display_RenderAll(void);
//renderizza solo la riga dei valori
void Display_RenderValues(const int *values);
//imposta un valore (index 0..4)
void Display_SetValue(uint8_t index, int value);
//incrementa un valore (index 0..4)
void Display_IncrementValue(uint8_t index, int delta);
//imposta il carattere per la modalità
void Display_SetMode(char mode_char);

#endif
