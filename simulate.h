#ifndef _SIMULATE_H
#define _SIMULATE_H

#ifdef SIMULATE

#include <unistd.h>   // usleep
#include <stdio.h>    // printf, sprintf
#include <stdint.h>   // types
#include <string.h>   // strlen
#include <sys/time.h> // timeval, gettimeofday

// some default vals
#define A0 30
#define A1 31
#define INPUT  1
#define OUTPUT 0

// disable LCD
#undef LCD
#undef LCD_I2C

void pinMode(uint8_t, uint8_t);
int digitalRead(uint8_t);
void digitalWrite(uint8_t pin, uint8_t val);
int analogRead(uint8_t);
void analogWrite(uint8_t pin, int val);
void delay(unsigned long ms);
unsigned long millis();
long map(long x, long in_min, long in_max, long out_min, long out_max);
size_t print(long val);
size_t print(char *val);

#endif // SIMULATE

#endif // _SIMULATE_Hstruct timeval _start, _now; // set in main;
