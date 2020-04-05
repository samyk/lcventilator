#include "config.h"
#include "simulate.h"
#include "ventilator.h"

///////////////////////////////////////////////////////////////
// simulation helpers
#ifdef SIMULATE

#define SIM_DELAY 100

#define MAX_ANALOG_PINS 20
#define MAX_DIGITAL_PINS 20

uint8_t analogReadValues[MAX_ANALOG_PINS];
uint8_t digitalReadValues[MAX_DIGITAL_PINS];
uint8_t outputPins[MAX_DIGITAL_PINS];

struct timeval _start, _now; // set in main


// XXX
void pinMode(uint8_t, uint8_t)
{
  delay(SIM_DELAY);
}

void digitalWrite(uint8_t pin, uint8_t val)
{
  outputPins[pin] = val ? 255 : 0;
  delay(SIM_DELAY);
}

void analogWrite(uint8_t pin, int val)
{
  outputPins[pin] = val;
  delay(SIM_DELAY);
}

int digitalRead(uint8_t pin)
{
  return digitalReadValues[pin];
}
int analogRead(uint8_t pin)
{
  return analogReadValues[pin];
}

void delay(unsigned long ms)
{
  usleep(ms * 1000);
}

unsigned long millis()
{
  gettimeofday(&_now, NULL);
  return ((_now.tv_sec - _start.tv_sec) * 1000 + (_now.tv_usec - _start.tv_usec) / 1000);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

size_t print(long val)
{
  printf("%u", val);
  return sizeof(long);
}
size_t print(char *val)
{
  printf("%s", val);
  return strlen(val);
}

int main()
{
  analogReadValues[ACTUATOR_DIST_PIN] = 200;
  analogReadValues[CYCLES_PER_MIN_PIN] = 128;

  gettimeofday(&_start, NULL);

  setup();
  while (1)
    loop();
  return 0;
}
#endif

