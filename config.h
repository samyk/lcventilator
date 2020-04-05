#ifndef _CONFIG_H
#define _CONFIG_H

#define LCD // enable LCD
//#define LCD_I2C // enable I2C LCD

#define DEBUG // enable serial debugging
#define SERIAL_SPEED 9600

/////////////////////////////
////     PINS
/////////////////////////////
#define CYCLES_PER_MIN_PIN A0
#define ACTUATOR_DIST_PIN  A1
#define MOTOR_IN1_PIN       9
#define MOTOR_IN2_PIN      10
#define MOTOR_ENABLE_PIN   13

// LCD I2C pins (if LCD_I2C is defined)
#define LCD_I2C_ADDRESS 0x27

// LCD pins (if LCD is defined)
#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4  5
#define LCD_D5  4
#define LCD_D6  3
#define LCD_D7  2

/////////////////////////////
////     VALUES
/////////////////////////////
#define MAX_CYCLES_PER_MIN 40 // max full actuations (fwd+back) per minute
#define MIN_CYCLES_PER_MIN  1 // minimum actuations per minute
#define LCD_COLUMNS 16
#define LCD_ROWS 2

#define PERCENTAGE_GOING_IN 25 // percentage of time we should go in vs out, not including delays

// motor control speed (pwm value)
#define MIN_SPEED   0
#define MAX_SPEED 255

// display values of (relative) distance
#define MIN_DISTANCE   0
#define MAX_DISTANCE 255

// analogRead range on analog pins
#define MIN_ANALOG_VALUE    0
#define MAX_ANALOG_VALUE 1023

#define DELAY_PRE_CYCLE_MS 10 // milliseconds to wait before actuating forward
#define DELAY_MID_CYCLE_MS 10 // milliseconds to wait before actuating in reverse

///////////////////////////////////////////////////////////////////////

#ifdef DEBUG
#ifdef SIMULATE
#define SDEBUG // simulate debug
#define d(...)     print(__VA_ARGS__)
#define dln(...) { print(__VA_ARGS__); print("\n"); }
#else
#define RDEBUG // real debug
#define d(...)   Serial.print(__VA_ARGS__)
#define dln(...) Serial.println(__VA_ARGS__)
#endif
#else
#define d(...)   (void)0
#define dln(...) (void)0
#endif

#endif // _CONFIG_H
