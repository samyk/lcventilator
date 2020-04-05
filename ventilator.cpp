/*

Low cost ventilator based on Dr. Jeffrey Ebin's design.

Supports most low cost MCUs (Arduino Uno, Nano, Mega, Teensy, etc)
- optional LCD or I2C LCD
- 2 pots required to control rate + distance

-samy kamkar
2020/03/20

TODO:
- support diff delay between forward and reverse actuating
- support duty cycle of motor
- schematic
- resolve any warnings

to simulate:
  g++ -DSIMULATE *.cpp -o ventilator

*/

// change settings in config.h
#include "config.h"
#include "ventilator.h"
#include "simulate.h"

///////////////////////////////////////////////////////////////////////

#if defined(LCD) && defined(LCD_I2C)
#error "define either LCD, LCD_I2C, or neither"
#endif

#if defined(LCD)
#define _LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

#elif defined(LCD_I2C)
#define _LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);
#endif

#define BUFFER_SIZE (LCD_COLUMNS+1)
char buffer[BUFFER_SIZE];

uint16_t cycles_input = 0, cycles_per_min = 0, actuator_input = 0, speed = 0;
uint32_t start_time;
bool last_direction, direction;

// milliseconds per cycle (not including pre/post delay times)
#define MS_IN_MIN ((uint32_t)60 * 1000)
#define TOTAL_DELAY_MS (DELAY_PRE_CYCLE_MS + DELAY_MID_CYCLE_MS)
#define TOTAL_DELAY_MS_PER_MIN (cycles_per_min * TOTAL_DELAY_MS)
#define CYCLE_MS ((MS_IN_MIN - TOTAL_DELAY_MS_PER_MIN) / cycles_per_min)
#define CURRENT_TIME_IN_CYCLE ((millis() - start_time) % CYCLE_MS)

// which side of the breath we're in
#define PERCENT_OF(X, PERCENT) (X / (100.0 / PERCENT))
#define CYCLE_TIME_IN (DELAY_PRE_CYCLE_MS + PERCENT_OF(CYCLE_MS, PERCENTAGE_GOING_IN))
#define DIRECTION (CURRENT_TIME_IN_CYCLE >= CYCLE_TIME_IN)
#define CURRENT_SPEED (PERCENT_OF(speed, (direction ? PERCENTAGE_GOING_IN : (100 - PERCENTAGE_GOING_IN))))
#define BREATH_IN  0
#define BREATH_OUT 1

///////////////////////////////////////////////
// lcd helper functions

// clear lcd
void lcdClear()
{
#ifdef _LCD
  lcd.clear();
#endif
}

// print to column and row
void lcdPrint(uint8_t col, uint8_t row, char *buf)
{
#ifdef _LCD
  lcd.setCursor(col, row);
  lcd.print(buf);
#endif
}

// default to first column
void lcdPrint(uint8_t row, char *buf)
{
#ifdef _LCD
  lcd.setCursor(0, row);
  lcd.print(buf);
#endif
}

// default to first column and row
void lcdPrint(char *buf)
{
#ifdef _LCD
  lcdPrint(0, 0, buf);
#endif
}

////////////////////////////////////////////////////
// other helper functions

// handle analog value mappings
long analogMap(long input, long from, long to)
{
  return map(input, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, from, to);
}

long analogMap(long input, long to)
{
  return analogMap(input, 0, to);
}

// drive our motor, L293D or similar
void driveMotor(uint8_t m_speed, bool m_direction)
{
  d("driving motor speed=");
  d(m_speed);
  d(" direction=");
  dln(direction);

  analogWrite(MOTOR_ENABLE_PIN, m_speed);
  digitalWrite(MOTOR_IN1_PIN, !m_direction);
  digitalWrite(MOTOR_IN2_PIN,  m_direction);
}

// stop motor
void stopMotor()
{
  d("stop motor");
  analogWrite(MOTOR_ENABLE_PIN, 0);
}

// grab values from pots and scale
void readInputs()
{
  start_time     = millis();
  actuator_input = analogRead(ACTUATOR_DIST_PIN);
  cycles_input   = analogRead(CYCLES_PER_MIN_PIN);
  cycles_per_min = analogMap(cycles_input, MIN_CYCLES_PER_MIN, MAX_CYCLES_PER_MIN);
  speed          = analogMap(actuator_input, MIN_SPEED, MAX_SPEED);

  dln("read inputs");
  lcdPrint(0, 1, "C");
}

// display to LCD
void display()
{
  // read from analog pins because normally we only change values on breath
  uint8_t tmp_cpm  = analogMap(analogRead(CYCLES_PER_MIN_PIN), MIN_CYCLES_PER_MIN, MAX_CYCLES_PER_MIN);
  uint8_t tmp_rate = analogMap(analogRead(ACTUATOR_DIST_PIN),  MIN_DISTANCE, MAX_DISTANCE);

  // print to display
  snprintf(buffer, BUFFER_SIZE, "%c Breath/min: %2d", direction ? '+' : '-', tmp_cpm);
  lcdPrint(buffer);

  snprintf(buffer, BUFFER_SIZE, "  Distance:  %3d", tmp_rate);
  lcdPrint(1, buffer);

  // debug output to serial (if DEBUG enabled)
  d(millis());
  d(" sec=");
  d(int(millis()/1000));
  d(" cycles=");
  d(cycles_input);
  d(" actuator=");
  d(actuator_input);
  d(" CYCLE_MS=");
  d(CYCLE_MS);
  d(" DIR=");
  d(DIRECTION);
  d(" dir=");
  d(direction);
  d(" speed=");
  d(speed);
  d(" curspeed=");
  d(CURRENT_SPEED);
  d(" PGI=");
  d(PERCENTAGE_GOING_IN);
  d(" cpm=");
  d(cycles_per_min);
  d(" tcpm=");
  d(tmp_cpm);
  d(" trate=");
  dln(tmp_rate);
}


//////////////////////////////////////////////////
// main
void setup()
{
#ifdef RDEBUG
  Serial.begin(SERIAL_SPEED);
#endif

  // pot pins
  pinMode(CYCLES_PER_MIN_PIN, INPUT);
  pinMode(ACTUATOR_DIST_PIN, INPUT);

  // motor pins
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

#ifdef _LCD
  // try to support LiquidCrystal_I2C libs from F Malpartida, Frank de Brabander, Marco Schwartz, ejoyneering, Tony Kambourakis (platformio 136, 576, 1574, 6158)
#if defined(LiquidCrystal_i2c_h) // if using 6158 by ejoyneering
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
#elif defined(LiquidCrystal_4bit_h) // 136
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
#else
  lcd.begin(LCD_COLUMNS, LCD_ROWS, LCD_5x8DOTS);
#endif

#endif

  lcdClear();
  lcdPrint("Ventilator On");
  delay(2000);

  // get initial values
  readInputs();
}

// read our values, drive motor, display output
void loop()
{
  // cache our direction
  direction = DIRECTION;

  // only read new values once we switch directions
  if (last_direction != direction)
  {
    // begin cycle
    if (direction == BREATH_IN)
    {
      readInputs();
#if DELAY_PRE_CYCLE_MS
      stopMotor();
      delay(DELAY_PRE_CYCLE_MS);
#endif
    }

    // return from other half cycle
    else if (direction == BREATH_OUT)
    {
#if DELAY_MID_CYCLE_MS
      stopMotor();
      delay(DELAY_MID_CYCLE_MS);
#endif
    }
  }
  last_direction = direction;

  driveMotor(CURRENT_SPEED, direction);

  display();
}
