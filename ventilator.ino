/*

Low cost ventilator based on Dr. Jeff Ebin's design

-samy kamkar
2020/03/20

BOM:
- MCU (widely optional, most Arduinos [Uno, Mega, Nano, etc], Teensy, etc)
- motor (linear actuator)
- motor driver (eg L293D)
- ...


TODO:
- should there be a delay before actuating?
- should there be a delay after actuating?
- support duty cycle of motor
- schematic
- DONE add lcd display
- DONE display to lcd
  - DONE display rate 1-40
  - DONE display distance 1-750 (changed to 255)
- resolve any warnings
- DONE add motor control
- DONE support l293d
- DONE read inputs
- DONE control motor based of inputs
- DONE only change values after a full cycle
- DONE detect how long actuation takes to calculate rate properly
- DONE support diff boards

*/

#define LCD // enable LCD
//#define DEBUG // enable serial debugging
#define SERIAL_SPEED 9600

/////////////////////////////
////     PINS
/////////////////////////////
#define CYCLES_PER_MIN_PIN A0
#define ACTUATOR_DIST_PIN  A1
#define MOTOR_IN1_PIN       9
#define MOTOR_IN2_PIN      10
#define MOTOR_ENABLE_PIN   13
#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4  5
#define LCD_D5  4
#define LCD_D6  3
#define LCD_D7  2

/////////////////////////////
////     VALUES
/////////////////////////////
#define MAX_CYCLES_PER_MIN 40
#define MIN_CYCLES_PER_MIN  1
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// motor control speed (pwm value)
#define MIN_SPEED   0
#define MAX_SPEED 255

// display values of (relative) distance
#define MIN_DISTANCE   0
#define MAX_DISTANCE 255

// analogRead range on analog pins
#define MIN_ANALOG_VALUE    0
#define MAX_ANALOG_VALUE 1023

///////////////////////////////////////////////////////////////////////

#ifdef DEBUG
#define d(...)   Serial.print(__VA_ARGS__)
#define dln(...) Serial.println(__VA_ARGS__)
#else
#define d(...)   (void)0
#define dln(...) (void)0
#endif

#ifdef LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#endif

#define BUFFER_SIZE (LCD_COLUMNS+1)
char buffer[BUFFER_SIZE];

uint16_t cycles_input = 0, cycles_per_min = 0, actuator_input = 0, speed = 0;
uint32_t start_time;
bool last_direction, direction;

// milliseconds per cycle
#define CYCLE_MS (((uint32_t)60 * 1000) / cycles_per_min)

// which side of the breath we're in
#define DIRECTION (((millis() - start_time) % CYCLE_MS) / (CYCLE_MS / 2))

void setup()
{
#ifdef DEBUG
  Serial.begin(SERIAL_SPEED);
#endif

  // pot pins
  pinMode(CYCLES_PER_MIN_PIN, INPUT);
  pinMode(ACTUATOR_DIST_PIN, INPUT);

  // motor pins
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

#ifdef LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
#endif

  lcdClear();
  lcdPrint("Ventilator On");

  // get initial values
  readInputs();
}

// read our values, drive motor, display output
void loop()
{
  // only read new values once we switch directions
  direction = DIRECTION;
  if (direction == 0 && last_direction != direction)
    readInputs();
  last_direction = direction;

  Serial.println("drive");
  driveMotor(speed, direction);

  display();
}

////////////////////////////////////////////////////
// helper functions

// grab values from pots and scale
void readInputs()
{
  start_time     = millis();
  actuator_input = analogRead(ACTUATOR_DIST_PIN);
  cycles_input   = analogRead(CYCLES_PER_MIN_PIN);
  cycles_per_min = analogMap(cycles_input, MIN_CYCLES_PER_MIN, MAX_CYCLES_PER_MIN);
  speed          = analogMap(actuator_input, MIN_SPEED, MAX_SPEED);

  lcdPrint(0, 1, "C");
  dln("read inputs");
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
  d(" speed=");
  d(speed);
  d(" cpm=");
  d(cycles_per_min);
  d(" tcpm=");
  d(tmp_cpm);
  d(" trate=");
  dln(tmp_rate);
}

// handle analog value mappings
long analogMap(long input, long to)
{
  return analogMap(input, 0, to);
}

long analogMap(long input, long from, long to)
{
  return map(input, MIN_ANALOG_VALUE, MAX_ANALOG_VALUE, from, to);
}

// drive our motor, L293D or similar
void driveMotor(uint8_t m_speed, bool m_direction)
{
  analogWrite(MOTOR_ENABLE_PIN, m_speed);
  digitalWrite(MOTOR_IN1_PIN, !m_direction);
  digitalWrite(MOTOR_IN2_PIN,  m_direction);
}

///////////////////////////////////////////////
// lcd helper functions

// clear lcd
void lcdClear()
{
#ifdef LCD
  lcd.clear();
#endif
}

// default to first column and row
void lcdPrint(char *buf)
{
#ifdef LCD
  lcdPrint(0, 0, buf);
#endif
}

// print to column and row
void lcdPrint(uint8_t col, uint8_t row, char *buf)
{
#ifdef LCD
  lcd.setCursor(col, row);
  lcd.print(buf);
#endif
}

// default to first column
void lcdPrint(uint8_t row, char *buf)
{
#ifdef LCD
  lcd.setCursor(0, row);
  lcd.print(buf);
#endif
}
