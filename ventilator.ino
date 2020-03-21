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
- add lcd display
- display to lcd
  - display rate 1-40
  - display volume 1-750
- resolve any warnings
- DONE add motor control
- DONE support l293d
- DONE read inputs
- DONE control motor based of inputs
- DONE only change values after a full cycle
- DONE detect how long actuation takes to calculate rate properly
- DONE support diff boards

*/

// leave on to get serial output
#define DEBUG
#define SERIAL_SPEED 9600

/////////////////////////////
////     PINS
/////////////////////////////
#define CYCLES_PER_MIN_PIN A0
#define ACTUATOR_DIST_PIN  A1
#define MOTOR_IN1_PIN       9
#define MOTOR_IN2_PIN      10
#define MOTOR_ENABLE_PIN   13


/////////////////////////////
////     VALUES
/////////////////////////////
#define MAX_CYCLES_PER_MIN 40
#define MIN_CYCLES_PER_MIN  1
#define DELAY_BEFORE_ACTUATING_MS 10
#define DELAY_AFTER_ACTUATING_MS  10
#define MAX_DUTY_CYCLE_PERCENT 10 // my linear actuator says 10%

// motor control speed (pwm value)
#define MIN_SPEED   0
#define MAX_SPEED 255

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

unsigned int cycles_input = 0, cycles_per_min = 0, actuator_input = 0, speed;
unsigned long start_time;
bool last_direction;

// milliseconds per cycle
#define CYCLE_MS (((long)60 * 1000) / cycles_per_min)

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

  // get initial values
  readInputs();
}

// read our values, drive motor, display output
void loop()
{
  // only read new values once we switch directions
  bool direction = DIRECTION;
  if (direction == 0 && last_direction != direction)
    readInputs();
  last_direction = direction;

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

  dln("read inputs");
}

// display to LCD
void display()
{
  int rate = analogMap(actuator_input, 0, 750);

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
  d(" cpm=");
  d(cycles_per_min);
  d(" rate=");
  dln(rate);
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
void driveMotor(int speed, bool direction)
{
  analogWrite(MOTOR_ENABLE_PIN, speed);
  digitalWrite(MOTOR_IN1_PIN, !direction);
  digitalWrite(MOTOR_IN2_PIN,  direction);
}
