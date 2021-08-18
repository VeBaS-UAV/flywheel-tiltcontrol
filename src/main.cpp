#include <Arduino.h>

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
  #define _ATTINY_
  const int PWM_INPUT_PIN = PB0;   // attiny PB0->leg 5
  const int PWM_INPUT_IRQ_PIN = PCINT0; // Int 0 for pin 2 // attiny
  const int PWM_OUTPUT_PIN = PB3;  // attiny PB3 -> leg 2
  const int PWM_OUTPUT_REVERSE_PIN = PB4;  // attiny PB4 -> leg 3
  const float PWM_FREQUENCY = 504; /* in Hz */
  const float PWM_RESOLUTION = 8;  /* in bit */
  const int LED_PIN = PB1;
// #define DEBUG
#elif defined(ARDUINO_TEENSY31)
#define _TEENSY_
  const int PWM_INPUT_PIN = 14; // teensy 3.2
  const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
  const int PWM_OUTPUT_PIN = 10;
  const float PWM_FREQUENCY = 488; /* in Hz */
  const float PWM_RESOLUTION = 8;  /* in bit */
  // const int LED_PIN = LED_BUILTIN;
// #define DEBUG
#else // other platforms
  #define _ARUINO_
  const int PWM_INPUT_PIN = 14; // teensy 3.2
  const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
  const int PWM_OUTPUT_PIN = 10;
  const float PWM_FREQUENCY = 488; /* in Hz */
  const float PWM_RESOLUTION = 8;  /* in bit */
  // const int LED_PIN = LED_BUILTIN;
// #define DEBUG
#endif

// valid PWM range definition
const int PWM_LOW = 1100;    /* in us */
const int PWM_CENTER = 1500; /* in us */
const int PWM_HIGH = 1900;   /* in us */

// define hysteresis for center value
const int PWM_CTL_HYSTERESIS = 100; /* in us */

// PWM definition based in uc specs
const int PWM_MAX_DUTY_IN_US = 1.0 / PWM_FREQUENCY * 1000 * 1000;
const float PWM_MAX_DUTY_VALUE = pow(2, PWM_RESOLUTION) - 1;


// state variables
int pwm_out_duty_in_us = PWM_CENTER; /* in us */
int mode_out = 0;
long mode_last_changed = 0;

// delay on each iteration
const int LOOP_DELAY = 200; /* in ms */
// used as delay method, MAX_MODE_COUNTER x LOOP DELAY = time for mode step
// const int MAX_MODE_COUNTER = 3;

// time between mode steps
const int MODE_STEP_TIME = 500; /* in ms */

// last pwm duty time
volatile unsigned long pwm_input_duty_in_us = 0;
volatile unsigned long pwm_start_time = 0;

// enable/disable debug message over serial port

#ifdef DEBUG
#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
#include <DigiKeyboard.h>
#define DEBUG_INIT // DigiKeyboard.begin()
#define DEBUG_PRINT(x) DigiKeyboard.print(x)
#define DEBUG_PRINTLN(x) DigiKeyboard.println(x)
#define DEBUG_FLUSH // DigiKeyboard.refresh();
#else
#define DEBUG_INIT Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_FLUSH // DigiKeyboard.refresh();
#endif
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// interrupt routing to calculate the duty length in us
void handle_pwm_input_interrupt() {

  if (digitalRead(PWM_INPUT_PIN)) {
    pwm_start_time = micros();
  } else {
    pwm_input_duty_in_us = micros() - pwm_start_time;
  }
}

#ifdef _ATTINY_
// IRQ subroutine for arv irq handling
ISR(PCINT0_vect)
{
    handle_pwm_input_interrupt();
}
#endif

void setup() {

#ifdef DEBUG
  DEBUG_INIT;
#endif

  // pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  // pinMode(PWM_OUTPUT_REVERSE_PIN, OUTPUT);

  mode_last_changed = millis();
#ifdef _ARDUINO_
  attachInterrupt(PWM_INPUT_IRQ_PIN, handle_pwm_input_interrupt, CHANGE);
#elif defined _ATTINY_
  cli();

  GIMSK |= (1 << PCIE);
  PCMSK |= (1 << PWM_INPUT_IRQ_PIN); // set int bit

  sei();
#endif

  DEBUG_PRINTLN("init completed");
}


// return the pwm value based on the irq handler and reset it to 0
int get_pwm_duty_time_in_us_and_reset() {
  if (pwm_input_duty_in_us > 0) {
    DEBUG_PRINT("Last time ");
    DEBUG_PRINTLN(pwm_input_duty_in_us);
    int result = pwm_input_duty_in_us;
    pwm_input_duty_in_us = 0;
    return result;
  }

  return 0;
}

// return [-1,0,1] depending on the pwm input value
int current_input_mode() {
  int pwm_in_us = get_pwm_duty_time_in_us_and_reset();

  int pwm_deflection = pwm_in_us - PWM_CENTER;

  if (abs(pwm_deflection) < PWM_CTL_HYSTERESIS) {
    return 0;
  }

  if (pwm_deflection > 0) {
    return 1;
  }

  return -1;
}

// calculate time in us to duty value based on PWM frequency and resolution
int convert_usec_to_duty_value(int duty_time_in_us) {
  return map(duty_time_in_us, 0, PWM_MAX_DUTY_IN_US, 0, PWM_MAX_DUTY_VALUE);
}

int sign(int x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}

int set_output_mode(int mode_out) {
  int pwm = PWM_CENTER;

  if (mode_out > 0) {
    pwm = PWM_HIGH;
  }
  if (mode_out < 0) {
    pwm = PWM_LOW;
  }

  DEBUG_PRINT("pwm: ");
  DEBUG_PRINTLN(pwm);

  int pwm_out_duty_value = convert_usec_to_duty_value(pwm);

  DEBUG_PRINT("pwm duty value: ");
  DEBUG_PRINTLN(pwm_out_duty_value);
  analogWrite(PWM_OUTPUT_PIN, pwm_out_duty_value);
/*
  // calculate the reverse PWM value
  int delta_pwm = pwm - PWM_LOW;
  int pwm_inv = PWM_HIGH - delta_pwm;

  int pwm_out_duty_value_inv = convert_usec_to_duty_value(pwm_inv);

  DEBUG_PRINT("pwm duty value (inv): ");
  DEBUG_PRINTLN(pwm_out_duty_value_int);

  analogWrite(PWM_OUTPUT_REVERSE_PIN, pwm_out_duty_value_inv);
*/
  return 0;
}


void loop() {

  int mode_in = current_input_mode();

  DEBUG_PRINT("mode_in: ");
  DEBUG_PRINTLN(mode_in);

  DEBUG_PRINT("mode_out: ");
  DEBUG_PRINTLN(mode_out);

  // if flywheel is activated
  if (mode_in != 0) {

    long delta_t = millis() - mode_last_changed;

    if (delta_t > MODE_STEP_TIME) {

      int mode_delta = mode_in - mode_out;

      mode_out = min(max(mode_out + sign(mode_delta), -1), 1);

      DEBUG_PRINT("mode out update: ");
      DEBUG_PRINTLN(mode_out);

      mode_last_changed = millis();
    }

    set_output_mode(mode_out);
  } else {
    mode_last_changed = millis();
  }

  DEBUG_PRINTLN("####");
  delay(LOOP_DELAY);
}
