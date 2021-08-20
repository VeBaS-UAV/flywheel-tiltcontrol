#include <Arduino.h>

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
  #define _ATTINY_
  const int PWM_INPUT_PIN = PB0;   // attiny PB0->leg 5
  const int PWM_INPUT_IRQ_PIN = PCINT0; // Int 0 for pin 2 // attiny
  const int PWM_OUTPUT_PIN = PB4;  // attiny PB3 -> leg 2
  const int PWM_OUTPUT_REVERSE_PIN = PB3;  // attiny PB4 -> leg 3
  // const float PWM_FREQUENCY = 504; /* in Hz */
  // const float PWM_RESOLUTION = 8;  /* in bit */
  const int LED_PIN = PB1;
// #define DEBUG
#elif defined(ARDUINO_TEENSY31)
#define _TEENSY_
  const int PWM_INPUT_PIN = 14; // teensy 3.2
  const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
  const int PWM_OUTPUT_PIN = 10;
  // const float PWM_FREQUENCY = 488; /* in Hz */
  // const float PWM_RESOLUTION = 8;  /* in bit */
  // const int LED_PIN = LED_BUILTIN;
// #define DEBUG
#else // other platforms
  #define _ARUINO_
  const int PWM_INPUT_PIN = 14; // teensy 3.2
  const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
  const int PWM_OUTPUT_PIN = 10;
  const int PWM_OUTPUT_REVERSE_PIN = PB3;  // attiny PB4 -> leg 3
  // const float PWM_FREQUENCY = 488; /* in Hz */
  // const float PWM_RESOLUTION = 8;  /* in bit */
  // const int LED_PIN = LED_BUILTIN;
// #define DEBUG
#endif

// valid PWM range definition
const int PWM_LOWER_BOUND = 1100;    /* in us */
const int PWM_CENTER = 1500; /* in us */
const int PWM_UPPER_BOUND = 1900;   /* in us */

// define hysteresis for center value
const int PWM_CTL_HYSTERESIS = 100; /* in us */

// delay on each iteration
const int LOOP_DELAY_IN_MS = 20; /* in ms */
// time between mode steps
const int LONG_PRESS_TIME_IN_MS = 500; /* in ms */

// state variables
int pwm_output_pulse_width_in_us = PWM_CENTER; /* in us */
unsigned long mode_last_changed_timestamp_in_ms = 0;
int mode_out = 0;


// last pwm duty time
volatile unsigned long pwm_input_pulse_width_in_us = 0;
volatile unsigned long pwm_input_pulse_start_in_us = 0;

// #define sign(x) (x==0 ? 0 : (x>1 ? 1 : -1))
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
    pwm_input_pulse_start_in_us = micros();
  } else {
    pwm_input_pulse_width_in_us = micros() - pwm_input_pulse_start_in_us;

    // handle overflow after 70min
    if (pwm_input_pulse_width_in_us < 0)
    {
      pwm_input_pulse_width_in_us = 0;
    }

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

  pinMode(PWM_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_REVERSE_PIN, OUTPUT);

  mode_last_changed_timestamp_in_ms = millis();
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
int get_pwm_duty_time_in_us() {

  // reset when last edge is long ago...(1sec)
  if (pwm_input_pulse_start_in_us + 1000L*1000L < micros())
  {
    // pwm_input_pulse_width_in_us = 0;
  }

  return pwm_input_pulse_width_in_us;
}

// return [-1,0,1] depending on the pwm input value
int current_input_mode() {
  int pwm_in_us = get_pwm_duty_time_in_us();

  int pwm_deflection = pwm_in_us - PWM_CENTER;

  if (abs(pwm_deflection) < PWM_CTL_HYSTERESIS) {
    return 0;
  }

  if (pwm_deflection > 0) {
    return 1;
  }

  return -1;
}

int sign(int x) {
  if (x > 0)
    return 1;
  if (x < 0)
    return -1;
  return 0;
}

void do_soft_pwm(int pin, int high_time_in_us)
{
  unsigned long start_in_us = micros();

  if (high_time_in_us < PWM_LOWER_BOUND || high_time_in_us > PWM_UPPER_BOUND)
  {
    return;
  }

  digitalWrite(pin, HIGH);
  while(start_in_us + high_time_in_us > micros())
  {
    // do nothing
  }
  digitalWrite(pin, LOW);
}

int set_output_mode(int mode_out) {
  int pwm_width_in_us = PWM_CENTER;

  if (mode_out > 0) {
    pwm_width_in_us = PWM_UPPER_BOUND;
  }
  if (mode_out < 0) {
    pwm_width_in_us = PWM_LOWER_BOUND;
  }

  DEBUG_PRINT("pwm: ");
  DEBUG_PRINTLN(pwm_width_in_us);

  do_soft_pwm(PWM_OUTPUT_PIN, pwm_width_in_us);
  // calculate the reverse PWM value

  int pwm_width_reversed_in_us = PWM_UPPER_BOUND - (pwm_width_in_us - PWM_LOWER_BOUND);
  do_soft_pwm(PWM_OUTPUT_REVERSE_PIN, pwm_width_reversed_in_us);

  return 0;

}

void loop() {

  int mode_input = current_input_mode();

  DEBUG_PRINT("mode_in: ");
  DEBUG_PRINTLN(mode_input);

  DEBUG_PRINT("mode_out: ");
  DEBUG_PRINTLN(mode_out);

  if (mode_input == 0)
  {
    mode_last_changed_timestamp_in_ms = millis();
  }

  long input_mode_diff_timedelta_in_ms = millis() - mode_last_changed_timestamp_in_ms;
  // if flywheel is activated
  if (input_mode_diff_timedelta_in_ms > LONG_PRESS_TIME_IN_MS) {

      int mode_delta = mode_input  - mode_out;

      mode_out = min(max(mode_out + sign(mode_delta), -1), 1);

      DEBUG_PRINT("mode out update: ");
      DEBUG_PRINTLN(mode_out);

      mode_last_changed_timestamp_in_ms = millis();
  }

  set_output_mode(mode_out);

  DEBUG_PRINTLN("####");
  delay(LOOP_DELAY_IN_MS);
}
