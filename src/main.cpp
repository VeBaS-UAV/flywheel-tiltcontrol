#include <Arduino.h>

#if defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny45__)
const int PWM_INPUT_PIN = PB0;   // attiny PB0->leg 5
const int PWM_INPUT_IRQ_PIN = PCINT0; // Int 0 for pin 2 // attiny
const int PWM_OUTPUT_PIN = PB3;  // attiny PB3 -> leg 2
const int PWM_OUTPUT_REVERSE_PIN = PB4;  // attiny PB4 -> leg 3
const float PWM_FREQUENCY = 504; /* in Hz */
const float PWM_RESOLUTION = 8;  /* in bit */
const int LED_PIN = PB1;
#elif defined(ARDUINO_TEENSY31)
const int PWM_INPUT_PIN = 14; // teensy 3.2
const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
const int PWM_OUTPUT_PIN = 10;
const float PWM_FREQUENCY = 488; /* in Hz */
const float PWM_RESOLUTION = 8;  /* in bit */
const int LED_PIN = LED_BUILTIN;
#else // other platforms
const int PWM_INPUT_PIN = 14; // teensy 3.2
const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
const int PWM_OUTPUT_PIN = 10;
const float PWM_FREQUENCY = 488; /* in Hz */
const float PWM_RESOLUTION = 8;  /* in bit */
const int LED_PIN = LED_BUILTIN;
#endif

// #define DEBUG

// valid PWM range definition
const int PWM_LOW = 1100;    /* in us */
const int PWM_CENTER = 1500; /* in us */
const int PWM_HIGH = 1900;   /* in us */

// define hysteresis for center value
const int PWM_CTL_HYSTERESIS = 100; /* in us */

// PWM definition based in uc specs
// usually, 488Hz and 8bit resolution is default
// const float PWM_FREQUENCY = 488; /* in Hz */
// const float PWM_RESOLUTION = 8; /* in bit */
const int PWM_MAX_DUTY_IN_US = 1.0 / PWM_FREQUENCY * 1000 * 1000;
const float PWM_MAX_DUTY_VALUE = pow(2, PWM_RESOLUTION) - 1;

// delay on each iteration
const int LOOP_DELAY = 200; /* in ms */

int pwm_out_duty_in_us = PWM_CENTER; /* in us */
int mode_counter = 0;
int mode_out = 0;
const int MAX_MODE_COUNTER = 3;

int toggle_led = 0;

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

void setup() {

#ifdef DEBUG
  DEBUG_INIT;
#endif

  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_INPUT_PIN, INPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_REVERSE_PIN, OUTPUT);

  cli(); // clear global IRQ enable bit

  attachInterrupt(PWM_INPUT_IRQ_PIN, handle_pwm_input_interrupt, CHANGE);

  GIMSK |= (1 << PCIE); // set pin change interrupt
  PCMSK |= (1 << PWM_INPUT_IRQ_PIN); // set int bit

  sei(); // enable global IRQ enable bit

  DEBUG_PRINTLN("init completed");
}

int current_pwm_high_in_us() {
  if (pwm_input_duty_in_us > 0) {
    DEBUG_PRINT("Last time ");
    DEBUG_PRINTLN(pwm_input_duty_in_us);
    int result = pwm_input_duty_in_us;
    pwm_input_duty_in_us = 0;
    return result;
  }

  return 0;
}

// IRQ subroutine for arv irq handling
ISR(PCINT0_vect)
{
    handle_pwm_input_interrupt();

}

int current_input_mode() {
  int pwm_in_us = current_pwm_high_in_us();

  int pwm_c = pwm_in_us - PWM_CENTER;

  if (abs(pwm_c) < PWM_CTL_HYSTERESIS) {
    return 0;
  }

  if (pwm_c > 0) {
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

int set_pwm(int mode_out) {
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

  int delta_pwm = pwm - PWM_LOW;
  int pwm_inv = PWM_HIGH - delta_pwm;

  int pwm_out_duty_value_inv = convert_usec_to_duty_value(pwm_inv);

  DEBUG_PRINT("pwm duty value (inv): ");
  DEBUG_PRINTLN(pwm_out_duty_value_int);

  analogWrite(PWM_OUTPUT_REVERSE_PIN, pwm_out_duty_value_inv);

  return 0;
}

void loop() {

  int mode_in = current_input_mode();

  DEBUG_PRINT("mode_in: ");
  DEBUG_PRINTLN(mode_in);

  DEBUG_PRINT("mode_out: ");
  DEBUG_PRINTLN(mode_out);
  if (mode_in == 0) {
    mode_counter = MAX_MODE_COUNTER - 1;
  }

  if (mode_in != 0) {

    if (mode_in != mode_out) {
      mode_counter++;
    }

    DEBUG_PRINT("mode counter: ");
    DEBUG_PRINTLN(mode_counter);

    if (mode_counter > MAX_MODE_COUNTER) {

      int mode_delta = mode_in - mode_out;

      mode_out = min(max(mode_out + sign(mode_delta), -1), 1);
      mode_counter = 0;

      DEBUG_PRINT("mode out update: ");
      DEBUG_PRINTLN(mode_out);
    }
    set_pwm(mode_out);
  }

  DEBUG_PRINTLN("####");
  delay(LOOP_DELAY);
}
