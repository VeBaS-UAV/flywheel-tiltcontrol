#include <Arduino.h>

const int LED_PIN = LED_BUILTIN;

// input PIN for PWM input signal
//const int PWM_INPUT_PIN = 2; //attiny
const int PWM_INPUT_PIN = 14; // teensy 3.2

//need to be set on attiny manually
const int PWM_INPUT_IRQ_PIN = digitalPinToInterrupt(PWM_INPUT_PIN); // teensy
//const int PWM_INPUT_IRQ_PIN = 0; //Int 0 for pin 2 // attiny

// output PIN for PWM signal
//const int PWM_OUTPUT_PIN = 1; //attiny
const int PWM_OUTPUT_PIN = 10;

// valid PWM range definition
const int PWM_LOW = 1100; /* in us */
const int PWM_CENTER = 1500; /* in us */
const int PWM_HIGH = 2000; /* in us */

// define hysteresis for center value
const int PWM_CTL_HYSTERESIS = 50; /* in us */

// P control value for PWM output based on PWM input-output error
// I and D control not implemented
//const float PWM_CTL_P = 0.01f;  /* control factor for PWM control in 1/1000 steps*/

// PWM definition based in uc specs
// usually, 488Hz and 8bit resolution is default
const float PWM_FREQUENCY = 488; /* in Hz */
const float PWM_RESOLUTION = 8; /* in bit */
const int PWM_MAX_DUTY_IN_US = 1.0/PWM_FREQUENCY*1000*1000;
const float PWM_MAX_DUTY_VALUE = pow(2, PWM_RESOLUTION) - 1;

// delay on each iteration
const int LOOP_DELAY = 200; /* in ms */

int pwm_out_duty_in_us = PWM_CENTER; /* in us */
int mode_counter = 0;
int mode_out = 0;
const int MAX_MODE_COUNTER = 5;

int toggle_led = 0;

// last pwm duty time
volatile unsigned long pwm_input_duty_in_us = 0;
volatile unsigned long pwm_start_time = 0;


// enable/disable debug message over serial port
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// interrupt routing to calculate the duty length in us
void handle_pwm_input_interrupt() {

  if (digitalRead(PWM_INPUT_PIN))
  {
    pwm_start_time = micros();
  } else {
    pwm_input_duty_in_us = micros() - pwm_start_time;
  }

}

void setup() {

#ifdef DEBUG
    Serial.begin(115200);
#endif

  pinMode(LED_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  pinMode(PWM_INPUT_PIN, INPUT);

  attachInterrupt(PWM_INPUT_IRQ_PIN, handle_pwm_input_interrupt, CHANGE);

  DEBUG_PRINTLN("init completed");

}

int current_pwm_high_in_us()
{
  if (pwm_input_duty_in_us >0)
  {
    DEBUG_PRINT("Last time ");
    DEBUG_PRINTLN(pwm_input_duty_in_us);
    int result = pwm_input_duty_in_us;
    pwm_input_duty_in_us = 0;
    return result;
  }
  
  return 0;
}

int current_input_mode()
{
  int pwm_in_us = current_pwm_high_in_us();

  int pwm_c = pwm_in_us - PWM_CENTER;

  if (abs(pwm_c) < PWM_CTL_HYSTERESIS)
  {
    return 0;
  }

  if (pwm_c > 0)
  {
    return 1;
  }

  return -1;

}

// calculate time in us to duty value based on PWM frequency and resolution
int convert_usec_to_duty_value(int duty_time_in_us)
{
    return map(duty_time_in_us, 0, PWM_MAX_DUTY_IN_US, 0, PWM_MAX_DUTY_VALUE);
}

int sign(int x)
{
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

int set_pwm(int mode_out)
{
  int pwm = PWM_CENTER;

  if (mode_out > 0)
  {
    pwm =  PWM_HIGH;
  }
  if (mode_out < 0)
  {
    pwm =  PWM_LOW;
  }

  DEBUG_PRINT("pwm: ");
  DEBUG_PRINTLN(pwm);

  int pwm_out_duty_value = convert_usec_to_duty_value(pwm);

  DEBUG_PRINT("pwm duty value: ");
  DEBUG_PRINTLN(pwm_out_duty_value);
  analogWrite(PWM_OUTPUT_PIN, pwm_out_duty_value);

  return 0;
}


void loop() {

  digitalWrite(LED_PIN, toggle_led);
  toggle_led = !toggle_led;

  // get current pwm value
  //int pwm_in_duty_in_us = current_pwm_high_in_us();

  int mode_in = current_input_mode();

  DEBUG_PRINT("mode_in: ");
  DEBUG_PRINTLN(mode_in);

  if (mode_in == 0)
  {
    mode_counter = MAX_MODE_COUNTER - 1;
  }

  if (mode_in != 0)
  {

    DEBUG_PRINT("mode_out: ");
    DEBUG_PRINTLN(mode_out);

    if (mode_in != mode_out)
    {
      mode_counter++;
    }

    DEBUG_PRINT("mode counter: ");
    DEBUG_PRINTLN(mode_counter);

    if(mode_counter > MAX_MODE_COUNTER)
    {

      int mode_delta = mode_in - mode_out;

      mode_out = min(max(mode_out + sign(mode_delta),-1), 1);
      mode_counter = 0;

      DEBUG_PRINT("mode out update: ");
      DEBUG_PRINTLN(mode_out);

    }
    set_pwm(mode_out);

  }

  DEBUG_PRINTLN("####");
  delay(LOOP_DELAY);
  
}
