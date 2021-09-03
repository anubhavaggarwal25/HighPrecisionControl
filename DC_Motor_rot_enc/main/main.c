#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "rotary_encoder.h"
#include "driver/pcnt.h"

static const char *TAG = "Simple_DC_Motor_Control";

#define GPIO_PWM0A_OUT 19   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 18   //Set GPIO 16 as PWM0B

#define ROT_ENC_A_GPIO 34   // set GPIO 34 as Rotary Encoder GPIO A
#define ROT_ENC_B_GPIO 35   // set GPIO 35 as Rotary Encoder GPIO B

#define ENABLE_HALF_STEPS false  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0      // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION    false  // Set to true to reverse the clockwise/counterclockwise sense

#define ENC_COUNT_REV 110   // Motor encoder output pulses per 360 degree revolution (Manually measured )
#define MAX_RPM 600 // Maximum Speed of motor

#define PCNT_H_LIM_VAL 20000
#define PCNT_L_LIM_VAL -20000
#define PCNT_THRESH1_VAL 5
#define PCNT_THRESH0_VAL -5
#define PCNT_INPUT_SIG_IO   34  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  35  // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO      33 // Output GPIO of a sample 1 Hz pulse generator

#define KP 0
#define KI 0.000005
#define KD 0.00015
#define MAX_CONTROL_VALUE 100

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
/*
  Do Not Change these Parameters
*/
int previous_value = 0;       // previous pulse counts
int Current_value = 0;        // current pulse counts
int wheel_pulse_count = 0;    // pulse counts in 1 second
float current_rpm_value = 0;  // current speed in rpm
float previous_duty = 0.0;    // previous duty cycle for PID

float current_duty = 0.0;     // Current duty cycle for PID
float filtered_duty = 0.0;    // duty after applying PID
float error = 0.0;            // error in duty cycle
float Integral_value = 0.0;   // changes in Integral Value
float control_value = 0.0;    // Changes in control value
float Derivative = 0.0;       // changes in derivative value
bool stop_motor = false;      // flag to stop motot

/*
  Change these parameters according to your needs
*/
float init_duty = 30.0;       // initial duty cycle
int revtotake = 10;           // revolutions you want to take

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    int pcnt_unit = (int)arg;
    pcnt_evt_t evt;
    evt.unit = pcnt_unit;
    /* Save the PCNT event type that caused an interrupt
       to pass it to the main program */
    pcnt_get_event_status(pcnt_unit, &evt.status);
    xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);
}

/* Configure LED PWM Controller
 * to output sample pulses at 1 Hz with duty of about 10%
 */
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_10_BIT;
    ledc_timer.freq_hz          = 1;  // set output frequency at 1 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = LEDC_OUTPUT_IO;
    ledc_channel.duty       = 100; // set duty at about 10%
    ledc_channel.hpoint     = 0;
    ledc_channel_config(&ledc_channel);
}

static void pcnt_configuration(int unit)
{
  /* Prepare configuration for the PCNT unit */
  pcnt_config_t pcnt_config = {
    // Set PCNT input signal and control GPIOs
    .pulse_gpio_num = PCNT_INPUT_SIG_IO,
    .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
    .channel = PCNT_CHANNEL_0,
    .unit = unit,
    // What to do on the positive / negative edge of pulse input?
    .pos_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
    .neg_mode = PCNT_COUNT_INC,   // Count up on the positive edge
    // What to do when control input is low or high?
    .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
    .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
    // Set the maximum and minimum limit values to watch
    .counter_h_lim = PCNT_H_LIM_VAL,
    .counter_l_lim = PCNT_L_LIM_VAL,
  };
  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_config);

  /* Configure and enable the input filter */
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);

  /* Set threshold 0 and 1 values and enable events to watch */
  pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
  pcnt_event_enable(unit, PCNT_EVT_THRES_1);
  pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
  pcnt_event_enable(unit, PCNT_EVT_THRES_0);
  /* Enable events on zero, maximum and minimum limit values */
  pcnt_event_enable(unit, PCNT_EVT_ZERO);
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit, PCNT_EVT_L_LIM);

  /* Initialize PCNT's counter */
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  /* Install interrupt service and add isr callback handler */
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

  /* Everything is set up, now go to counting */
  pcnt_counter_resume(unit);
}

// Execute the pusle counter to count the no. of ticks in 1 sec
static void pulse_counter_execute(void *arg){
  int pcnt_unit = PCNT_UNIT_0;

  /* Initialize LEDC to generate sample pulse signal */
  ledc_init();

  /* Initialize PCNT event queue and PCNT functions */
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
  pcnt_configuration(pcnt_unit);

  int16_t count = 0;
  pcnt_evt_t evt;
  portBASE_TYPE res;
  while (1) {
      /* Wait for the event information passed from PCNT's interrupt handler.
       * Once received, decode the event type and print it on the serial monitor.
       */
      res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
      if (res == pdTRUE) {
          pcnt_get_counter_value(pcnt_unit, &count);
          /*
            RPM Calculation
          */
          previous_value = Current_value;
          Current_value = count;
          wheel_pulse_count = Current_value - previous_value;
          current_rpm_value = (float) wheel_pulse_count * 60 / ENC_COUNT_REV;
          previous_duty = current_duty;
          current_duty = current_rpm_value * 100.0 / (float) MAX_RPM;
          //ESP_LOGI(TAG, "Pulse Count: %d, Current Speed: %f RPM, Current Duty Cycle: %f, Filtered Duty: %f", wheel_pulse_count, current_rpm_value, current_duty, filtered_duty);
          ESP_LOGI(TAG, "RPM value: %f, current_duty = %f, PID Duty: %f", current_rpm_value, current_duty, filtered_duty);
          if (evt.status & PCNT_EVT_THRES_1) {
              ESP_LOGI(TAG, "THRES1 EVT");
          }
          if (evt.status & PCNT_EVT_THRES_0) {
              ESP_LOGI(TAG, "THRES0 EVT");
          }
          if (evt.status & PCNT_EVT_L_LIM) {
              ESP_LOGI(TAG, "L_LIM EVT");
          }
          if (evt.status & PCNT_EVT_H_LIM) {
              ESP_LOGI(TAG, "H_LIM EVT");
          }
          if (evt.status & PCNT_EVT_ZERO) {
              ESP_LOGI(TAG, "ZERO EVT");
          }
      } else {
          pcnt_get_counter_value(pcnt_unit, &count);
          /*
            RPM Calculation
          */
          previous_value = Current_value;
          Current_value = count;
          wheel_pulse_count = Current_value - previous_value;
          current_rpm_value = (float) wheel_pulse_count * 60 / ENC_COUNT_REV;
          previous_duty = current_duty;
          current_duty = current_rpm_value * 100.0 / (float) MAX_RPM;
          ESP_LOGI(TAG, "RPM value: %f, current_duty = %f, PID Duty: %f", current_rpm_value, current_duty, filtered_duty);
      }
  }
}

// PID control loop
static void PID_execute(void *args){
  while(1){
    error = init_duty - current_duty;
    Integral_value += KI * error * 10;
    if(Integral_value > 1.0){
      Integral_value = 1.0;
    }
    if(Integral_value < -1.0){
      Integral_value = -1.0;
    }

    Derivative = (current_duty - previous_duty) / 10;

    control_value = (KP * error) + Integral_value - (KD * Derivative);

    if(control_value > 1.0){
      Integral_value = 1.0;
    }
    if(control_value < -1.0){
      Integral_value = -1.0;
    }
    filtered_duty = (float)(control_value * MAX_CONTROL_VALUE);
    vTaskDelay (2);   // Delay 20 ms = 2 Ticks
  }
}

/**
 GPIO initialization
*/
static void mcpwm_example_gpio_initialize(void){
  printf("initialize mcpwm gpio...\n");
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/*
  Motor moves in forward direction, with duty cycle = duty %
*/
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle){
  /*printf("Turing motor forward... \n");*/
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
  mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
}

/*
  Motor moves in backward direction, with duty cycle = duty %
*/
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle){
  /*printf("Turing motor backward... \n");*/
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
  mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
}

static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num){
  //printf("Motor Stopped \n");
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

static void rotary_encoder_execute(void *arg){
  int current_position = 0;
  int previous_position = 0;
  int step_difference = 0;
  /*
  Rotary Encoder Configuration
  */
  // esp32-rotary-encoder requires that the GPIO ISR service is installed before calling rotary_encoder_register()
  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  // Initialise the rotary encoder device with the GPIOs for A and B signals
  rotary_encoder_info_t info = { 0 };
  ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
  ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
#ifdef FLIP_DIRECTION
  ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
#endif

  // Create a queue for events from the rotary encoder driver.
  // Tasks can read from this queue to receive up to date position information.
  QueueHandle_t event_queue = rotary_encoder_create_queue();
  ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));

  while (1)
  {
      // Wait for incoming events on the event queue.
      rotary_encoder_event_t event = { 0 };
      if (xQueueReceive(event_queue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE)
      {
        previous_position = current_position;
        current_position = event.state.position;
        step_difference = current_position - previous_position;
        ESP_LOGI(TAG, "Event: position %d, direction %s, difference %d", event.state.position,
                     event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "clockwise" : "Anti-Clockwise") : "Not set",
                   current_position - previous_position);
        if(((event.state.position)/ENC_COUNT_REV) >= revtotake && revtotake > 0){
          printf("stopping motor\n");
          stop_motor = true;
        }
        else if(((event.state.position )/ENC_COUNT_REV) <= revtotake && revtotake < 0){
          printf("stopping motor\n");
          stop_motor = true;
        }
      }
      else
      {
          // Poll current position and direction
          rotary_encoder_state_t state = { 0 };
          ESP_ERROR_CHECK(rotary_encoder_get_state(&info, &state));
          ESP_LOGI(TAG, "Poll: position %d, direction %s, , difference %d", state.position,
                   state.direction ? (state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "clockwise" : "Anti-Clockwise") : "Not Set",
                 current_position - previous_position);


          // Reset the device
          if (RESET_AT && (state.position >= RESET_AT || state.position <= -RESET_AT))
          {
              ESP_LOGI(TAG, "Reseting the Device");
              ESP_ERROR_CHECK(rotary_encoder_reset(&info));
          }
      }
  }
  ESP_LOGE(TAG, "queue receive failed");

  ESP_ERROR_CHECK(rotary_encoder_uninit(&info));
}

static void mcpwm_execute(void *arg){
  /*
    DC motor configurations
  */
  // 1. initialization of the GPIOs
  mcpwm_example_gpio_initialize();

  //2. Initialize mcpwm configuration
  printf("Configuring Initial Parameters of mcpwm... \n");
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;  // frequency = 1000 Hz
  pwm_config.cmpr_a = 0;  // duty cycle for PWMxA
  pwm_config.cmpr_b = 0;  // duty cycle for PWMxB
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // configure PWM0A & PWM0B with above settings

  while(1){
    // Move Clockwise
    if(stop_motor == true){
      brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    }
    else if(filtered_duty == 0.0){
      if(init_duty > 0){  // positive duty cycle
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, abs(init_duty));
      }
      else if(init_duty < 0){ // negative duty cycle
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, abs(init_duty));
      }
    }
    else{
      if(filtered_duty > 0){  // positive duty cycle
        brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, abs(filtered_duty));
      }
      else if(filtered_duty < 0){ // negative duty cycle
        brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, abs(filtered_duty));
      }
    }
    vTaskDelay(1000/portTICK_RATE_MS);
  }
}


void app_main(void)
{
  printf("Testing brushed motor... \n");
  xTaskCreate(mcpwm_execute, "mcpwm_execute", 4096, NULL, 5, NULL);
  xTaskCreate(rotary_encoder_execute, "rotary_encoder_execute", 4096, NULL, 2, NULL);
  xTaskCreate(pulse_counter_execute, "pulse_counter_execute", 4096, NULL, 5, NULL);
  xTaskCreate(PID_execute, "PID_execute", 4096, NULL, 10, NULL);
}
