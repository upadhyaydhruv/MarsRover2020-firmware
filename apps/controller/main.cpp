/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "mbed.h"
#include "rover_config.h"

#define DEBUG

static constexpr uint32_t MAX_UINT32               = 0xFFFFFFFF;
static constexpr uint16_t SEND_INTERVAL_MS         = 100;
static constexpr uint8_t DEBOUNCE_THRES            = 3;
static constexpr uint8_t ANALOG_NUM_SAMPLE_AVERAGE = 5;

DigitalIn joy_btn(JOY_BTN);
DigitalIn pb_1(PB1);
DigitalIn pb_2(PB2);
DigitalIn sw_1a(SW_1A);
DigitalIn sw_1b(SW_1B);
DigitalIn sw_2a(SW_2A);
DigitalIn sw_2b(SW_2B);

static DigitalIn digital_inputs[] = {
    joy_btn, pb_1, pb_2, sw_1a, sw_1b, sw_2a, sw_2b,
};

static constexpr uint8_t NUM_DIGITAL_IN = sizeof(digital_inputs) / sizeof(PinName);
// Although for the test board, there is only 7 digital inputs, in the real boards
// there can be more than 8 digital inputs, so using uint16_t here
uint16_t digital_inputs_values                 = 0;
uint16_t digital_inputs_curr_values            = 0;
uint16_t digital_inputs_counts[NUM_DIGITAL_IN] = {0};

AnalogIn joy_sm_x(JOY_SM_X);
AnalogIn joy_sm_y(JOY_SM_Y);
AnalogIn joy_x(JOY_X);
AnalogIn joy_y(JOY_Y);
AnalogIn pot_al(POT_AL);
AnalogIn slide_pot_al(SLIDE_POT_AL);

static AnalogIn analog_inputs[] = {
    joy_sm_x, joy_sm_y, joy_x, joy_y, pot_al, slide_pot_al,
};

static constexpr uint8_t NUM_ANALOG_IN                                  = sizeof(analog_inputs) / sizeof(PinName);
uint16_t analog_inputs_values[NUM_ANALOG_IN][ANALOG_NUM_SAMPLE_AVERAGE] = {0};
uint16_t analog_inputs_values_idx[NUM_ANALOG_IN]                        = {0};
uint32_t analog_inputs_sums[NUM_ANALOG_IN]                              = {0};

static constexpr uint8_t NUM_BYTES_DIGITAL_IN    = sizeof(uint16_t);
static constexpr uint8_t NUM_BYTES_PER_ANALOG_IN = sizeof(uint16_t);
static constexpr uint16_t NUM_BYTES_ANALOG_IN    = sizeof(uint16_t) * NUM_ANALOG_IN;

// Timer for checking interval between sending data to PC
Timer timer;

// Serial object for sending data to PC
Serial pc(USBTX, USBRX);

// Integrating debouncing is applied, not sure if this is necessary, needs testing
void read_but_switches(uint16_t& digital_inputs_values, uint16_t& digital_inputs_curr_values,
                       uint16_t digital_inputs_counts[NUM_DIGITAL_IN]) {
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    int val = digital_inputs[i].read();  // val = 0 or 1
    // if we read the same value as previous read, increment count
    if ((digital_inputs_curr_values & 1 << i) == val) digital_inputs_counts[i]++;
    // if not, set count back to zero
    else
      digital_inputs_counts[i] = 0;
    // when there are DEBOUNCE_THRES number of consecutive reads of the same value
    if (digital_inputs_counts[i] >= DEBOUNCE_THRES) {
      digital_inputs_values |= (digital_inputs_curr_values & 1 << i);
    }

    // record the current value for comparsion in next read
    digital_inputs_curr_values |= val << i;
  }
}

// Not sure if the mbed analogIn reads fast enough, needs testing
// Moving averaging is applied, not sure if this is necessary, needs testing
void read_joy_pot(uint16_t analog_inputs_values[NUM_ANALOG_IN][ANALOG_NUM_SAMPLE_AVERAGE],
                  uint16_t analog_inputs_values_idx[NUM_ANALOG_IN], uint32_t analog_inputs_sums[NUM_ANALOG_IN]) {
  for (int i = 0; i < NUM_ANALOG_IN; i++) {
    uint16_t val      = analog_inputs[i].read();
    uint16_t curr_idx = analog_inputs_values_idx[i];

    analog_inputs_sums[i] -= analog_inputs_values[i][curr_idx];
    analog_inputs_values[i][curr_idx] = val;
    analog_inputs_sums[i] += analog_inputs_values[i][curr_idx];

    curr_idx++;
    if (curr_idx > ANALOG_NUM_SAMPLE_AVERAGE) curr_idx = 0;

    analog_inputs_values_idx[i] = curr_idx;
  }
}

bool interval_passed(uint32_t start_time, uint32_t current_time, uint16_t interval) {
  if (current_time > start_time) return (current_time - start_time) > interval;
  // else the timer has wrapped around
  else
    return (MAX_UINT32 - start_time + current_time) > interval;
}

void prepare_data(uint8_t* outbuff, uint16_t& digital_inputs_values, uint32_t analog_inputs_sums[NUM_ANALOG_IN]) {
  memcpy(outbuff, &digital_inputs_values, NUM_BYTES_DIGITAL_IN);

  for (int i = 0; i < NUM_ANALOG_IN; i++) {
    /*
            Before the moving average array for each input is filled at least once, some elements will be zero,
            but it's probably ok to assume it will be filled by the time we calculate the average, so we'll just
            divide by ANALOG_NUM_SAMPLE_AVERAGE here
            But testing is still needed
    */
    uint16_t avg = analog_inputs_sums[i] / ANALOG_NUM_SAMPLE_AVERAGE;
    memcpy(outbuff + NUM_BYTES_DIGITAL_IN + i * sizeof(avg), &avg, sizeof(avg));
  }
}

void clean_data(uint16_t& digital_inputs_values, uint16_t& digital_inputs_curr_values,
                uint16_t digital_inputs_counts[NUM_DIGITAL_IN],
                uint16_t analog_inputs_values[NUM_ANALOG_IN][ANALOG_NUM_SAMPLE_AVERAGE],
                uint16_t analog_inputs_values_idx[NUM_ANALOG_IN], uint32_t analog_inputs_sums[NUM_ANALOG_IN]) {
  digital_inputs_values      = 0;
  digital_inputs_curr_values = 0;

  memset(digital_inputs_counts, 0, sizeof(digital_inputs_counts[0]) * NUM_DIGITAL_IN);

  memset(analog_inputs_values, 0, sizeof(analog_inputs_values[0][0]) * NUM_ANALOG_IN * ANALOG_NUM_SAMPLE_AVERAGE);
  memset(analog_inputs_values_idx, 0, sizeof(analog_inputs_values_idx[0]) * NUM_ANALOG_IN);
  memset(analog_inputs_sums, 0, sizeof(analog_inputs_sums[0]) * NUM_ANALOG_IN);
}

void send_to_pc(uint16_t& digital_inputs_values, uint16_t& digital_inputs_curr_values,
                uint16_t digital_inputs_counts[NUM_DIGITAL_IN],
                uint16_t analog_inputs_values[NUM_ANALOG_IN][ANALOG_NUM_SAMPLE_AVERAGE],
                uint16_t analog_inputs_values_idx[NUM_ANALOG_IN], uint32_t analog_inputs_sums[NUM_ANALOG_IN]) {
  static uint8_t outbuff[NUM_BYTES_DIGITAL_IN + NUM_BYTES_ANALOG_IN] = {0};

  prepare_data(outbuff, digital_inputs_values, analog_inputs_sums);

  // sending data out
  for (unsigned i = 0; i < sizeof(outbuff); i++) {
    pc.putc(outbuff[i]);
  }

  clean_data(digital_inputs_values, digital_inputs_curr_values, digital_inputs_counts, analog_inputs_values,
             analog_inputs_values_idx, analog_inputs_sums);
}

int main() {
  printf("Beginning robot controller fw app.\r\n");
  // set flow control, as we are sending a lot stuff
  pc.set_flow_control(SerialBase::RTSCTS, USB_RTS, USB_CTS);
  timer.start();

  while (1) {
    uint32_t start_time = timer.read_ms();

    // Assume all inputs during SEND_INTERVAL_MS is unchanging, i.e. all inputs in this period is accumulated
    while (!interval_passed(start_time, timer.read_ms(), SEND_INTERVAL_MS)) {
      // May need to implement these two read functions as threads, needs testing
      read_but_switches(digital_inputs_values, digital_inputs_curr_values, digital_inputs_counts);
      read_joy_pot(analog_inputs_values, analog_inputs_values_idx, analog_inputs_sums);
    }

#ifndef DEBUG
    send_to_pc(digital_inputs_values, digital_inputs_curr_values, digital_inputs_counts, analog_inputs_values,
               analog_inputs_values_idx, analog_inputs_sums);
#else
    printf("Digital Inputs: %x", digital_inputs_values);

    for (int i = 0; i < NUM_ANALOG_IN; i++) {
      float avg = analog_inputs_sums[i] / ANALOG_NUM_SAMPLE_AVERAGE;
      printf("Andlog Inputs %d: %f", i, avg);
    }

#endif
  }

  return 1;
}
