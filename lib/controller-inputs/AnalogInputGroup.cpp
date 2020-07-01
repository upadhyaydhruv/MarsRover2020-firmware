#include "AnalogInputGroup.h"

AnalogInputGroup::AnalogInputGroup(AnalogBusIn& inputs, uint8_t numPins, uint8_t numSampleToAverage)
    : m_inputs(inputs), m_numPins(numPins), m_num_samples_to_average(numSampleToAverage) {}

uint8_t AnalogInputGroup::getNumInputs() const {
  return m_numPins;
}

void AnalogInputGroup::read() {
  static uint16_t values[MAX_INPUTS_NUM] = {0};

  // read the values
  m_inputs.read_u16(values);

  // apply moving average
  for (unsigned i = 0; i < m_numPins; i++) {
    uint16_t curr_idx = m_values_idx[i];

    m_values_sum[i] -= m_values[i][curr_idx];
    m_values[i][curr_idx] = values[i];
    m_values_sum[i] += m_values[i][curr_idx];

    curr_idx++;
    // wrap around the index
    if (curr_idx > m_num_samples_to_average) curr_idx = 0;

    // we also record how many sample we have read so far, and cap it at to the value of
    // m_num_samples_to_average
    if (m_num_samples < m_num_samples_to_average) m_num_samples++;

    m_values_idx[i] = curr_idx;
  }
}

void AnalogInputGroup::getValues(float* values) {
  for (unsigned i = 0; i < m_numPins; i++) {
    uint16_t u16_val = m_values_sum[i] / m_num_samples;

    // normalize from 0x0 - 0xFFFF to 0.0 - 1.0
    values[i] = float(u16_val) / float(0xFFFF);  // no need to * 1.0;
  }

  reset();
}

void AnalogInputGroup::getValues(uint16_t* values) {
  for (unsigned i = 0; i < m_numPins; i++) {
    values[i] = m_values_sum[i] / m_num_samples;
  }

  reset();
}

void AnalogInputGroup::reset() {
  m_num_samples = 0;

  for (unsigned i = 0; i < m_numPins; i++) {
    m_values_idx[i] = 0;
    m_values_sum[i] = 0;

    for (unsigned j = 0; j < m_num_samples_to_average; j++) {
      m_values[i][j] = 0;
    }
  }
}