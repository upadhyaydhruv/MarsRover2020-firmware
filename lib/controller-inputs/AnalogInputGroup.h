#pragma once

#include "AnalogBusIn.h"
#include "mbed.h"

/* This class wraps an AnalogBusIn object. It is used for reading a group of analog inputs,
   and provides averaging to the read values
*/

class AnalogInputGroup {
 public:
  AnalogInputGroup(AnalogBusIn& inputs, uint8_t numPins, uint8_t numSampleToAverage = DEFAULT_NUM_SAMPLE_AVERAGE);

  virtual ~AnalogInputGroup() = default;

  // some getters
  uint8_t getNumInputs() const;

  static constexpr uint8_t MAX_INPUTS_NUM = 8;

  /* Read the analog inputs. This function is intended to be called as often as possible,
     ideally in a tight loop
      */
  void read();

  /*
          Get the average values of the analog inputs by reference.
          This function is overloaded to be able to return the analog input values in different format

          Note that the order of pins in the constructor of AnalogBusIn is the reverse order of the pins in the
          byte order. If you have BusIn(a,b,c,d,e,f,g,h) then the order of bits in the byte would be
          hgfedcba with a being bit 0, b being bit 1, c being bit 2 and so on.
  */

  // the client should make sure the passed in array is large enough
  void getValues(float* values);
  void getValues(uint16_t* values);

  /* reset the internal variables to start fresh, called by getValuesAndInvalidReads */
  void reset();

 protected:
  static constexpr uint8_t MAX_NUM_SAMPLE_AVERAGE      = 20;
  static constexpr uint16_t DEFAULT_NUM_SAMPLE_AVERAGE = 5;

  AnalogBusIn& m_inputs;
  uint16_t m_numPins;
  uint8_t m_num_samples_to_average = DEFAULT_NUM_SAMPLE_AVERAGE;
  uint8_t m_num_samples            = 0;
  // we make the array size big enough when declared. This might result in some waste, but this
  // allows us to not dynamically allocate memory
  uint16_t m_values[MAX_INPUTS_NUM][MAX_NUM_SAMPLE_AVERAGE] = {0};
  uint16_t m_values_idx[MAX_INPUTS_NUM]                     = {0};
  uint32_t m_values_sum[MAX_INPUTS_NUM]                     = {0};
};