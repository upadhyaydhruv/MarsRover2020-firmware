/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "SystemReport.h"
#include "mbed.h"

AnalogIn potVoltageIn(PA_0);
PwmOut servoPwmOut(PA_1);

// main() runs in its own thread in the OS
int main() {
  servoPwmOut.period(0.020);

  while (true) {
    float potVoltage = potVoltageIn.read();
    servoPwmOut.pulsewidth(0.001 + 0.001 * potVoltage / 3.3);
  }
}
