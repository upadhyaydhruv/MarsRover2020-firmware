#pragma once

// Serial config
#define ROVER_DEFAULT_SERIAL_BAUD_RATE 115200

// CAN bus config
#define ROVER_CANBUS_FREQUENCY          500000  // 500 kbps
#define ROVER_CANID_FILTER_MASK         0xFE0   // Use bits [5:10] for addressing and 0:7 for command/message type
#define ROVER_CANID_FIRST_ERROR_TX      0x100
#define ROVER_CANID_FIRST_SAFETY_TX     0x720
#define ROVER_CANID_FIRST_SAFETY_RX     0x730
#define ROVER_CANID_FIRST_ARM_RX        0x740
#define ROVER_CANID_FIRST_ARM_TX        0x750
#define ROVER_CANID_FIRST_SCIENCE_RX    0x760
#define ROVER_CANID_FIRST_SCIENCE_TX    0x770
#define ROVER_CANID_FIRST_GIMBTONOMY_RX 0x780
#define ROVER_CANID_FIRST_GIMBTONOMY_TX 0x790

// Controls
#define ROVER_MOTOR_PWM_FREQ_HZ 20000  // 20 kHz
