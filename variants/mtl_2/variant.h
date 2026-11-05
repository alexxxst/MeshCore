/*
 * variant.h
 * Copyright (C) 2023 Seeed K.K.
 * MIT License
 */

 #pragma once

 #include "WVariant.h"

////////////////////////////////////////////////////////////////////////////////
// Low frequency clock source

#define VARIANT_MCK       (64000000ul)

//#define USE_LFXO      // 32.768 kHz crystal oscillator
#define USE_LFRC    // 32.768 kHz RC oscillator

////////////////////////////////////////////////////////////////////////////////
// Power

#define PIN_EXT_VCC          (13)   // P0.13
#define EXT_VCC              (PIN_EXT_VCC)

#define BATTERY_PIN          (31)   // P0.31
#define ADC_RESOLUTION        12

////////////////////////////////////////////////////////////////////////////////
// Number of pins

#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (3)
#define NUM_ANALOG_OUTPUTS   (0)

////////////////////////////////////////////////////////////////////////////////
// UART pin definition
#define PIN_SERIAL1_TX       (-1)
#define PIN_SERIAL1_RX       (-1)

#define PIN_SERIAL2_TX       (22)    // P0.22
#define PIN_SERIAL2_RX       (20)    // P0.20

////////////////////////////////////////////////////////////////////////////////
// I2C pin definition

#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (36)    // P1.04
#define PIN_WIRE_SCL         (11)    // P0.11
//#define PIN_WIRE1_SDA        (13)
//#define PIN_WIRE1_SCL        (14)

////////////////////////////////////////////////////////////////////////////////
// SPI pin definition

#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_SCK          (33)   // P1.11
#define PIN_SPI_MISO         (2)    // P0.02
#define PIN_SPI_MOSI         (47)   // P1.15

#define PIN_SPI_NSS          (45)   // P1.13

//#define PIN_SPI1_SCK         (18)
//#define PIN_SPI1_MISO        (19)
//#define PIN_SPI1_MOSI        (20)

////////////////////////////////////////////////////////////////////////////////
// GPS
#define GPS_BAUDRATE            9600
#define GPS_RX_PIN              PIN_SERIAL2_RX
#define GPS_TX_PIN              PIN_SERIAL2_TX
#define PIN_GPS_STANDBY         (24)     // P0.24
#define PIN_GPS_EN              (PIN_GPS_STANDBY)

////////////////////////////////////////////////////////////////////////////////
// Builtin LEDs

#define PIN_LED              (-1)
#define LED_PIN              PIN_LED
#define LED_BLUE             PIN_LED
#define LED_BUILTIN          PIN_LED
#define LED_STATE_ON         1

////////////////////////////////////////////////////////////////////////////////
// Builtin buttons

#define PIN_BUTTON1             (-1)  // Menu / User Button
#define PIN_BUTTON2             (-1) // Joystick Up
#define PIN_BUTTON3             (-1) // Joystick Down
#define PIN_BUTTON4             (33) // Joystick Left   P1.01
#define PIN_BUTTON5             (34) // Joystick Right  P1.02
#define PIN_BUTTON6             (-1) // Joystick Press
#define PIN_BUTTON7             (32) // Joystick Press   P1.00
#define PIN_BUTTON8             (39) // Joystick Press   P1.07

#define PIN_BACK_BTN            PIN_BUTTON1
#define JOYSTICK_UP             PIN_BUTTON2
#define JOYSTICK_DOWN           PIN_BUTTON3
#define JOYSTICK_LEFT           PIN_BUTTON4
#define JOYSTICK_RIGHT          PIN_BUTTON5
#define JOYSTICK_PRESS          PIN_BUTTON6
#define PIN_USER_BTN            PIN_BUTTON7
#define PIN_USER_POWER          PIN_BUTTON8

#define USER_POWER_MIN          10
#define USER_POWER_MAX          20

////////////////////////////////////////////////////////////////////////////////
// Buzzer

#define PIN_VIBRATION           (8)            // P0.8
#define PIN_BUZZER              (6)            // P0.6

////////////////////////////////////////////////////////////////////////////////
// LBT
#define USE_LBT
#define SAMPLING_THRESHOLD 14
#define NUM_NOISE_FLOOR_SAMPLES 64
