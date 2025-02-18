/*
 *  PinDefinitionsAndMore.h
 *
 *  Contains SERVO*_PIN definitions for ServoEasing examples for various platforms
 *  as well as includes and definitions for LED_BUILTIN
 *
 *  Copyright (C) 2020-20232  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of ServoEasing https://github.com/ArminJo/ServoEasing.
 *
 *  ServoEasing is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

/*
 * Pin mapping table for different platforms - used by all examples
 *
 * Platform         Servo1      Servo2      Servo3      Analog     Core/Pin schema
 * -------------------------------------------------------------------------------
 * (Mega)AVR + SAMD    9          10          11          A0
 * 2560               46          45          44          A0
 * ATtiny3217         20|PA3       0|PA4       1|PA5       2|PA6   MegaTinyCore
 * ESP8266            14|D5       12|D6       13|D7        0
 * ESP32               5          18          19          A0
 * BluePill          PB7         PB8         PB9         PA0
 * APOLLO3            11          12          13          A3
 * RP2040             6|GPIO18     7|GPIO19    8|GPIO20
 */

#if defined(__AVR_ATtiny1616__)  || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny3217__) // Tiny Core Dev board
#define SERVO1_PIN     20
#define SERVO2_PIN      0
#define SERVO3_PIN      1
#define SPEED_IN_PIN    2 // A6

#elif defined(__AVR_ATmega2560__) // Default as for ATmega328 like on Uno, Nano etc.
#define SERVO1_PIN 53 // For ATmega2560 pins 44 to 46 are connected to timer 5 and can therefore be used also by the Lightweight Servo library
#define SERVO2_PIN 45
#define SERVO3_PIN 44 // Channel of pin 44 is also used for generation of internal ServoEasing timing
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1

#elif defined(__AVR__) // Default as for ATmega328 like on Uno, Nano etc.
#define SERVO1_PIN 9 // For ATmega328 pins 9 + 10 are connected to timer 2 and can therefore be used also by the Lightweight Servo library
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SPEED_IN_PIN A0
// Potentiometer for value at A3 is breadboard friendly plugged in at A1 and A5 :-)
#define MODE_ANALOG_INPUT_PIN A2
#define REFRESH_PERIOD_ANALOG_INPUT_PIN A3

#elif defined(ESP8266)
#define SERVO1_PIN  14 // D5
#define SERVO2_PIN  12 // D6
#define SERVO3_PIN  13 // D7
#define SPEED_IN_PIN A0

#elif defined(ESP32)
#define SERVO1_PIN  5
#define SERVO2_PIN 18
#define SERVO3_PIN 19
#define SPEED_IN_PIN A0 // 36/VP
#define MODE_ANALOG_INPUT_PIN A3 // 39

#elif defined(STM32F1xx) || defined(__STM32F1__) // BluePill
// STM32F1xx is for "Generic STM32F1 series / STMicroelectronics:stm32" from STM32 Boards from STM32 cores of Arduino Board manager
// __STM32F1__is for "Generic STM32F103C series / stm32duino:STM32F1" from STM32F1 Boards (STM32duino.com) of Arduino Board manager
#define SERVO1_PIN PB7
#define SERVO2_PIN PB8
#define SERVO3_PIN PB9 // Needs timer 4 for Servo library
#define SPEED_IN_PIN PA0
#define MODE_ANALOG_INPUT_PIN PA1

#elif defined(ARDUINO_ARCH_APOLLO3) // Sparkfun Apollo boards
#define SERVO1_PIN 11
#define SERVO2_PIN 12
#define SERVO3_PIN 13
#define SPEED_IN_PIN A2
#define MODE_ANALOG_INPUT_PIN A3

#elif defined(ARDUINO_ARCH_MBED) // Arduino Nano 33 BLE
#define SERVO1_PIN 6
#define SERVO2_PIN 7
#define SERVO3_PIN 8
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1

#elif defined(ARDUINO_ARCH_RP2040) //Arduino Nano Connect, Pi Pico with arduino-pico core https://github.com/earlephilhower/arduino-pico
#define SERVO1_PIN 18
#define SERVO2_PIN 19
#define SERVO3_PIN 20
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1

#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAM)
#define SERVO1_PIN  5
#define SERVO2_PIN  6
#define SERVO3_PIN  7
#define SPEED_IN_PIN A1 // A0 is DAC output
#define MODE_ANALOG_INPUT_PIN A2

#if !defined(ARDUINO_SAMD_ADAFRUIT)
// On the Zero and others we switch explicitly to SerialUSB
#define Serial SerialUSB
#endif

// Definitions for the Chinese SAMD21 M0-Mini clone, which has no led connected to D13/PA17.
// Attention!!! D2 and D4 are swapped on these boards!!!
// If you connect the LED, it is on pin 24/PB11. In this case activate the next two lines.
//#undef LED_BUILTIN
//#define LED_BUILTIN 24 // PB11
// As an alternative you can choose pin 25, it is the RX-LED pin (PB03), but active low.In this case activate the next 3 lines.
//#undef LED_BUILTIN
//#define LED_BUILTIN 25 // PB03
//#define FEEDBACK_LED_IS_ACTIVE_LOW // The RX LED on the M0-Mini is active LOW

#else
#warning Board / CPU is not detected using pre-processor symbols -> using default values, which may not fit. Please extend PinDefinitionsAndMore.h.
// Default valued for unidentified boards
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SPEED_IN_PIN A0
#define MODE_ANALOG_INPUT_PIN A1

#endif

#define SERVO_UNDER_TEST_PIN SERVO1_PIN

#define SPEED_OR_POSITION_ANALOG_INPUT_PIN SPEED_IN_PIN
#define POSITION_ANALOG_INPUT_PIN SPEED_IN_PIN

// for ESP32 LED_BUILTIN is defined as: static const uint8_t LED_BUILTIN 2
#if !defined(LED_BUILTIN) && !defined(ESP32)
#define LED_BUILTIN PB1
#endif
