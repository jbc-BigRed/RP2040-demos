/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration calculates an FFT of audio input, and
 * then displays that FFT on a 640x480 VGA display.
 * 
 * Core 0 computes and displays the FFT.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green 
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 26 ---> Audio input [0-3.3V]

     KEYPAD CONNECTIONS
    - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
    - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
    - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
    - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
    - GPIO 13  -->     Pin 5 (button col 1)
    - GPIO 14  -->     Pin 6 (button col 2)
    - GPIO 15  -->     Pin 7 (button col 3)
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - ADC channel 0
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include VGA graphics library
#include "vga16_graphics_v2.h"
// Include standard libraries
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <iso646.h>
#include <pico/error.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/divider.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"
#include <rotaryencoder/debounced_encoder.h>



#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
#define BUTTON 4

static encoder_state enc_state;
static int32_t encoder_value = 0;

// Read A/B as a 2-bit value: bit0 = A, bit1 = B, 0..3
static inline uint8_t read_encoder_terminals(void) {
    uint8_t a = gpio_get(ENCODER_PIN_A);
    uint8_t b = gpio_get(ENCODER_PIN_B);
    // if you use pull-ups, signals are active-low:
    // convert to logical "pressed = 1"
    a = !a;
    b = !b;
    return (a | (b << 1)) & 0x3;
}

int main() {
    stdio_init_all();

    // Init pins
    gpio_init(ENCODER_PIN_A);
    gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_A);

    gpio_init(ENCODER_PIN_B);
    gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_B);

    // Initialize encoder state with current terminal state
    uint8_t initial = read_encoder_terminals();
    encoder_debounced_full_step_init(&enc_state, initial);

    int32_t last_value = encoder_value;

    while (true) {
        uint8_t term = read_encoder_terminals();
        switch (encoder_debounced_full_step_update(&enc_state, term)) {
            case ENCODER_ACTION_TURN_CW:
                encoder_value++;
                break;
            case ENCODER_ACTION_TURN_CCW:
                encoder_value--;
                break;
            default:
                break;
        }

        if (encoder_value != last_value) {
            last_value = encoder_value;
            printf("Encoder value = %ld\n", (long)encoder_value);
        }

        // Polling rate: 1 kHz is usually enough; adjust as needed
        sleep_ms(1);
    }
}