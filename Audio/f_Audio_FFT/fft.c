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
#define BUTTON_PIN 4

static encoder_state enc_state;
//static int32_t encoder_value = 0;


volatile int SCROLL_SPEED = 10 ;
volatile int CENTER_FREQ = 440 ;
volatile float SCALING_FACTOR = 10.0 ;

// state machine variables - potentiometer button pressing
#define PIN_POT_BUTTON 5 // gpio 5 (pin 7)
// states
#define INIT 0 // potentiometer has no impact
#define MOD_SCROLL_SPEED 1 // to adjust the scroll speed, increases rectangle size
#define MOD_CENTER_FREQ 2 // to adjust the center frequency of tuning
#define MOD_SCALING_FACTOR 3 // how much to multiply the values for the heat map (sensitivity)
volatile unsigned int POT_STATE = INIT ; // initalize the state of this fsm
volatile unsigned int P_CYCLE_STATE = INIT ;
volatile int p_possible = 0 ;
volatile int pot_funct = INIT ;

#define NOT_PRESSED 0
#define MAYBE_PRESSED 1
#define PRESSED 2
#define MAYBE_NOT_PRESSED 3

static struct pt_sem pot_btn_pressed ;


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

// Input button debouncing for potentiometer state management
static PT_THREAD(protothread_POT_debouncing(struct pt *pt)) 
{
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;
  
  while(1) {
    begin_time = time_us_32() ;

    int p = gpio_get(BUTTON_PIN) ; // value of pot button press
   
    // implementing this debouncing algorithm with switch for clarity rather than if statements
    switch (POT_STATE) {
      case NOT_PRESSED :
        if (p == 0) { // if the button is low (pressed)
          POT_STATE = MAYBE_PRESSED ;
          p_possible = p ; 
        }
        break ;
      case MAYBE_PRESSED :
        if (p == p_possible) {
            POT_STATE = PRESSED ;
            PT_SEM_SIGNAL(pt, &pot_btn_pressed) ; // send flag, potFSM thread will be activated by this
            printf("Button pressed") ;
        }
        else { 
            POT_STATE = NOT_PRESSED ;
        }
        break ;
      case PRESSED :
        if (p == 1) { // if the button is seen high again, maybe not pressed
            POT_STATE = MAYBE_NOT_PRESSED ;
            p_possible = p ;
        }
        break ;
      case MAYBE_NOT_PRESSED :
        if (p == p_possible) { //  possible is 1 right now, so if it is high send to not pressed
          POT_STATE = NOT_PRESSED ;
        }
        else {
          POT_STATE = PRESSED ;
        }
        break ;
    }
    
    // delay in accordance with frame rate
    spare_time = 30000 - (time_us_32() - begin_time) ;

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time) ;
  }
  PT_END(pt) ;
} // thread for the debouncing

// thread to manage state using debounced input
static PT_THREAD(protothread_potFSM(struct pt *pt))
{
  PT_BEGIN(pt) ;

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;

  while(1) {
    // since this thread just cycles based on button presses, 
    // can just wait until the flag is incremented, 
    // and then restart STATE_1 in a loop without switch statements

    PT_SEM_WAIT(pt, &pot_btn_pressed);

    begin_time = time_us_32() ; // idk where to put this

    P_CYCLE_STATE = (P_CYCLE_STATE + 1) % 4 ; // states 0 through 3, will loop when state reaches 3

    switch (P_CYCLE_STATE) { // based on state display the currrent state and determine the function of the potentiometer
      case INIT :
        //strcpy(pot_state_buffer, "Standby") ;
        pot_funct = INIT ;
      break ;
      case MOD_SCROLL_SPEED :
        //strcpy(pot_state_buffer, "Adjusting scroll speed: ") ;
        pot_funct = MOD_SCROLL_SPEED ;
      break ;
      case MOD_CENTER_FREQ :
        //strcpy(pot_state_buffer, "Adjusting center frequency: ") ;
        pot_funct = MOD_CENTER_FREQ ;
      break ;
      case MOD_SCALING_FACTOR :
        //strcpy(pot_state_buffer, "Adjusting scaling factor: ") ;
        pot_funct = MOD_SCALING_FACTOR ;
      break ;
    }

    // delay in accordance with frame rate
    spare_time = 30000 - (time_us_32() - begin_time) ;

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time) ;
  }

  PT_END(pt) ;
} // thread for potentiometer FSM

// Encoder protothread
static PT_THREAD (protothread_encoder(struct pt *pt))
{
    PT_BEGIN(pt);
    
    // Variables for maintaining frame rate
    static int spare_time;
    static uint32_t begin_time;
    static uint8_t output;
    static int action;
    
    while(1) {
        begin_time = time_us_32();
        
        // Read encoder terminals
        output = read_encoder_terminals();
        
        // Update encoder state and get action
        action = encoder_debounced_half_step_update(&enc_state, output);
        
        switch (action) {
            case ENCODER_ACTION_TURN_CW:
                switch (pot_funct) {
                    case MOD_SCROLL_SPEED:
                        SCROLL_SPEED++ ;
                        break;
                    case MOD_CENTER_FREQ:
                        CENTER_FREQ++ ;
                        break ;
                    case MOD_SCALING_FACTOR:
                        SCALING_FACTOR += 0.1 ;
                        break ;
                    case INIT :
                        break ;
                }
                break;
            case ENCODER_ACTION_TURN_CCW:
                switch (pot_funct) {
                    case MOD_SCROLL_SPEED:
                        SCROLL_SPEED++ ;
                        break;
                    case MOD_CENTER_FREQ:
                        CENTER_FREQ++ ;
                        break ;
                    case MOD_SCALING_FACTOR:
                        SCALING_FACTOR += 0.1 ;
                        break ;
                    case INIT :
                        break ;
                }
                break;
            default:
                break;
        }
        
        // Print value if changed
        printf("scroll speed: %d, center freq: %d, scaling factor: %f\n", SCROLL_SPEED, CENTER_FREQ, SCALING_FACTOR) ;
        
        // Polling rate: 1 kHz (1ms delay)
        spare_time = 1000 - (time_us_32() - begin_time);
        
        // Yield for necessary amount of time
        PT_YIELD_usec(spare_time);
    }
    
    PT_END(pt);
}


int main() {
    stdio_init_all();

    // Initialize encoder pins
    gpio_init(ENCODER_PIN_A);
    gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_A);

    gpio_init(ENCODER_PIN_B);
    gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_B);

    // Initialize button pin (if needed)
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // Initialize encoder state with current terminal state
    uint8_t initial = read_encoder_terminals();
    encoder_debounced_full_step_init(&enc_state, initial);

    // Then add the thread:
    pt_add_thread(protothread_encoder);
    pt_add_thread(protothread_POT_debouncing) ;
    pt_add_thread(protothread_potFSM) ;
    pt_schedule_start ;
}