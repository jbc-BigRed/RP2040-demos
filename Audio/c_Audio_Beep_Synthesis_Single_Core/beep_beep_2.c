/**
 *  V. Hunter Adams (vha3@cornell.edu)
    Jack Chaney (jbc282@cornell.edu)
    Arielle Huang (aph74@cornell.edu)

    bird sounds come out the pico

    GPIO 5 (pin 7) Chip select
    GPIO 6 (pin 9) SCK/spi0_sclk
    GPIO 7 (pin 10) MOSI/spi0_tx
    GPIO 2 (pin 4) GPIO output for timing ISR
    3.3v (pin 36) -> VCC on DAC
    GND (pin 3)  -> GND on DAC

    KEYPAD CONNECTIONS
    - GPIO 9   -->  330 ohms  --> Pin 1 (button row gg
    - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
    - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
    - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
    - GPIO 13  -->     Pin 5 (button col 1)
    - GPIO 14  -->     Pin 6 (button col 2)
    - GPIO 15  -->     Pin 7 (button col 3)

    SERIAL CONNECTIONS
    - GPIO 0        -->     UART RX (white)
    - GPIO 1        -->     UART TX (green)
    - RP2040 GND    -->     UART GND

 */

// Include necessary libraries
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// Include pre-calculated LUTs
#include "sin_table.h"
#include "swoop_table.h"
#include "chirp_table.h"

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

// Constants for keypad
unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;
char keytext[40];
int prev_key = 0;

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0))
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
//volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;
volatile unsigned int phase_incr_main_0 ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           6500 // 50kint/sec(0.130sec) interrupts
#define BEEP_REPEAT_INTERVAL    50000

// State machine variables
volatile unsigned int count_0 = 0 ;
volatile unsigned int make_beep = 0 ;
volatile int possible = 0 ;
volatile unsigned int STATE_0 = 0 ;

// Variables for recording
volatile int bird_noises[16] = {    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1} ; // array for recording
volatile int is_recording = 0 ;
volatile int noise_idx = 0 ;
volatile int is_done = 0 ; // 0 until a -1 is seen or the end of the array is hit
volatile int done_recording = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

// GPIO for timing the ISR
#define ISR_GPIO 2

// Define DMA channel and a buffer to hold the value to be transferred
int dma_chan;
volatile uint16_t dma_buffer;

// This timer ISR is called on core 0
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interrupt for timing analysis
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    // now instead of using STATE_0 here to trigger a repeated beep
    // use the make_beep flag to trigger the beep
    if (make_beep == 1) {
        // see if button 1 or 2 is pressed to determine swoop or chirp
        if (possible == 1) {
            phase_incr_main_0 = swoop_table[ count_0 ] ;
        }
        else if (possible == 2) {
            // generating phase at current count for chirp
            phase_incr_main_0 = chirp_table[ count_0 ] ;
        }

        // DDS phase and sine table lookup
        phase_accum_main_0 += phase_incr_main_0  ;

        // TODO: ADD CONDITIONAL LOGIC TO MAKE THE PAUSE FOR FINAL LAB
        // Quick fixed point multiplication to calculated amplitude for wave
        fix15 modulated_sine = multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0 >> 24]) ;

        // keypad # = 11 due to the masking for computing the valid keycode
        // adds pause if not 1 and 2
        if (possible != 1) {
            if (possible !=2 ) {
                modulated_sine = 0 ;
            }
        }

        DAC_output_0 = ((int32_t)modulated_sine * 2047 >> 15) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits, config_chan is the output channel of the
        // DAC
        DAC_data_0 = ( DAC_config_chan_A | ( DAC_output_0 & 0xffff ))  ;

        // DMA-based non-blocking write to DAC
        dma_buffer = DAC_data_0 ;

        // If the DMA isn't busy, then a new transfer is triggered, this makes sure sound output is continuous
        if(!dma_channel_is_busy(dma_chan)) {
            dma_channel_set_read_addr(dma_chan,
                &dma_buffer, true) ;
        }

        // Increment the counter
        count_0 += 1 ;

        // is beep done? if so reset the variables
        if (count_0 == BEEP_DURATION) {
            count_0 = 0 ;
            make_beep = 0;
            // if the toggle from record to not record play the beep
            if (is_recording == 0 && done_recording == 1) {
                if ((noise_idx < 16) && (bird_noises[noise_idx] != -1)) {
                    make_beep = 1 ;
                    noise_idx += 1 ;
                }
            }
            current_amplitude_0 = 0;
        }

    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0) ;

}


// blinking light thread
// This thread runs on core 1
static PT_THREAD (protothread_led_blink(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {

        // Toggle on LED
        gpio_put(LED, !gpio_get(LED)) ;

        // Yield for 500 ms
        PT_YIELD_usec(500000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// Keyboard thread
// This thread runs on core 0
static PT_THREAD (protothread_debouncy_boi(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    // incrementer for looping
    static int i ;
    // keypad variable to track button presses
    static uint32_t keypad ;
    // maps to active key for retrigger purposes
    static int active_key = -1 ;

    while(1) {
        // Below code until else (i=-1) ; is checking what the button pressed is, then after will implement state machine
        if (done_recording == 0) {
            // Scan the keypad!
            for (i=0; i<KEYROWS; i++) {
                // Set a row high
                gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                                (scancodes[i] << BASE_KEYPAD_PIN)) ;
                // Small delay required
                sleep_us(1) ;
                // Read the keycode
                keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
                // Break if button(s) are pressed
                if (keypad & button) break ;
            }
            // If we found a button . . .
            if (keypad & button) {
                // Look for a valid keycode.
                for (i=0; i<NUMKEYS; i++) {
                    if (keypad == keycodes[i]) break ;
                }
                // If we don't find one, report invalid keycode
                if (i==NUMKEYS) (i = -1) ;
            }
            // Otherwise, indicate invalid/non-pressed buttons
            else (i=-1) ;
        }
        else {
            // want to not set i = -1 if in the play state after recording
            if (done_recording == 1 && is_recording == 0) {
                i = bird_noises[noise_idx] ;
                possible = i ; // matches possible to the current i
                STATE_0 = 1;
                if (i == -1 || i == 15) { // if reached the end of the recording or the max length of recording
                    printf("exiting spoofed keypress loop") ;
                    done_recording = 0;
                    for (int k = 0; k < 16; k++) {
                        bird_noises[k] = -1 ;
                    }
                    noise_idx = 0 ;
                }
            }
        }

        // Now implementing state machine logic to see when the beep will play (FSM)
        // STATE_0 is initialized as 0 when program starts
        // If STATE_0 == 0 (keypad = -1), then remain in that state
        // Not pressed state
        if (STATE_0 == 0) {
            // if no press or invalid, stay in state 0
            if (i == -1) {
                STATE_0 = 0;
            }
            // press is valid, move to maybe pressed state
            else {
                STATE_0 = 1;
                possible = i;
            }
        }
        // Maybe pressed state
        else if (STATE_0 == 1) {
            // if the numbers match up, then move on to next state
            // this is the state transitioning from maybe pressed to pressed
            // so now the beep will be triggered here (flag)
            if (possible == i) {
                STATE_0 = 2;
                make_beep = 1;
                // toggle button for recording
                if (i == 11) { // pressed #
                    if (is_recording == 0) { // if previously in the not recording state, set to record
                        is_recording = 1 ;
                    }
                    else {
                        is_recording = 0 ; // if previously in the recording state, then set to play
                        noise_idx = 0 ; // start the noise playing from the beginning
                        done_recording = 1 ; // arbitrary play noise flag
                    }
                }
                // if recording is valid, then see what button was pressed and then add to the recording (up to 16 presses per recording)
                if (is_recording) {
                    if ((i == 1 || i == 2 || i == 3) && (noise_idx < 16)) {
                        bird_noises[noise_idx] = i ;
                        printf("pos: %d\nval: %d\n", noise_idx, i) ;
                        noise_idx++ ;
                    }
                }
            }
            // else go back to not pressed
            else {
                STATE_0 = 0;
            }
        }
        // pressed state
        else if (STATE_0 == 2) {
            // if key pressed still matches remain in state
            if (possible == i) {
                STATE_0 = 2;
            }
            // else move to maybe not pressed
            else {
                STATE_0 = 3;
            }
        }
        // maybe not pressed state
        else if (STATE_0 == 3) {
            // if matches, go back to pressed
            if (possible == i) {
                STATE_0 = 2;
            }
            // else go to not pressed
            else {
                STATE_0 = 0;
            }
        }

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

void core1_entry() {

    // Add keypad FSM thread to core1 scheduler
    pt_add_thread(protothread_debouncy_boi) ;

    // Start scheduler on core1
    pt_schedule_start ;
}

// dds main
// Core 0 entry point
int main() {

    // Overclock
    set_sys_clock_khz(250000, true) ;

    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    // Configure DMA for sending to DAC
    dma_chan = dma_claim_unused_channel(true) ;
    dma_channel_config c = dma_channel_get_default_config(dma_chan) ;

    // Transfer 16 bit values from buffer to SPI transmit register
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16) ;

    // No read increment since the buffer will be written to and read repeatedly
    channel_config_set_read_increment(&c, false);

    // No write increment since the destination (SPI TX) is fixed
    channel_config_set_write_increment(&c, false);

    // Set the DMA to trigger then the SPI TX FIFO has space
    channel_config_set_dreq(&c, spi_get_dreq(SPI_PORT, true)) ;

    // configure DMA channel to write to the spi data register from the buffer, one value at a time, and we will tell it when to start (trigger:false)
    dma_channel_configure(
        dma_chan,
        &c,
        &spi_get_hw(SPI_PORT)->dr,
        &dma_buffer,
        1,
        false
    ) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // launch core1 entry
    multicore_launch_core1(core1_entry) ;

    // Add core 0 thread
    pt_add_thread(protothread_led_blink) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}
