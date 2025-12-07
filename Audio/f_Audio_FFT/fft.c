/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration calculates an FFT of audio input, and
 * then displays that FFT on a 640x480 VGA display.
 * 
 * Core 0 computes and displays the FFT.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 2  ---> ENCODER_PIN_A
 *  - GPIO 3  ---> ENCODER_PIN_B
 *  - GPIO 5  ---> Button input for rotary encoder 
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green 
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 22 ---> Tune button input (button 2)
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 26 ---> Audio input [0-3.3V]
 *  - GPIO VSYS ---> Encoder power [5V]

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

// include for rotary encoder
#include <rotaryencoder/common.h>
#include <rotaryencoder/debounced_encoder.h>

// Define the LED pin
#define LED     25

// === the fixed point macros (16.15) ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
///////////////////////// FIX END /////////////////////////////////

/////////////////////////// Audio ADC configuration ////////////////////////////////
// ADC Channel and pin
#define ADC_AUDIO_CHAN 0
#define ADC_AUDIO_PIN 26 // pin 31, this is the mic input
#define ADC_LINEIN_CHAN 2
#define ADC_LINEIN_PIN 28 // GPIO 28, pin 34, this is the line in input
// Number of samples per FFT
#define NUM_SAMPLES 1024
// Number of samples per FFT, minus 1
#define NUM_SAMPLES_M_1 1023
// Length of short (16 bits) minus log2 number of samples (10)
#define SHIFT_AMOUNT 6
// Log2 number of samples
#define LOG2_NUM_SAMPLES 10
// Sample rate (Hz)
#define Fs 10000.0 // affects bin size
// ADC clock rate (unmutable!)
#define ADCCLK 48000000.0

// DMA channels for sampling ADC
int sample_chan ;
int control_chan ;

//////////////////////////////// Audio ADC END ////////////////////////////// 

//////////////////// Pot and Rotary Configuration //////////////////
// ADC Channel and pin
// #define ADC_POT_CHAN 1
// #define ADC_POT_PIN 27 // pin 32

#define ENCODER_PIN_A 2 // GPIO 2, pin 4 CLK
#define ENCODER_PIN_B 3 // GPIO 3, pin 5 DT
// power at vsys, pin 39

static encoder_state enc_state;

const int MAX_SCROLL_SPEED = 10 ;
const int MAX_CENTER_FREQ = 800 ;
const float MAX_SCALING_FACTOR = 100.0 ;

const int MIN_SCROLL_SPEED = 0 ;
const int MIN_CENTER_FREQ = 0 ;
const float MIN_SCALING_FACTOR = 0.0 ;

volatile int SCROLL_SPEED = 8 ; // drawing speed
volatile int CENTER_FREQ = 440 ; // tuning center frequency
volatile float SCALING_FACTOR = 6.0; // sensitivity

char pot_text_buffer[10] ;
//////////////////// Pot ADC END //////////////////


// FRAME RATE AND CLOCK SPEED
#define FRAME_RATE_30 33000 // ~30fps
#define FRAME_RATE_60 16500 // ~60fps
#define CLOCK_SPEED 250000


////////////////// math/ fft /////////////////////////////////////////////
// Max and min macros
#define max(a,b) ((a>b)?a:b)
#define min(a,b) ((a<b)?a:b)

// graph layout consts
const int SPECTRO_Y_START = 20;
const int SPECTRO_HEIGHT = 460;
const int SPECTRO_WIDTH = 640;
const int SPECTRO_X_START = 0;
const int SPECTRO_Y_END = SPECTRO_Y_START + SPECTRO_HEIGHT;
const int SPECTRO_X_END = SPECTRO_X_START + SPECTRO_WIDTH;
volatile int time_x = 0;

// 0.4 in fixed point (used for alpha max plus beta min)
fix15 zero_point_4 = float2fix15(0.4) ;

// Here's where we'll have the DMA channel put ADC samples
uint8_t sample_array[NUM_SAMPLES] ;
// And here's where we'll copy those samples for FFT calculation
fix15 fr[NUM_SAMPLES] ;
fix15 fi[NUM_SAMPLES] ;

// Sine table for the FFT calculation
fix15 Sinewave[NUM_SAMPLES]; 
// Hann window table for FFT calculation
fix15 window[NUM_SAMPLES]; 
////////////////////////////// fft end////////////////////////////////////

// Pointer to address of start of sample buffer
uint8_t * sample_address_pointer = &sample_array[0] ;

///////////////////// Constants for keypad ///////////////////////////////
// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12
unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
                            // 0, 1, 2, 3, 4
                            // 5, 6, 7, 8, 9
                            // *, #

                            // 1, 2, 3
                            // 4, 5, 6
                            // 7, 8, 9
                            // *, 0, #
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;

// Constants for debouncing state machine for keypad
#define NOT_PRESSED 0
#define MAYBE_PRESSED 1
#define PRESSED 2
#define MAYBE_NOT_PRESSED 3
volatile int possible = 0 ;
volatile unsigned int KEYPAD_STATE = NOT_PRESSED ;
char notes[12][6] = {"A#/Bb", "C", "C#/Db", "D", "D#/Eb", "E", "F", "F#/Gb", "G", "G#/Ab", "A", "B"} ; // mapping to the keycodes (index i)
char desired_note_buffer[30] = "Desired Tuning Note: "; // for outputting note on the VGA display
char current_note[6] = "None" ;
/////////////////////////////////// keypad end ///////////////////////////

// state management for display///////////////////////////////////////////

volatile short int change_tuning_state = 0;
volatile short int change_tuning_note = 0;
volatile short int tuning_text_drawn = 0;

volatile short int knob_mode_change = 0;
volatile short int knob_value_change = 0;

/////////////////// Constants for input mode state machine ///////////////
// debouncing inputs
//volatile unsigned int D_STATE = NOT_PRESSED ; // state variable for debouncer

// state machine variables - potentiometer button pressing
#define PIN_POT_BUTTON 5 // gpio 5 (pin 7)
// states
#define INIT 0 // potentiometer has no impact
#define MOD_SCROLL_SPEED 1 // to adjust the scroll speed, increases rectangle size
#define MOD_CENTER_FREQ 2 // to adjust the center frequency of tuning
#define MOD_SCALING_FACTOR 3 // how much to multiply the values for the heat map (sensitivity)
volatile unsigned int POT_STATE = INIT ; // initalize the state of this fsm
volatile unsigned int P_CYCLE_STATE = INIT ;
char pot_state_buffer[40] ; // buffer for snprintf for the current state
volatile int pot_funct = INIT ; // function of potentiometer, initialized at INIT, can be ADJUST_BALLS, ADJUST_BOUNCE
volatile int p_possible = 0 ;
//volatile int pot_btn_pressed = 0 ; // for initiating the pot cycle
static struct pt_sem pot_btn_pressed ;


// button for tuning enable
#define PIN_TUNE_BUTTON 22 // GPIO 22 (pin 29)
#define TUNE_DIS 0 // initialize on no tuning
#define TUNE_EN 1 // enable tuning on a button press
char tuning_state_buffer[17] ; // for vga
volatile unsigned int TUNE_STATE = TUNE_DIS ;
volatile unsigned int T_CYCLE_STATE = TUNE_DIS ;
volatile int tune_funct = INIT ;
//volatile int tune_btn_pressed = 0 ; // for initiating the pot cycle
static struct pt_sem tune_btn_pressed ;
volatile int t_possible = 0 ;

// button for source select 
#define SOURCE_SELECT 4 // GPIO 4, pin 6

// STATE TRACKING FOR AUDIO SOURCE
#define SOURCE_MIC 0
#define SOURCE_LINE 1
volatile int current_source = SOURCE_MIC;
volatile int request_source_switch; // flag to tell core 0 to switch 

///////////////////////// input state machine end /////////////////////////////////////////


///////////////////////////////tuning stuff////////////////////////////////
fix15 note_frequencies[12] = {float2fix15(466.16), // A#/Bb
                              float2fix15(261.63), // C
                              float2fix15(277.18), // C#/Db
                              float2fix15(293.66), // D
                              float2fix15(311.13), // D#/Eb
                              float2fix15(329.63), // E
                              float2fix15(349.23), // F
                              float2fix15(369.99), // F#/Gb
                              float2fix15(392), // G
                              float2fix15(415.30), // G#/Ab
                              float2fix15(440), // A
                              float2fix15(493.88) // B
                              } ; // based on octave 4 tuning, matches index of notes array

volatile int curr_tuning_note_idx = -1 ; // make it so no note is chosen initially
volatile fix15 curr_tuning_freq = 0 ;

// the value to multiply the center frequency by to get the bounds to draw the horizontal tuning bars
static fix15 cents_padding = float2fix15(1.0116194403) ; // based on 20 cents, 2^(cents/1200)
//fix15 n_cents_padding = float2fix15(-1.0116194403) ; // negative bound
// actual variables for the bounds of the lines
volatile fix15 lbound_y = 0 ; 
volatile fix15 ubound_y = 0 ;
volatile fix15 lbound_freq = 0 ;
volatile fix15 ubound_freq = 0 ;

volatile int tuning_flag = 0 ; // if tuning is enabled, stay high. else low (will ensure that the bars are only redrawn when tuning enabled)
volatile fix15 detected_freq = 0; // current frequency being played

///////////////////////////////tuning stuff////////////////////////////////

// converts the desired frequency into a y-value for the spectrogram
// essentially normalizes the frequency graph to fit the VGA screen
static inline int freq_2_spectro(fix15 frequency) {
  //float freq = fix2float15(frequency) ;
  fix15 bin = multfix15(frequency, float2fix15(NUM_SAMPLES / Fs)) ;

  //float norm_val =  (float)(SPECTRO_HEIGHT - 1) / (float)(NUM_SAMPLES >> 1) ; // mirrors the for loop 
  
  // inverse of the way its done in the FFT
  int y = (SPECTRO_HEIGHT - 1) - (fix2int15(bin) << 1) ;

  // clamp the upper and lower bounds of the graph
  if (y < 0) y = 0 ;
  else if (y >= SPECTRO_HEIGHT) y = SPECTRO_HEIGHT - 1 ;

  return SPECTRO_Y_START + y ;
}

// Read A/B as a 2-bit value: bit0 = A, bit1 = B, 0..3
static inline uint8_t read_encoder_terminals(void) {
    uint8_t a = gpio_get(ENCODER_PIN_A);
    uint8_t b = gpio_get(ENCODER_PIN_B);
    // if you use pull-ups, signals are active-low:
    // convert to logical "pressed = 1"
    a = !a;
    b = !b;
    return (a | (b << 1)) & 0x3; // combines a and b into one packet: 0xba, matches the source files
}

// Peforms an in-place FFT. For more information about how this
// algorithm works, please see https://vanhunteradams.com/FFT/FFT.html
void FFTfix(fix15 fr[], fix15 fi[]) {
    
    unsigned short m;   // one of the indices being swapped
    unsigned short mr ; // the other index being swapped (r for reversed)
    fix15 tr, ti ; // for temporary storage while swapping, and during iteration
    
    int i, j ; // indices being combined in Danielson-Lanczos part of the algorithm
    int L ;    // length of the FFT's being combined
    int k ;    // used for looking up trig values from sine table
    
    int istep ; // length of the FFT which results from combining two FFT's
    
    fix15 wr, wi ; // trigonometric values from lookup table
    fix15 qr, qi ; // temporary variables used during DL part of the algorithm
    
    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// BIT REVERSAL //////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Bit reversal code below based on that found here: 
    // https://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
    for (m=1; m<NUM_SAMPLES_M_1; m++) {
        // swap odd and even bits
        mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ... 
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= SHIFT_AMOUNT ;
        // don't swap that which has already been swapped
        if (mr<=m) continue ;
        // swap the bit-reveresed indices
        tr = fr[m] ;
        fr[m] = fr[mr] ;
        fr[mr] = tr ;
        ti = fi[m] ;
        fi[m] = fi[mr] ;
        fi[mr] = ti ;
    }
    //////////////////////////////////////////////////////////////////////////
    ////////////////////////// Danielson-Lanczos //////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    L = 1 ;
    // Log2 of number of samples, minus 1
    k = LOG2_NUM_SAMPLES - 1 ;
    // While the length of the FFT's being combined is less than the number 
    // of gathered samples . . .
    while (L < NUM_SAMPLES) {
        // Determine the length of the FFT which will result from combining two FFT's
        istep = L<<1 ;
        // For each element in the FFT's that are being combined . . .
        for (m=0; m<L; ++m) { 
            // Lookup the trig values for that element
            j = m << k ;                         // index of the sine table
            wr =  Sinewave[j + NUM_SAMPLES/4] ; // cos(2pi m/N)
            wi = -Sinewave[j] ;                 // sin(2pi m/N)
            wr >>= 1 ;                          // divide by two
            wi >>= 1 ;                          // divide by two
            // i gets the index of one of the FFT elements being combined
            for (i=m; i<NUM_SAMPLES; i+=istep) {
                // j gets the index of the FFT element being combined with i
                j = i + L ;
                // compute the trig terms (bottom half of the above matrix)
                tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]) ;
                ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]) ;
                // divide ith index elements by two (top half of above matrix)
                qr = fr[i]>>1 ;
                qi = fi[i]>>1 ;
                // compute the new values at each index
                fr[j] = qr - tr ;
                fi[j] = qi - ti ;
                fr[i] = qr + tr ;
                fi[i] = qi + ti ;
            }    
        }
        --k ;
        L = istep ;
    }
}

// Runs on core 0
static PT_THREAD (protothread_fft(struct pt *pt))
{
    // Indicate beginning of thread
    PT_BEGIN(pt) ;
    // printf("Starting capture\n") ;
    // Start the ADC channel
    dma_start_channel_mask((1u << sample_chan)) ;
    // Start the ADC
    adc_run(true) ;

    // Declare some static variables
    // static int height ;             // for scaling display
    // static float max_freqency ;     // holds max frequency
    static int i ;                  // incrementing loop variable

    // Statics for time
    static uint32_t begin_time;
    static int spare_time;
    static fix15 max_fr ;           // temporary variable for max freq calculation
    static int max_fr_dex ;         // index of max frequency

    // Write some text to VGA
    setTextColor(WHITE) ;
    setCursor(65, 0) ;
    setTextSize(1) ;

    // Will be used to write dynamic text to screen
    static char freqtext[40];

    // track current time (x-coord)
    static int y_pixel;
    static int freq_bin_index;
    static int scaled_mag;
    static float float_magnitude;

    while(1) {
        // get start time to facilitate clamping frame rate to 30 fps
        begin_time = time_us_32();

        // Wait for NUM_SAMPLES samples to be gathered
        // Measure wait time with timer. THIS IS BLOCKING
        dma_channel_wait_for_finish_blocking(sample_chan);

        // check if a source switch was requested by pressing source button
        if (request_source_switch) {
          // pause adc to prevent fifo write while switching
          adc_run(false);

          // empty the fifo
          adc_fifo_drain();
          
          // switch mux
          if (current_source == SOURCE_LINE) {
            adc_select_input(ADC_LINEIN_CHAN);
          }
          else {
            adc_select_input(ADC_AUDIO_CHAN);
          }
          // clear flag, start adc
          request_source_switch = 0;
          adc_run(true);
        }

        // Copy/window elements into a fixed-point array
        for (i=0; i<NUM_SAMPLES; i++) {
            fr[i] = multfix15(int2fix15((int)sample_array[i]), window[i]) ;
            fi[i] = (fix15) 0 ;
        }

        // Zero max frequency and max frequency index
        max_fr = 0 ;
        max_fr_dex = 0 ;

        // Restart the sample channel, now that we have our copy of the samples
        dma_channel_start(control_chan) ;

        // Compute the FFT
        FFTfix(fr, fi) ;

        // Find the magnitudes (alpha max plus beta min)
        for (i = 0; i < (NUM_SAMPLES>>1); i++) {  
            // get the approx magnitude
            fr[i] = abs(fr[i]); 
            fi[i] = abs(fi[i]);
            // reuse fr to hold magnitude
            fr[i] = max(fr[i], fi[i]) + 
                    multfix15(min(fr[i], fi[i]), zero_point_4); 

            // Keep track of maximum
            if (fr[i] > max_fr && i>4) {
                max_fr = fr[i] ;
                max_fr_dex = i ;
            }
        }
        // for the actual tuning
        // compute the dominant frequency (max magnitude)
        detected_freq = multfix15(int2fix15(max_fr_dex), float2fix15(Fs/NUM_SAMPLES)) ; // bin width = (Fs/NUM_SAMPLES)

        /////////////////// spectrogram //////////////////////////////////

        // draw new vertical time slice
        for (i = 0; i < SPECTRO_HEIGHT; i++) {
            // i = freq bin index (0-255)
            freq_bin_index = ((SPECTRO_HEIGHT-1) - i) >> 1;

            if (freq_bin_index < 0) freq_bin_index = 0;

            // skip first 5 bins (low-freq noise)
            if (i < 5) {
                fillRect(time_x, SPECTRO_Y_START + i, SCROLL_SPEED, 1, BLACK);
                // drawPixel(time_x, SPECTRO_Y_START + i, BLACK);
                continue;
            }
            
            float_magnitude = fix2float15(fr[freq_bin_index]);
            // scale magnitude 
            scaled_mag = (int)(float_magnitude * SCALING_FACTOR);
            //int scaled_mag = fix2int15(multfix15(fr[i], int2fix15(36)));
            scaled_mag = max(0, min(255, scaled_mag));

            scaled_mag = fix2int15(multfix15(fr[freq_bin_index], int2fix15(30)));

            if (scaled_mag < 0) scaled_mag = 0;
            if (scaled_mag > 255) scaled_mag = 255;
            // TODO: IF NEEDED, TUNE MAGS USING THIS INFO: https://vanhunteradams.com/Spectrogram/Spectrogram.html
            // map scaled mag to color
            // these thresholds need to be tuned
            // color spectrogram code
            short color;
            if(scaled_mag < 5) color = BLACK;
            else if (scaled_mag < 20) color = BLUE;
            else if (scaled_mag < 40) color = GREEN;
            else if (scaled_mag < 70) color = CYAN;
            else if (scaled_mag < 180) color = RED;
            else if (scaled_mag < 200) color = YELLOW;
            else color = WHITE;

            // calculate y-coord
            // plot bin zero at bottom, subtract i since vga origin is top left
            //int y_pixel = (SPECTRO_Y_START + SPECTRO_HEIGHT - 1) - i;
            y_pixel = SPECTRO_Y_START + i;

            // draw pixel for this (time, freq)
            //drawPixel(time_x, y_pixel, color);
            fillRect(time_x, y_pixel, SCROLL_SPEED, 1, color);
        }

        // update time_x coord for next frame
        time_x += SCROLL_SPEED;

        // if at end, wraparound
        if (time_x >= SPECTRO_X_END) {
            time_x = SPECTRO_X_START;
        }

        /////////////////// spectrogram end //////////////////////////////

        // spare_time = FRAME_RATE_30 - (time_us_32() - begin_time);
        // // check if framerate is met 
        // if (spare_time < 0) gpio_put(LED, 1);
        // else gpio_put(LED, 0);
        // PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
        PT_YIELD(pt);
        // don't exit this while loop
    }
    PT_END(pt) ;
}

// run on core1
static PT_THREAD (protothread_keypad_debounce(struct pt *pt)) 
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
    static short int prev_state;
    static int spare_time;
    static uint32_t begin_time;

    while(1) {
      begin_time = time_us_32();
        // Below code until else (i=-1) ; is checking what the button pressed is, then after will implement state machine
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
        prev_state = KEYPAD_STATE;

        // implementing this debouncing algorithm with switch for clarity rather than if statements
        // KEYPAD_STATE = NOT_PRESSED upon initialization
        switch (KEYPAD_STATE) {
        case NOT_PRESSED :
            if (i != -1) { // if the scan is valid
            KEYPAD_STATE = MAYBE_PRESSED ;
            possible = i ; 
            }
            break ;
        case MAYBE_PRESSED :
            if (i == possible) {
            KEYPAD_STATE = PRESSED ;
            // only alter stuff if in tuning mode
            if (tuning_flag) {
              strcpy(current_note, notes[i]) ; // write desired note to the desired_note_buffer

              curr_tuning_note_idx = i ; // set the i to the global index for the current note selected

              // current center frequency to tune to, accounts for shift in center_freq
              curr_tuning_freq = note_frequencies[curr_tuning_note_idx] + (int2fix15(440) - int2fix15(CENTER_FREQ)); 
              // based on the fact that the initial tuning array is centered on 440?
              // reset cursor and black out screen
                fillRect(SPECTRO_X_START, SPECTRO_Y_START, SPECTRO_WIDTH, SPECTRO_HEIGHT, BLACK) ;
                time_x = SPECTRO_X_START;
            }

            // calculate the y-values of the horizontal line for tuning
            ubound_freq = multfix15(curr_tuning_freq, cents_padding) ; // upper bound
            lbound_freq = divfix(curr_tuning_freq, cents_padding) ; // lower bound
            ubound_y = freq_2_spectro(ubound_freq) + 1 ;
            lbound_y = freq_2_spectro(lbound_freq) - 1;
            }
            else {
            KEYPAD_STATE = NOT_PRESSED ;
            }
            break ;
        case PRESSED :
            if (possible != i) { // if the button not the same, maybe not pressed
            KEYPAD_STATE = MAYBE_NOT_PRESSED ;
            }
            break ;
        case MAYBE_NOT_PRESSED :
            if (possible == i) { //  possible is 1 right now, so if it is high send to not pressed
            KEYPAD_STATE = PRESSED ;
            }
            else {
            KEYPAD_STATE = NOT_PRESSED ;
            }
            break ;
        if (KEYPAD_STATE != prev_state) {
          change_tuning_note = 1;
        }
        else {
          change_tuning_note = 0;
        }
        }
      // Loop timing
      spare_time = 30000 - (time_us_32() - begin_time);
      PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
    }
    // Indicate thread end
    PT_END(pt) ;
} // for keypad

// Input button debouncing for potentiometer state management
static PT_THREAD(protothread_POT_debouncing(struct pt *pt)) 
{
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;
  static short int prev_state = 0 ;
  
  while(1) {
    begin_time = time_us_32() ;

    int p = gpio_get(PIN_POT_BUTTON) ; // value of pot button press
    prev_state = POT_STATE;
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
            //printf("Button pressed") ;
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
    if (prev_state != POT_STATE) {
      knob_mode_change = 1;
    }
    else {
      knob_mode_change = 0;
    }
    
    // // delay in accordance with frame rate
    // Loop timing
    spare_time = 30000 - (time_us_32() - begin_time);
    PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
  }
  PT_END(pt) ;
} // thread for the debouncing

// Input button debouncing for potentiometer state management
static PT_THREAD(protothread_tune_debouncing(struct pt *pt)) 
{
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;
  unsigned int prev_state = NOT_PRESSED;
  
  while(1) {
    begin_time = time_us_32() ;

    int t = gpio_get(PIN_TUNE_BUTTON) ; // value of tune button press

    // implementing this debouncing algorithm with switch for clarity rather than if statements
    switch (TUNE_STATE) {
      case NOT_PRESSED :
        if (t == 0) { // if the button is low (pressed)
          TUNE_STATE = MAYBE_PRESSED ;
          prev_state = NOT_PRESSED;
          t_possible = t ; 
        }
        break ;
      case MAYBE_PRESSED :
        if (t == t_possible) {
            TUNE_STATE = PRESSED ;
            prev_state = MAYBE_PRESSED;
            PT_SEM_SIGNAL(pt, &tune_btn_pressed) ; // send flag, potFSM thread will be activated
            PT_YIELD_usec(200000);
            begin_time = time_us_32();
        }
        else {
          prev_state = TUNE_STATE;
          TUNE_STATE = NOT_PRESSED ;
        }
        break ;
      case PRESSED :
        if (t == 1) {
            TUNE_STATE = MAYBE_NOT_PRESSED ;
            prev_state = PRESSED;
            t_possible = t ;
        }
        break ;
      case MAYBE_NOT_PRESSED :
        prev_state = MAYBE_NOT_PRESSED;
        if (t == t_possible) { //  possible is 1 right now, so if it is high send to not pressed
          TUNE_STATE = NOT_PRESSED ;
        }
        else {
          TUNE_STATE = PRESSED ;
        }
        break ;
    }
    if (prev_state != TUNE_STATE) {
      change_tuning_state = 1;
    }
    else {
      change_tuning_state = 0;
    }
    
    // // delay in accordance with frame rate
    // Loop timing
    spare_time = 30000 - (time_us_32() - begin_time);
    PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
  }
  PT_END(pt) ;
} // thread for the debouncing

static PT_THREAD(protothread_source_select_debouncing(struct pt *pt))
{
    PT_BEGIN(pt);
    static int spare_time;
    static uint32_t begin_time;
    
    static short int prev_s_state = NOT_PRESSED;
    static short int s_possible = 0;
    static short int SOURCE_STATE = NOT_PRESSED;
    static int s_reading;

    while(1) {
        begin_time = time_us_32();

        s_reading = gpio_get(SOURCE_SELECT);

        // FSM
        switch (SOURCE_STATE) {
            case NOT_PRESSED :
                if (s_reading == 0) { // Button pressed
                    SOURCE_STATE = MAYBE_PRESSED ;
                    s_possible = s_reading ; 
                }
                break ;
            case MAYBE_PRESSED :
                if (s_reading == s_possible) {
                    SOURCE_STATE = PRESSED ;
                    
                    // toggle source var
                    if (current_source == SOURCE_MIC) {
                        current_source = SOURCE_LINE;
                    } else {
                        current_source = SOURCE_MIC;
                    }
                    // Request the hardware switch on Core 0 w/flag
                    request_source_switch = 1; 
                }
                else {
                    SOURCE_STATE = NOT_PRESSED ;
                }
                break ;
            case PRESSED :
                if (s_reading == 1) { // release
                    SOURCE_STATE = MAYBE_NOT_PRESSED ;
                    s_possible = s_reading ;
                }
                break ;
            case MAYBE_NOT_PRESSED :
                if (s_reading == s_possible) { 
                    SOURCE_STATE = NOT_PRESSED ;
                }
                else {
                    SOURCE_STATE = PRESSED ;
                }
                break ;
        }
        
        // Loop timing
        spare_time = 30000 - (time_us_32() - begin_time);
        PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
    }
    PT_END(pt);
}

// thread to manage state using debounced input
static PT_THREAD(protothread_potFSM(struct pt *pt))
{
  PT_BEGIN(pt) ;

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;
  // get standby to show up immediately at runtime
  strcpy(pot_state_buffer, "Standby");
  strcpy(pot_text_buffer, "");

  while(1) {
    // since this thread just cycles based on button presses, 
    // can just wait until the flag is incremented, 
    // and then restart STATE_1 in a loop without switch statements

    PT_SEM_WAIT(pt, &pot_btn_pressed);

    begin_time = time_us_32() ; // idk where to put this

    P_CYCLE_STATE = (P_CYCLE_STATE + 1) % 4 ; // states 0 through 3, will loop when state reaches 3

    switch (P_CYCLE_STATE) { // based on state display the currrent state and determine the function of the potentiometer
      case INIT :
        strcpy(pot_state_buffer, "Standby") ;
        strcpy(pot_text_buffer, "") ;
        pot_funct = INIT ;
      break ;
      case MOD_SCROLL_SPEED :
        strcpy(pot_state_buffer, "Speed:       ") ;
        sprintf(pot_text_buffer, "%d", SCROLL_SPEED) ;
        pot_funct = MOD_SCROLL_SPEED ;
      break ;
      case MOD_CENTER_FREQ :
        strcpy(pot_state_buffer, "Tuning Ref:  ") ;
        sprintf(pot_text_buffer, "%d", CENTER_FREQ) ;
        pot_funct = MOD_CENTER_FREQ ;
      break ;
      case MOD_SCALING_FACTOR :
        strcpy(pot_state_buffer, "Sensitivity: ") ;
        sprintf(pot_text_buffer, "%f", SCALING_FACTOR) ;
        pot_funct = MOD_SCALING_FACTOR ;
      break ;
    }

    // Loop timing
    spare_time = 30000 - (time_us_32() - begin_time);
    PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
  }

  PT_END(pt) ;
} // thread for potentiometer FSM

// thread to manage state using debounced input
static PT_THREAD(protothread_tuneFSM(struct pt *pt))
{
  PT_BEGIN(pt) ;

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;

  while(1) {
    // since this thread just cycles based on button presses, 
    // can just wait until the flag is incremented, 
    // and then restart STATE_1 in a loop without switch statements

    PT_SEM_WAIT(pt, &tune_btn_pressed);

    begin_time = time_us_32() ; // idk where to put this

    T_CYCLE_STATE = (T_CYCLE_STATE + 1) % 2 ; // states 0 through 1, will loop when state reaches 1

    switch (T_CYCLE_STATE) { // based on state display the currrent state and determine the function of the potentiometer
      case TUNE_DIS :
        // old tuning disabled flag
        // strcpy(tuning_state_buffer, "Tuning disabled") ;
        tune_funct = TUNE_DIS ;
        tuning_flag = 0 ; 
        // reset cursor and black out screen
        fillRect(SPECTRO_X_START, SPECTRO_Y_START, SPECTRO_WIDTH, SPECTRO_HEIGHT, BLACK) ;
        time_x = SPECTRO_X_START;
      break ;
      case TUNE_EN :
        // old tuning enabled flag
        // strcpy(tuning_state_buffer, "Tuning enabled") ;
        tune_funct = TUNE_EN ;
        tuning_flag = 1 ; // enable drawing for tuning lines
        // reset cursor and black out screen
        fillRect(SPECTRO_X_START, SPECTRO_Y_START, SPECTRO_WIDTH, SPECTRO_HEIGHT, BLACK) ;
        time_x = SPECTRO_X_START;
      break ;
    }

    // Loop timing
    spare_time = 30000 - (time_us_32() - begin_time);
    PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
  }

  PT_END(pt) ;
} // thread for tune FSM

// Encoder protothread
static PT_THREAD (protothread_encoder(struct pt *pt))
{
    PT_BEGIN(pt);
    
    // Variables for maintaining frame rate
    static int spare_time;
    static uint32_t begin_time;
    static uint8_t output;
    static enum encoder_action result;
    
    while(1) {
        begin_time = time_us_32();
        
        // Read encoder terminals
        output = read_encoder_terminals();
        
        // Update encoder state and get action
        result = encoder_debounced_full_step_update(&enc_state, output);

        static int prev_speed =  8;
        static int prev_center = 440;
        static float prev_scale = 6.0;
        
        switch (result) {
            case ENCODER_ACTION_TURN_CW:
                switch (pot_funct) {
                    case MOD_SCROLL_SPEED:
                        (SCROLL_SPEED < MAX_SCROLL_SPEED) ? SCROLL_SPEED++ : (SCROLL_SPEED = MAX_SCROLL_SPEED)  ; // clamp to max value
                        sprintf(pot_text_buffer, "%d", SCROLL_SPEED) ;
                        break;
                    case MOD_CENTER_FREQ:
                        (CENTER_FREQ < MAX_CENTER_FREQ) ? CENTER_FREQ++ : (CENTER_FREQ = MAX_CENTER_FREQ)  ;
                        sprintf(pot_text_buffer, "%d", CENTER_FREQ) ;
                        break ;
                    case MOD_SCALING_FACTOR:
                        (SCALING_FACTOR < MAX_SCALING_FACTOR) ? SCALING_FACTOR += 0.5 : (SCALING_FACTOR = MAX_SCALING_FACTOR) ;
                        sprintf(pot_text_buffer, "%f", SCALING_FACTOR) ;
                        break ;
                    case INIT :
                        //strcpy(pot_text_buffer, "") ;
                        break ;
                }
                break;
            case ENCODER_ACTION_TURN_CCW:
                switch (pot_funct) {
                    case MOD_SCROLL_SPEED:
                        (SCROLL_SPEED > MIN_SCROLL_SPEED) ? SCROLL_SPEED-- : (SCROLL_SPEED = MIN_SCROLL_SPEED) ; // clamp to min value
                        sprintf(pot_text_buffer, "%d", SCROLL_SPEED) ;
                        break;
                    case MOD_CENTER_FREQ:
                        (CENTER_FREQ > MIN_CENTER_FREQ) ? CENTER_FREQ-- : (CENTER_FREQ = MIN_CENTER_FREQ) ;
                        sprintf(pot_text_buffer, "%d", CENTER_FREQ) ;
                        break ;
                    case MOD_SCALING_FACTOR:
                        (SCALING_FACTOR > MIN_SCALING_FACTOR) ? SCALING_FACTOR -= 0.5 : (SCALING_FACTOR = MIN_SCALING_FACTOR);
                        sprintf(pot_text_buffer, "%f", SCALING_FACTOR) ;
                        break ;
                    case INIT :
                        
                        break ;
                }
                break;
            default:
                break;
        }
        if ((prev_center != CENTER_FREQ) || (prev_speed != SCROLL_SPEED) || (prev_scale != SCALING_FACTOR)) {
          knob_value_change = 1;
        }
        else {
          knob_value_change = 0;
        }
        
    // Loop timing
    spare_time = 30000 - (time_us_32() - begin_time);
    PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
    }
    
    PT_END(pt);
}

// on core1
static PT_THREAD (protothread_noncrit_vga(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    static int spare_time;
    static uint32_t begin_time;
    static short int prev_source_drawn = -1;
    static short int prev_tuning_flag = -1;
    static short int prev_note_idx = -1;
    static short int prev_p_state = -1;
    static short int prev_speed = -1;
    static short int prev_center = -1;
    static short int prev_scale = -1.0;

    setTextColor(WHITE) ;
    setTextSize(1) ;
    char concat_pot_state[50] ;
    setCursor(300, 10);
    writeString("Input: Mic    ");
    setCursor(520, 10);
    sprintf(concat_pot_state, "%s%s", pot_state_buffer, pot_text_buffer) ;
    // make sure standby shows up immediately
    writeString(concat_pot_state) ;

    while(1) {
        begin_time = time_us_32();

        // TUNING LOGIC ///////////////////////////////////////////
        // write note to desired_note_buffer
        sprintf(desired_note_buffer, "Tuning To: %s", current_note);
        if (tuning_flag != prev_tuning_flag || curr_tuning_note_idx != prev_note_idx) {
          fillRect(10, 10, 150, 10, BLACK);

          setCursor(10, 10);
          if (tuning_flag) {
            sprintf(desired_note_buffer, "Tuning to: %s", current_note);
            writeString(desired_note_buffer);
          }
          else {
            writeString("Tuning Disabled");
          }

          prev_tuning_flag = tuning_flag;
          prev_note_idx = curr_tuning_note_idx;
        }
        
        
        if(tuning_flag == 1) {
          if ((detected_freq <= ubound_freq) && (detected_freq >= lbound_freq)) {
            drawHLine(SPECTRO_X_START, ubound_y, SPECTRO_WIDTH, GREEN) ; // upper
            drawHLine(SPECTRO_X_START, lbound_y, SPECTRO_WIDTH, GREEN) ; // lower
          }
          else {
            drawHLine(SPECTRO_X_START, ubound_y, SPECTRO_WIDTH, RED) ; // upper
            drawHLine(SPECTRO_X_START, lbound_y, SPECTRO_WIDTH, RED) ; // lower
          }      
        }
        ///////// TUNING LOGIC END ///////////////////////////////////////

        // SOURCE ///////////////////////////////////////////////////
        // show source indicator
        if (current_source != prev_source_drawn) {
          fillRect(300, 10, 100, 10, BLACK);
          setCursor(300, 10);
          if (current_source == SOURCE_MIC) {
            writeString("Input: Mic    ");
          }
          else {
            writeString("Input: Line-in");
          }
          prev_source_drawn = current_source;
        }
        // SOURCE END /////////////////////////////////////////////////

        ////////// POT /////////////////////////////////////////////
        if (P_CYCLE_STATE != prev_p_state || 
            SCROLL_SPEED != prev_speed ||
            CENTER_FREQ != prev_center ||
            SCALING_FACTOR != prev_scale) {

          // clear area
          setCursor(520, 10);
          fillRect(520, 10, 120, 10, BLACK);
          // MAKE STRING ON CURRENT STATE
          sprintf(concat_pot_state, "%s%s", pot_state_buffer, pot_text_buffer) ;
          writeString(concat_pot_state);
          // update trackers
          prev_p_state = P_CYCLE_STATE;
          prev_speed = SCROLL_SPEED;
          prev_center = CENTER_FREQ;
          prev_scale = SCALING_FACTOR;
        }
        // display potentiometer state
        // if changing state, then blank with rect
        ///////// POT END /////////////////////////////////////////////
      
      // Loop timing
      spare_time = 30000 - (time_us_32() - begin_time);
      PT_YIELD_usec(spare_time > 0 ? spare_time : 0);
    }
    
    // Indicate thread end
    PT_END(pt) ;
} // for text output on VGA


// Core 1 entry point (main() for core 1)
void core1_entry() {
    // Add and schedule threads
    pt_add_thread(protothread_keypad_debounce) ;
    pt_add_thread(protothread_POT_debouncing) ;
    pt_add_thread(protothread_tune_debouncing) ;
    pt_add_thread(protothread_potFSM) ;
    pt_add_thread(protothread_tuneFSM) ;
    //pt_add_thread(protothread_pot_ADC) ;
    pt_add_thread(protothread_noncrit_vga) ;
    pt_add_thread(protothread_encoder);
    pt_add_thread(protothread_source_select_debouncing);
    pt_schedule_start ;
}

// Core 0 entry point
int main() {
    // set overclock
    set_sys_clock_khz(CLOCK_SPEED, true) ;
    // Initialize stdio
    stdio_init_all();

    // Initialize the VGA screen
    initVGA() ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    ///////////////////////////////////////////////////////////////////////////////
    // ============================== ADC CONFIGURATION ==========================
    //////////////////////////////////////////////////////////////////////////////
    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(ADC_AUDIO_PIN); // for audio
    adc_gpio_init(ADC_LINEIN_PIN); // for the linein

    // Initialize the ADC harware
    // (resets it, enables the clock, spins until the hardware is ready)
    adc_init() ;

    // Select analog mux input (0...3 are GPIO 26, 27, 28, 29; 4 is temp sensor)
    adc_select_input(ADC_AUDIO_CHAN) ;

    // Setup the FIFO
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock. This is setup
    // to grab a sample at 10kHz (48Mhz/10kHz - 1)
    adc_set_clkdiv(ADCCLK/Fs);


    // Populate the sine table and Hann window table
    int ii;
    for (ii = 0; ii < NUM_SAMPLES; ii++) {
        Sinewave[ii] = float2fix15(sin(6.283 * ((float) ii) / (float)NUM_SAMPLES));
        window[ii] = float2fix15(0.5 * (1.0 - cos(6.283 * ((float) ii) / ((float)NUM_SAMPLES))));
    }

    /////////////////////////////////////////////////////////////////////////////////
    // ============================== ADC DMA CONFIGURATION =========================
    /////////////////////////////////////////////////////////////////////////////////

    sample_chan = dma_claim_unused_channel(true);
    control_chan = dma_claim_unused_channel(true);

    // Channel configurations
    dma_channel_config c2 = dma_channel_get_default_config(sample_chan);
    dma_channel_config c3 = dma_channel_get_default_config(control_chan);


    // ADC SAMPLE CHANNEL
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&c2, DREQ_ADC);
    // Configure the channel
    dma_channel_configure(sample_chan,
        &c2,            // channel config
        sample_array,   // dst
        &adc_hw->fifo,  // src
        NUM_SAMPLES,    // transfer count
        false            // don't start immediately
    );

    // CONTROL CHANNEL
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);      // 32-bit txfers
    channel_config_set_read_increment(&c3, false);                // no read incrementing
    channel_config_set_write_increment(&c3, false);               // no write incrementing
    channel_config_set_chain_to(&c3, sample_chan);                // chain to sample chan

    dma_channel_configure(
        control_chan,                         // Channel to be configured
        &c3,                                // The configuration we just created
        &dma_hw->ch[sample_chan].write_addr,  // Write address (channel 0 read address)
        &sample_address_pointer,                   // Read address (POINTER TO AN ADDRESS)
        1,                                  // Number of transfers, in this case each is 4 byte
        false                               // Don't start immediately.
    );

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

    // debouncing GPIO initialization
    gpio_init(PIN_POT_BUTTON) ;
    gpio_set_dir(PIN_POT_BUTTON, GPIO_IN); // set GPIO to input
    gpio_pull_up(PIN_POT_BUTTON) ; // drive the pin normally high, if button pressed will be low

    gpio_init(PIN_TUNE_BUTTON) ;
    gpio_set_dir(PIN_TUNE_BUTTON, GPIO_IN); // set GPIO to input
    gpio_pull_up(PIN_TUNE_BUTTON) ; // drive the pin normally high, if button pressed will be low

    gpio_init(SOURCE_SELECT);
    gpio_set_dir(SOURCE_SELECT, GPIO_IN);
    gpio_pull_up(SOURCE_SELECT);

    // Initialize encoder pins
    gpio_init(ENCODER_PIN_A);
    gpio_set_dir(ENCODER_PIN_A, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_A);

    gpio_init(ENCODER_PIN_B);
    gpio_set_dir(ENCODER_PIN_B, GPIO_IN);
    gpio_pull_up(ENCODER_PIN_B);

    // Initialize encoder state with current terminal state
    uint8_t initial = read_encoder_terminals();
    encoder_debounced_half_step_init(&enc_state, initial);

    // initialize semiphores
    PT_SEM_INIT(&pot_btn_pressed, 0);
    PT_SEM_INIT(&tune_btn_pressed, 0);
    
    // Launch core 1
    multicore_launch_core1(core1_entry);

    // Add and schedule core 0 threads
    pt_add_thread(protothread_fft) ;
    pt_schedule_start ;

}