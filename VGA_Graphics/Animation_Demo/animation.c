/**
 * Hunter Adams  (vha3@cornell.edu)
 * Jack Chaney   (jbc282@cornell.edu)
 * Arielle Huang (aph74@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
  - GPIO 15 ---> Button for user input
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - GPIO 26 ---> POTENTIOMETER FOR USER INPUT
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *  - ADC0
 */

// Include the VGA grahics library
#include "vga16_graphics_v2.h"
// Include standard libraries
#include <hardware/gpio.h>
#include <hardware/timer.h>
#include <hardware/adc.h>
#include <iso646.h>
#include <pico/error.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))

// Wall detection
#define hitBottom(b) (b>int2fix15(400))
#define hitTop(b) (b<int2fix15(25))
#define hitLeft(a) (a<int2fix15(10))
#define hitRight(a) (a>int2fix15(630))

// uS per frame
#define FRAME_RATE 33000

// clock speed
#define CLOCK_SPEED 250000

// lED PIN
#define LED 25

// ADC
#define ADC_PIN 26
#define ADC_CHAN 0

// GRAVITY FRACTION FOR BALL
#define GRAVITY_FRACTION 0.75

// BOUNCINESS
//#define BOUNCINESS 0.5
#define MAX_BOUNCE 1
volatile float bounciness = 0.5 ; // initialize bounce
char bounciness_buffer[8] ; // buffer for snprintf 
volatile float prev_bounciness ;
//volatile float prev_bounciness2 ;

// SPI configs for the DAC
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Samples per period in sine table
#define SINE_TABLE_SIZE 256

// Sine table and DAC data table
int raw_sin[SINE_TABLE_SIZE] ;
unsigned short dac_data[SINE_TABLE_SIZE] ;
unsigned short* addr_pointer = &dac_data[0] ;

// DAC config bits (channel A, 1x gain, active)
#define DAC_CONFIG_CHAN_A 0b0011000000000000 

// DMA channel vars
int data_chan ;
int ctrl_chan ;

// the color of the balls
char color = RED ;

// balls
#define BALL_SPAWN_X 320
#define BALL_SPAWN_Y 50
#define BALL_R  2 // BALL RADIUS
#define MAX_BALLS 2510 
volatile unsigned int active_balls = 1 ;
volatile unsigned int prev_active_balls ; // keeping track of number of balls 
volatile unsigned int fallen_balls = 0 ;
char active_balls_buffer[16] ; // buffer for snprintf 
char fallen_balls_buffer[20] ;

// peg on core 0
#define PEG_R 6 // peg radius
#define PEG_OFFSET 40

// top peg coords (used in generating the galton board)
fix15 peg0_x = int2fix15(320) ;
fix15 peg0_y = int2fix15(100) ;

// sum of squares for collisions
#define R_SUM_SQ int2fix15((BALL_R + PEG_R) * (BALL_R + PEG_R))

// timing variables
// static uint32_t begin_time ; // We needed local statics to do fps led
static uint32_t global_start_time_us;   // When the timer started
char curr_time_buffer[20] ; // for converting the unint time to buffer
volatile int updated_values = 0 ;

// histogram variables
#define NUM_BINS 15
int bins[NUM_BINS] = {0} ;
//int bin_idx = 0 ;
int new_heights[NUM_BINS] = {0} ; // the height of the histograms normalized to the max height
int old_heights[NUM_BINS] = {0} ; // old height of the bar

// state machine variables - debouncing
// states
#define NOT_PRESSED 0
#define MAYBE_PRESSED 1
#define PRESSED 2
#define MAYBE_NOT_PRESSED 3
volatile int possible = 1 ; // init high
volatile unsigned int STATE_0 = NOT_PRESSED ;

// state machine variables - button pressing
#define PIN_BUTTON 15 // gpio 15 (pin 20), use ground pin 18
// states
#define INIT 0
#define ADJUST_BALLS 1 // to adjust the number of balls with potentiometers
#define ADJUST_BOUNCE 2 // to adjust the bounciness
volatile unsigned int STATE_1 = INIT ; // initalize the state of this fsm
volatile int btn_pressed = 0 ;
char state_buffer[17] ; // buffer for snprintf for the current state
volatile int pot_funct = INIT ; // function of potentiometer, initialized at INIT, can be ADJUST_BALLS, ADJUST_BOUNCE

// Ball struct, stores all important ball info
struct ball {
  fix15 x ;
  fix15 y ;
  fix15 vx ;
  fix15 vy ;
  short last_peg ;
} ;

// peg struct for peg coords
struct peg {
  fix15 x ;
  fix15 y ;
} ;

// Make the peg array, and make two arrays for balls, one for each core
struct peg pegs[136] ;
struct ball balls[MAX_BALLS] ;
struct ball balls2[MAX_BALLS] ;

// make the thunk noise
static inline void audio_init() {
  // Init/config SPI
  spi_init(SPI_PORT, 20000000) ;
  spi_set_format(SPI_PORT, 16, 0, 0, 0) ;
  gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI) ;
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

  // Build the sine wave and DAC data tables
  for (int i = 0; i < SINE_TABLE_SIZE; i++) {
    // 12 bit sine wave, in range [0, 4095]
    raw_sin[i] = (int)(2047 * sin((float)i*6.383f / (float)SINE_TABLE_SIZE) + 2047) ;
    
    // Mask sine wave with config bits into dac_data
    dac_data[i] = DAC_CONFIG_CHAN_A | (raw_sin[i] & 0x0fff) ;
  }
  
  // Claim two DMA channels
  data_chan = dma_claim_unused_channel(true) ;
  ctrl_chan = dma_claim_unused_channel(true) ;

  // Configure the control channel
  // write the starting address of sound data into data channel's read address register
  dma_channel_config ctrl_conf = dma_channel_get_default_config(ctrl_chan) ;
  channel_config_set_transfer_data_size(&ctrl_conf, DMA_SIZE_32) ;
  channel_config_set_read_increment(&ctrl_conf, false) ;
  channel_config_set_write_increment(&ctrl_conf, false) ;
  channel_config_set_chain_to(&ctrl_conf, data_chan) ; //trigger data_chan when done

  dma_channel_configure(
    ctrl_chan, &ctrl_conf, 
    &dma_hw->ch[data_chan].read_addr, // write addr
    &addr_pointer, // read addr
    1, false
  ) ;

  // configure the data channel, to stream data to spi port
  dma_channel_config data_conf = dma_channel_get_default_config(data_chan) ;
  channel_config_set_transfer_data_size(&data_conf, DMA_SIZE_16) ;
  channel_config_set_read_increment(&data_conf, true) ;
  channel_config_set_write_increment(&data_conf, false) ;
  channel_config_set_dreq(&data_conf, spi_get_dreq(SPI_PORT, true)) ; // keep pace wth spi

  dma_channel_configure(
    data_chan, &data_conf, 
    &spi_get_hw(SPI_PORT)->dr,
    dac_data,
    SINE_TABLE_SIZE, 
    false
  ) ;
}
 // play the sound (see if the channels are available)
static inline void play_hit_sound() {
  // only play if channels clear
  if (dma_channel_is_busy(data_chan) || dma_channel_is_busy(ctrl_chan)) {
    return ;
  }
  dma_channel_start(ctrl_chan) ;
}

// intialize the balls (core0) into the array of balls and say no pegs hit
static inline void init_balls() {
  for (int i = 0; i < MAX_BALLS; i++)  {
    balls[i].x = int2fix15(BALL_SPAWN_X) ;
    balls[i].y = int2fix15(BALL_SPAWN_Y) ;
    float random_float = (float)rand() / (float)RAND_MAX ;
    float random_vx_float = (random_float * 0.4f) - 0.2f ;
    balls[i].vx = float2fix15(random_vx_float) ;
    balls[i].vy = int2fix15(0) ;
    balls[i].last_peg = -1 ;
  }
}

// intialize the balls (core1) into the array of balls and say no pegs hit
static inline void init_balls2() {
  for (int i = 0; i < MAX_BALLS; i++)  {
    balls2[i].x = int2fix15(BALL_SPAWN_X) ;
    balls2[i].y = int2fix15(BALL_SPAWN_Y) ;
    float random_float = (float)rand() / (float)RAND_MAX ;
    float random_vx_float = (random_float * 0.4f) - 0.2f ;
    balls2[i].vx = float2fix15(random_vx_float) ;
    balls2[i].vy = int2fix15(0) ;
    balls2[i].last_peg = -1 ;
  }
}

// create a ball
static inline void spawnBall(short ball_idx) {
  // Start ball in center of screen on spawn point
  balls[ball_idx].x = int2fix15(BALL_SPAWN_X) ;
  balls[ball_idx].y = int2fix15(BALL_SPAWN_Y) ;

  // re init the pegs 
  balls[ball_idx].last_peg = -1 ;
  
  // Generate random float between 0.0 and 1.0 for small initial vx
  float random_float = (float)rand() / (float)RAND_MAX ;
  
  // Scale float to range of [-0.2, 0.2]
  float random_vx_float = (random_float * 0.4f) - 0.2f ;
  
  // Convert the float to a fixed-point number and assign it to vx
  balls[ball_idx].vx = float2fix15(random_vx_float) ;

  // Moving down
  balls[ball_idx].vy = int2fix15(0) ;
}

// create a ball (inline makes it run faster)
static inline void spawnBall2(short ball_idx) {
  // Start ball in center of screen on spawn point
  balls2[ball_idx].x = int2fix15(BALL_SPAWN_X) ;
  balls2[ball_idx].y = int2fix15(BALL_SPAWN_Y) ;

  // re init the pegs 
  balls2[ball_idx].last_peg = -1 ;
  
  // Generate random float between 0.0 and 1.0 for small initial vx
  float random_float = (float)rand() / (float)RAND_MAX ;
  
  // Scale float to range of [-0.2, 0.2]
  float random_vx_float = (random_float * 0.4f) - 0.2f ;
  
  // Convert the float to a fixed-point number and assign it to vx
  balls2[ball_idx].vx = float2fix15(random_vx_float) ;

  // Moving down
  balls2[ball_idx].vy = int2fix15(0) ;
}

// create the galton board pegs
static inline void generateBoard() {
  int rows = 16;
  fix15 xi = peg0_x ;
  fix15 yi = peg0_y ;
  fix15 dy = int2fix15(19) ; // the x and y distance from center of top peg
  fix15 dx = int2fix15(38) ;
  fix15 dx_half = int2fix15(19) ;
  fix15 x_new ;
  int peg_idx = 0 ;
  
  for (int i = 0; i < rows; i++) {
    yi = peg0_y + multfix15(int2fix15(i), dy) ; // move the row down based on the first row

    for (int j = 0; j <= i; j++) {
      if ( peg_idx < 136 ) {
        // even row x spacing
        if (i % 2 == 0) {
          x_new = xi - multfix15(int2fix15(i), dx_half) + multfix15(int2fix15(j), dx) ;
        }
        // odd row x spacing
        else {
          x_new = xi - multfix15(int2fix15(i), dx_half) + multfix15(int2fix15(j), dx);
        }
      }

      pegs[peg_idx].x = x_new ;
      pegs[peg_idx].y = yi ;

      drawPeg(fix2int15(x_new), fix2int15(yi)) ; // draws the pegs for every row

      peg_idx++ ;
    }
  }
}

// Detect wallstrikes, update velocity and position
static inline void wallsAndEdges(short ball_idx) {
  // Reverse direction if we've hit a wall
  if (hitTop(balls[ball_idx].y)) {
    balls[ball_idx].vy = multfix15(float2fix15(bounciness), (-balls[ball_idx].vy)) ;
    balls[ball_idx].y  = (balls[ball_idx].y + int2fix15(5)) ;
  }
  if (hitBottom(balls[ball_idx].y)) {
    int x_pos = fix2int15(balls[ball_idx].x) ; // extract x position of ball
    //bin_idx = fix2int15(divfix(int2fix15((x_pos - 10) * NUM_BINS), 620));
    int bin_idx = ((x_pos - 10) * NUM_BINS / 620) ; // normalize the bin index to the size of the arena
    if (bin_idx < 0) { // edge case for the left side of board
      bin_idx = 0 ;
    }
    if (bin_idx >= NUM_BINS) { // edge case for the right side of the board
      bin_idx = NUM_BINS - 1;
    }
    bins[bin_idx]++; // increment number of balls in the bin
    spawnBall(ball_idx) ; 
    fallen_balls += 1 ;
  }
  // Mirror ball movement if hit edge of arena
  if (hitRight(balls[ball_idx].x)) {
    balls[ball_idx].vx = multfix15(float2fix15(bounciness), (-balls[ball_idx].vx)) ;
    balls[ball_idx].x  = (balls[ball_idx].x - int2fix15(5)) ;
  }
  if (hitLeft(balls[ball_idx].x)) {
    balls[ball_idx].vx = multfix15(float2fix15(bounciness), (-balls[ball_idx].vx)) ;
    balls[ball_idx].x  = (balls[ball_idx].x + int2fix15(5)) ;
  }
}

static inline void hitPeg(short ball_idx, short peg_idx) 
{
  // because this is the ball radius + peg radius, 
  // required to see if the x and y distances are less than the collision distance
  // computing difference in position
  // Differences in position
  fix15 dx = balls[ball_idx].x - pegs[peg_idx].x ;
  fix15 dy = balls[ball_idx].y - pegs[peg_idx].y ;

  // Quick bounding-box check (faster than sqrt every time)
  if ((absfix15(dx) < int2fix15(BALL_R + PEG_R)) && (absfix15(dy) < int2fix15(BALL_R + PEG_R))) {
    // Full distance check
    fix15 dist_sq = multfix15(dx, dx) + multfix15(dy, dy) ;
    fix15 min_dist = int2fix15(BALL_R + PEG_R) ;

    if (dist_sq < multfix15(min_dist, min_dist)) { 
      fix15 distance = sqrtfix(dist_sq) ;

      if (distance < 1) return ; // avoid dividing by 0

      // Normalized vector from peg to ball
      fix15 normal_x = divfix(dx, distance) ;
      fix15 normal_y = divfix(dy, distance) ;

      // Dot product (ball velocity and normal)
      fix15 dotprod = multfix15(normal_x, balls[ball_idx].vx) + multfix15(normal_y, balls[ball_idx].vy) ;

      // Intermediate term per pseudocode
      fix15 intermediate = -2 * dotprod ;

      if (intermediate > 0) {
        // Teleport the ball just outside peg surface
        balls[ball_idx].x = pegs[peg_idx].x + multfix15(normal_x, (min_dist + int2fix15(1))) ;
        balls[ball_idx].y = pegs[peg_idx].y + multfix15(normal_y, (min_dist + int2fix15(1))) ;

        // Update ball velocity
        balls[ball_idx].vx = balls[ball_idx].vx + multfix15(normal_x, intermediate) ;
        balls[ball_idx].vy = balls[ball_idx].vy + multfix15(normal_y, intermediate) ;

        // Lose some energy (BOUNCINESS factor)
        balls[ball_idx].vx = multfix15(float2fix15(bounciness), balls[ball_idx].vx) ;
        balls[ball_idx].vy = multfix15(float2fix15(bounciness), balls[ball_idx].vy) ;

        // Play sound if new peg struck
        if (balls[ball_idx].last_peg != peg_idx) {
          balls[ball_idx].last_peg = peg_idx ;
          play_hit_sound() ;
        }
      }
    } 
  }  
}

// Check collisions by finding where the ball is in board and then only checking two rows localized around position
static inline void check_collisions_opt(short ball_idx) 
{
  // Ball's y position determines its approximate row
  // Pegs are 19px apart vertically, starting at y=100
  short curr_row = fix2int15(
    divfix((balls[ball_idx].y - int2fix15(100)), int2fix15(19))) ;
  
  // Safety check for bounds
  if (curr_row < 0) curr_row = 0 ;
  if (curr_row > 15) curr_row = 15;
  
  // Calculate starting peg idx for curr_row and the next
  short start_peg = (curr_row * (curr_row + 1)) / 2 ;
  short end_peg   = ((curr_row + 2) * (curr_row + 3)) / 2 ;
   
  // bounds check
  if (end_peg > 136) end_peg = 136 ;
  
  // Only check pegs in two rows
  for (short peg = start_peg; peg < end_peg; peg++) {
    hitPeg(ball_idx, peg);
  }
}

// Because our initial implementation did not take an array as an argument, and we implemented multicore in lab, it was much easier to just make copy methods for the core1 balls as a quick and dirty way to almost double our balls number!
static inline void wallsAndEdges2(short ball_idx) {
  // Reverse direction if we've hit a wall
  if (hitTop(balls2[ball_idx].y)) {
    balls2[ball_idx].vy = multfix15(float2fix15(bounciness), (-balls2[ball_idx].vy)) ;
    balls2[ball_idx].y  = (balls2[ball_idx].y + int2fix15(5)) ;
  }
  if (hitBottom(balls2[ball_idx].y)) {
    int x_pos = fix2int15(balls2[ball_idx].x) ; // extract x position of ball
    //bin_idx = fix2int15(divfix(int2fix15((x_pos - 10) * NUM_BINS), 620));
    int bin_idx = ((x_pos - 10) * NUM_BINS / 620) ; // normalize the bin index to the size of the arena
    if (bin_idx < 0) { // edge case for the left side of board
      bin_idx = 0 ;
    }
    if (bin_idx >= NUM_BINS) { // edge case for the right side of the board
      bin_idx = NUM_BINS - 1;
    }
    bins[bin_idx]++; // increment number of balls in the bin
    spawnBall2(ball_idx) ; 
    fallen_balls += 1 ;
  }
  if (hitRight(balls2[ball_idx].x)) {
    balls2[ball_idx].vx = multfix15(float2fix15(bounciness), (-balls2[ball_idx].vx)) ;
    balls2[ball_idx].x  = (balls2[ball_idx].x - int2fix15(5)) ;
  }
  if (hitLeft(balls2[ball_idx].x)) {
    balls2[ball_idx].vx = multfix15(float2fix15(bounciness), (-balls2[ball_idx].vx)) ;
    balls2[ball_idx].x  = (balls2[ball_idx].x + int2fix15(5)) ;
  }
}

static inline void hitPeg2(short ball_idx, short peg_idx) 
{
  // because this is the ball radius + peg radius, 
  // required to see if the x and y distances are less than the collision distance
  // computing difference in position
  // Differences in position
  fix15 dx = balls2[ball_idx].x - pegs[peg_idx].x ;
  fix15 dy = balls2[ball_idx].y - pegs[peg_idx].y ;

  // Quick bounding-box check (faster than sqrt every time)
  if ((absfix15(dx) < int2fix15(BALL_R + PEG_R)) && (absfix15(dy) < int2fix15(BALL_R + PEG_R))) {
    // Full distance check
    fix15 dist_sq = multfix15(dx, dx) + multfix15(dy, dy) ;
    fix15 min_dist = int2fix15(BALL_R + PEG_R) ;

    if (dist_sq < multfix15(min_dist, min_dist)) { 
      fix15 distance = sqrtfix(dist_sq) ;

      if (distance < 1) return ; // avoid dividing by 0

      // Normalized vector from peg to ball
      fix15 normal_x = divfix(dx, distance) ;
      fix15 normal_y = divfix(dy, distance) ;

      // Dot product (ball velocity and normal)
      fix15 dotprod = multfix15(normal_x, balls2[ball_idx].vx) + multfix15(normal_y, balls2[ball_idx].vy) ;

      // Intermediate term per pseudocode
      fix15 intermediate = -2 * dotprod ;

      if (intermediate > 0) {
        // Teleport the ball just outside peg surface
        balls2[ball_idx].x = pegs[peg_idx].x + multfix15(normal_x, (min_dist + int2fix15(1))) ;
        balls2[ball_idx].y = pegs[peg_idx].y + multfix15(normal_y, (min_dist + int2fix15(1))) ;

        // Update ball velocity
        balls2[ball_idx].vx = balls2[ball_idx].vx + multfix15(normal_x, intermediate) ;
        balls2[ball_idx].vy = balls2[ball_idx].vy + multfix15(normal_y, intermediate) ;

        // Lose some energy (BOUNCINESS factor)
        balls2[ball_idx].vx = multfix15(float2fix15(bounciness), balls2[ball_idx].vx) ;
        balls2[ball_idx].vy = multfix15(float2fix15(bounciness), balls2[ball_idx].vy) ;

        // Play sound if new peg struck
        if (balls2[ball_idx].last_peg != peg_idx) {
          balls2[ball_idx].last_peg = peg_idx ;
          play_hit_sound() ;
        }
      }
    } 
  }  
}

static inline void check_collisions_opt2(short ball_idx) 
{
  // Ball's y position determines its approximate row
  // Pegs are 19px apart vertically, starting at y=100
  short curr_row = fix2int15(
    divfix((balls2[ball_idx].y - int2fix15(100)), int2fix15(19))) ;
  
  // Safety check for bounds
  if (curr_row < 0) curr_row = 0 ;
  if (curr_row > 15) curr_row = 15;
  
  // Calculate starting peg idx for curr_row and the next
  short start_peg = (curr_row * (curr_row + 1)) / 2 ;
  short end_peg   = ((curr_row + 2) * (curr_row + 3)) / 2 ;
   
  // bounds check
  if (end_peg > 136) end_peg = 136 ;
  
  // Only check pegs in two rows
  for (short peg = start_peg; peg < end_peg; peg++) {
    hitPeg2(ball_idx, peg);
  }
}

// Animation on core 0, animating the ball bouncing on peg
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static uint32_t begin_time ;
    static int spare_time ;
    for (int u = 0; u < active_balls; u++) {
      spawnBall(u) ;
    }
    
    // spawn the board
    generateBoard() ;

    // set variables for comparison for resetting histogram
    prev_active_balls = active_balls ;
    prev_bounciness = bounciness ;
    
    // initialize the VGA text
    setTextColor(RED) ;
    setTextSize(1) ;

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;

      generateBoard() ;
      // draw box over old text to erase
      fillRect(30, 40, 170, 50, BLACK);
      // Generate the text for the screen
      setCursor(30, 40);
      writeString("Balls on screen: ") ;
      snprintf(active_balls_buffer, 16, "%d", (2 * active_balls)); // converts int to string
      writeString(active_balls_buffer) ;
      setCursor(30, 50);
      writeString("Bounciness: ") ;
      snprintf(bounciness_buffer, 8, "%.2f", bounciness); // converts int to string
      writeString(bounciness_buffer) ;
      setCursor(30, 60);
      writeString("Balls fallen through: ") ;
      snprintf(fallen_balls_buffer, 20, "%d", fallen_balls); // converts int to string
      writeString(fallen_balls_buffer) ;
      setCursor(30, 70);
      writeString("Time since boot: ") ;
      snprintf(curr_time_buffer, 20, "%u", ((time_us_32() - global_start_time_us) / 1000000)); // converts uint time to seconds
      writeString(curr_time_buffer) ;

      // display the current state
      setCursor(30, 80);
      writeString("State: ") ;
      writeString(state_buffer) ;

      // update ball's position and velocity
      // need to do for every ball, then for every peg... nested for loops???
      for (int ball = 0; ball < MAX_BALLS; ball++) {
        // erase ball at old position
        maskBall(fix2int15(balls[ball].x), fix2int15(balls[ball].y)) ;
        if (ball < active_balls) {
          // Apply gravity to the ball
          balls[ball].vy = balls[ball].vy + float2fix15(GRAVITY_FRACTION) ;
          balls[ball].x += balls[ball].vx ;
          balls[ball].y += balls[ball].vy ;
          
          check_collisions_opt(ball);
          
          // Update the walls and edges, and handle wall collisions
          wallsAndEdges(ball) ;

          // draw the ball at its new position
          drawBall(fix2int15(balls[ball].x), fix2int15(balls[ball].y)) ;
          
        }
      }

      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

      // Check if framerate is met
      if (spare_time < 0) {
        gpio_put(LED, 1) ; 
      }
      else {
        gpio_put(LED, 0) ;
      }
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Animation on core 1, animating the ball bouncing on peg
static PT_THREAD (protothread_anim2(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static uint32_t begin_time ;
    static int spare_time ;
    for (int u = 0; u < active_balls; u++) {
      spawnBall2(u) ;
    }

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;

      // update ball's position and velocity
      // need to do for every ball, then for every peg... nested for loops???
      for (int ball = 0; ball < MAX_BALLS; ball++) {
        // erase ball at old position
        maskBall(fix2int15(balls2[ball].x), fix2int15(balls2[ball].y)) ;
        if (ball < active_balls) {
          // Apply gravity to the ball
          balls2[ball].vy = balls2[ball].vy + float2fix15(GRAVITY_FRACTION) ;
          balls2[ball].x += balls2[ball].vx ;
          balls2[ball].y += balls2[ball].vy ;
          
          check_collisions_opt2(ball);
          
          // Update the walls and edges, and handle wall collisions
          wallsAndEdges2(ball) ;

          // draw the ball at its new position
          drawBall(fix2int15(balls2[ball].x), fix2int15(balls2[ball].y)) ;
        }
      }

      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

      // // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // animation thread 2

// thread to draw ball histogram at bottom of screen
static PT_THREAD (protothread_histo(struct pt *pt)) 
{
  // Mark beginning of thread
    PT_BEGIN(pt);
    
    // Variables for maintaining frame rate
    static uint32_t begin_time ;
    static int spare_time ;

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;

      // normalize the histogram (keep track of the bin with the max height)
      int bin_max_h = -1 ;
      for (int i = 0; i < NUM_BINS; i++) {
        if (bins[i] > bin_max_h) {
          bin_max_h = bins[i] ;
        }
      }

      // draw the bars
      int bar_width = 38; // based on center to center distance between pegs
      int height_difference ; // remainder of balls not part of normalized

      for (int i = 0; i < NUM_BINS; i++) {
        if (bin_max_h > 0) {
          new_heights[i] = (bins[i] * 60) / bin_max_h ;
        }
        else {
          new_heights[i] = bins[i] ; 
        }
        int x_coord = (35 + (bar_width * i)) ;

        drawRect(x_coord, 470 - old_heights[i], bar_width, old_heights[i], BLACK) ;
        drawRect(x_coord, 470 - new_heights[i], bar_width, new_heights[i], RED) ;

        old_heights[i] = new_heights[i] ; // update old value regardless
      }
      
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
    }
    PT_END(pt);
} // thread to animate histogram

static PT_THREAD(protothread_pot(struct pt *pt)) 
{
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;

  while (1) {
    // Measure time at start of thread
    begin_time = time_us_32() ;

    // average the ADC reads to make sure that the noise is averaged out
    uint32_t adc_sum = 0 ;
    for (int i = 0; i < 16; i++) {
      adc_sum += adc_read() ;
    }
    uint32_t adc_filtered = adc_sum >> 4 ; // divide by 16 by shifting 4 bits

    // now do the potentiometer function based on what the other thread said
    switch (pot_funct) {
      fix15 imm_prod;
      case INIT : // idk if we need this, can change later
      // does nothing 
      break ;
      case ADJUST_BALLS :
        imm_prod = multfix15(int2fix15(adc_filtered), (MAX_BALLS)); // Don't need to convert MAX_BALLS
        active_balls = fix2int15(divfix(imm_prod, 4096));
      break ;
      case ADJUST_BOUNCE :
        imm_prod = multfix15(int2fix15(adc_filtered), float2fix15(MAX_BOUNCE));
        bounciness = fix2float15(divfix(imm_prod, int2fix15(4096))); //4096 is the scaling factor for adc
      break ;
    }

    if ((prev_active_balls >= active_balls + 1) || (prev_active_balls <= active_balls - 1) || (prev_bounciness >= bounciness + 0.1) || (prev_bounciness <= bounciness - 0.1)) {
      // reset the histogram and number of fallen balls
      for (int i = 0; i < 15; i++) { 
        bins[i] = 0 ;
        old_heights[i] = 0 ;
        new_heights[i] = 0 ;
      }
      fallen_balls = 0 ;
      //erase old histo
      fillRect(10, 410, 620, 60, BLACK) ;

      // Update previous values to reflect the new state
      prev_active_balls = active_balls;
      prev_bounciness = bounciness;
    }

    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time) ;
  }
  PT_END(pt) ;
} // thread for the potentiometer

// Input button debouncing for state management
static PT_THREAD(protothread_debouncing(struct pt *pt)) 
{
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int spare_time ;
  static uint32_t begin_time ;
  
  while(1) {
    begin_time = time_us_32() ;

    int i = gpio_get(PIN_BUTTON) ; // value of press
    
    // implementing this debouncing algorithm with switch for clarity rather than if statements
    switch (STATE_0) {
      case NOT_PRESSED :
        if (i == 0) { // if the button is low (pressed)
          STATE_0 = MAYBE_PRESSED ;
          possible = i ; 
        }
        break ;
      case MAYBE_PRESSED :
        if (i == possible) {
          STATE_0 = PRESSED ;
          btn_pressed++ ; // send flag, potFSM thread will be activated by this
        }
        else {
          STATE_0 = NOT_PRESSED ;
        }
        break ;
      case PRESSED :
        if (i == 1) { // if the button is seen high again, maybe not pressed
          STATE_0 = MAYBE_NOT_PRESSED ;
          possible = i ;
        }
        break ;
      case MAYBE_NOT_PRESSED :
        if (i == possible) { //  possible is 1 right now, so if it is high send to not pressed
          STATE_0 = NOT_PRESSED ;
        }
        else {
          STATE_0 = PRESSED ;
        }
        break ;
    }
    
    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

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

  // initialize the VGA text
  setTextColor(RED) ;
  setTextSize(1) ;

  while(1) {
    // since this thread just cycles based on button presses, 
    // can just wait until the flag is incremented, 
    // and then restart STATE_1 in a loop without switch statements

    PT_WAIT_UNTIL(pt, btn_pressed > 0) ; // not using semaphores (maybe can?)
    btn_pressed--; // reset flag now that it has been used

    begin_time = time_us_32() ; // idk where to put this

    STATE_1 = (STATE_1 + 1) % 3 ; // states 0 through 2, will loop when state reaches 2

    switch (STATE_1) { // based on state display the currrent state and determine the function of the potentiometer
      case INIT :
        strcpy(state_buffer, "INIT") ;
        pot_funct = INIT ;
      break ;
      case ADJUST_BALLS :
        strcpy(state_buffer, "Adjusting balls") ;
        pot_funct = ADJUST_BALLS ;
      break ;
      case ADJUST_BOUNCE :
        strcpy(state_buffer, "Adjusting bounce") ;
        pot_funct = ADJUST_BOUNCE ;
      break ;
    }

    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time) ;

    // yield for necessary amount of time
    PT_YIELD_usec(spare_time) ;
  }

  PT_END(pt) ;
} // thread for potentiometer FSM


// ========================================
// === Core 1 entry
// ========================================
// put user input thread on core1
void core1_entry() 
{

  //pt_add_thread(protothread_timer) ;
  
  // need to keep both potentiometer threads and FSM threads on the same core
  pt_add_thread(protothread_pot) ; 
  pt_add_thread(protothread_potFSM) ;
  pt_add_thread(protothread_debouncing) ;
  pt_add_thread(protothread_anim2);

  // start scheduler on core1
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  set_sys_clock_khz(CLOCK_SPEED, true) ;
  // initialize stio
  stdio_init_all() ;

  // setup audio
  audio_init();

  // SETUP GPIO
  gpio_init(LED) ;
  gpio_set_dir(LED, GPIO_OUT) ;
  gpio_put(LED, 0) ;

  // Setup adc
  adc_init() ;
  adc_gpio_init(ADC_PIN) ;
  adc_select_input(ADC_CHAN) ;

  // setup button gpio
  gpio_init(PIN_BUTTON) ;
  gpio_set_dir(PIN_BUTTON, GPIO_IN); // set GPIO to input
  gpio_pull_up(PIN_BUTTON) ; // drive the pin normally high, if button pressed will be low

  // initialize VGA
  initVGA() ;

  // initialize state to init
  // initialize the VGA text
  setTextColor(RED) ;
  setTextSize(1) ;
  setCursor(30, 80);
  writeString("State: ") ;
  strcpy(state_buffer, "INIT") ;
  writeString(state_buffer) ;
  pot_funct = INIT;

  // Seed random number gen
  srand(time_us_32()) ;
  
  // initialize balls and pegs
  init_balls() ;
  init_balls2() ;

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(core1_entry);

  // add threads
  pt_add_thread(protothread_anim);
  pt_add_thread(protothread_histo);


  // start scheduler
  pt_schedule_start ;
}
