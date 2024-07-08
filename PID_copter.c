/**
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 * - GPIO 15---> button(pulled up)
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

//****************************************
// PWM duty cycle
volatile int control=3125 ;
volatile int old_control=3125 ;
volatile int input=3125;
int k=0;
fix15 accel_angle=0;
fix15 gyro_angle_delta=0;
fix15 complementary_angle=0;
int desired_angle= 50;
fix15 filtered_az=0;
fix15 filtered_ay=0;
float kp_const= 45;
float kd_const=  40000;
float ki_const= 0.1;
int button_inp;
float V_control=0;
float error_angle=0;
float last_error_angle=0;
float prop_cont=0;
int start_mov=0;
int move_begin_time=0;
int move_90=0;
int move_120=0;
int move_60=0;
float diff_cont=0;
float int_cont=0;
int fle=0;

float pi= 3.141592;
//*******************************************
// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0
uint slice_num ;

// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));
control= control+((input - control)>>6);

if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
    }

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
     button_inp =gpio_get(22);
     if(start_mov==1)
    {      if(button_inp==0)
               {desired_angle=0;
                            }
    else{
        if(fle==0){
        move_begin_time=time_us_32();
        fle=1;}

        if((time_us_32()-move_begin_time)>0 && (time_us_32()-move_begin_time)<5000000)
        {
        desired_angle=90;}
        
          if((time_us_32()-move_begin_time)>5000000 && (time_us_32()-move_begin_time)<10000000)
        {
        desired_angle=120;}

        if((time_us_32()-move_begin_time)>10000000 && (time_us_32()-move_begin_time)<15000000)
        {
        desired_angle=60;}

        if((time_us_32()-move_begin_time)>15000000)
        {
        desired_angle=90;
        start_mov=0;
        fle=0;}
        }
    
    }
    
    
    if(button_inp==0)
    {start_mov=1;}

    mpu6050_read_raw(acceleration, gyro);
filtered_az= filtered_az + ((acceleration[2] - filtered_az)>>6);
    filtered_ay= filtered_ay + ((acceleration[1] - filtered_ay)>>6);
    accel_angle= multfix15(float2fix15(atan2(filtered_az,-filtered_ay)), oneeightyoverpi);
    gyro_angle_delta=multfix15(-gyro[0],zeropt001);
    complementary_angle=multfix15(complementary_angle-gyro_angle_delta,zeropt999) + multfix15(accel_angle,zeropt001);
    //***************************pid control
last_error_angle=error_angle;
error_angle= desired_angle- fix2float15( complementary_angle);
prop_cont= kp_const*error_angle;
diff_cont= kd_const*(error_angle-last_error_angle);
int_cont = int_cont + ki_const*error_angle ;
if(int_cont<-4500)
  {int_cont= -4500;}
  if(int_cont>4500)
  {int_cont=4500;}



    V_control = prop_cont + int_cont + diff_cont ;//+ diff_cont + multfix15( ki_const,int_cont);
  
  if(V_control<0)
  {V_control= 0;}
  if(V_control>5000)
  {V_control=5000;}

 input =  V_control;
    //********************************




    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = 0. ;
    static float OldMax = 500. ;

    // Control rate of drawing
    static int throttle ;

 

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "+2500") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+5000") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "90") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+180") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;
 char info[100];
      sprintf(info, "dcy:%d| ang:%f| erorang:%f|kp:%f|kd:%f|ki:%f|but:%d", control, fix2float15(complementary_angle),error_angle,prop_cont,diff_cont,int_cont,button_inp);
      setCursor(0,0);
      setTextColor2(WHITE, BLACK);
      setTextSize(1);
      writeString(info);
            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            drawPixel(xcoord, 430 - (int)(NewRange*((float)(((control)/10)-OldMin)/OldRange)), WHITE) ;
            //drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[1])*120.0)-OldMin)/OldRange)), RED) ;
            //drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[2])*120.0)-OldMin)/OldRange)), GREEN) ;

            // Draw top plot
            //drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle)*2.77)-OldMin)/OldRange)), RED) ;
            //drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }

   
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static float float_in ;
    while(1) {
   

    sprintf(pt_serial_out_buffer, "1=chan drwrate,2=chng ducyc,3:des ang,4=kp,5=kd,6=ki ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &k) ;
        

      
   if(k==1)
        {
        sprintf(pt_serial_out_buffer, "input a command: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &classifier) ;

        // num_independents = test_in ;
        if (classifier=='t') {
            sprintf(pt_serial_out_buffer, "timestep: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            if (test_in > 0) {
                threshold = test_in ;
            }
        }}
        else if(k==2)
        {sprintf(pt_serial_out_buffer, "input a duty cycle (0-5000): ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in > 5000) continue ;
        else if (test_in < 0) continue ;
        else input = test_in ;}

        else if(k==3)
        {sprintf(pt_serial_out_buffer, "input desired angle(0-180): ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        if (test_in > 5000) continue ;
        else if (test_in < 0) continue ;
        else desired_angle =  test_in ;}


        else if(k==4)
        {sprintf(pt_serial_out_buffer, "input kp: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        float test_f=0;
        sscanf(pt_serial_in_buffer,"%f", &test_f) ;
        kp_const = test_f ;}

        else if(k==5)
        {sprintf(pt_serial_out_buffer, "input kd: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        float test_f=0;
        sscanf(pt_serial_in_buffer,"%f", &test_f) ;
        kd_const=test_f;}


        else if(k==6)
        {sprintf(pt_serial_out_buffer, "input ki: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        float test_f=0;
        sscanf(pt_serial_in_buffer,"%f", &test_f) ;
        ki_const=test_f;}

        else {sprintf(pt_serial_out_buffer, "invalid input: ");
        serial_write ;}
        



    }

    
    
    /*
static int state=0;
     static int cur_input=-1;

     if( button_inp != cur_input && state == 0 )
               {state=1;
                     cur_input=button_inp;}
     if ( button_inp == cur_input && state == 1 )
                {state=2;
                if(button_inp==0){
                start_mov=1;
                }
                
                }
     if ( button_inp != cur_input && state == 2 )
     {state=1;
     
     cur_input=button_inp;}  */ 

    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;

    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);


    gpio_init(22);
    gpio_set_dir(22,GPIO_IN);
    gpio_pull_up(22);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}