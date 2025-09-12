#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <xil_types.h>



/*
Volatile – Variable descriptor marking this variable as immutable or unable to be reassigned in code.
unsigned volatile * - A pointer to an immutable unsigned integer. 

( * (unsigned volatile *) 0x40010000) – A pointer to an unsigned volatile pointer that is referencing the memory address 0x40010000. 
*/

#define BUTTONS     (* (unsigned volatile *) 0x40000000) // 4  pin In       btnU, btnL, btnR, btnD
#define JA          (* (unsigned volatile *) 0x40001000) // 8  pin In/Out   JA[7:0]
#define JA_DDR      (* (unsigned volatile *) 0x40001004) // 8  pin DDR,     1 means input, 0 means output
#define JB          (* (unsigned volatile *) 0x40002000) // 8  pin In/Out   JB[7:0]
#define JB_DDR      (* (unsigned volatile *) 0x40002004) // 8  pin DDR      1 means input, 0 means output
#define JC          (* (unsigned volatile *) 0x40003000) // 8  pin In/Out   JC[7:0]
#define JC_DDR      (* (unsigned volatile *) 0x40003004) // 8  pin DDR,     1 means input, 0 means output
#define JXADC       (* (unsigned volatile *) 0x40004000) // 8  pin In/Out   JXADC[7:0]
#define JXADC_DDR   (* (unsigned volatile *) 0x40004004) // 8  pin DDR      1 means input, 0 means output
#define LEDS        (* (unsigned volatile *) 0x40005000) // 16 pin Out      led[15:0]
#define ANODES      (* (unsigned volatile *) 0x40006000) // 4  pin Out      an[3:0]
#define SEVEN_SEG   (* (unsigned volatile *) 0x40006008) // 8  pin Out      dp, seg[6:0]
#define SWITCHES    (* (unsigned volatile *) 0x40007000) // 16 pin In       sw[15:0]
#define UART        (* (unsigned volatile *) 0x40008000) // Out of Scope
#define TIMER_0     (* (unsigned volatile *) 0x40009000)
#define TIMER_01    (* (unsigned volatile *) 0x40009004)
#define TIMER_02    (* (unsigned volatile *) 0x40009008)
#define TIMER_1     (* (unsigned volatile *) 0x4000A000)
#define TIMER_2     (* (unsigned volatile *) 0x4000B000)
#define TIMER_3     (* (unsigned volatile *) 0x4000C000)

// Memory Access Offsets - Buttons
#define BTND_OFFSET 0 // BTN[0]
#define BTNR_OFFSET 1 // BTN[1]
#define BTNL_OFFSET 2 // BTN[2]
#define BTNU_OFFSET 3 // BTN[3]

// Memory Access Offsets - Motors
#define L_PWM_OFFSET  0  // JC[0]
#define LEFT1_OFFSET  1  // JC[1]
#define LEFT2_OFFSET  2  // JC[2]
#define R_PWM_OFFSET  3  // JC[3]
#define RIGHT2_OFFSET 4  // JC[4]
#define RIGHT1_OFFSET 5  // JC[5]

// Memory Access Offsets - Quad Encs
#define L1_QUAD_ENC_OFFSET 0 // JA[0]
#define R1_QUAD_ENC_OFFSET 1 // JA[1]

// Constants
#define PWM_TOP 255
#define INCREMENT 1
#define TIMEOUT 10000000 // How long to wait until assuming trig was lost
#define ITP (uint32_t *)

#define JC_FORWARD_1 0b000000
#define JC_FORWARD_2 0b011011
#define JC_TURN1 0b101011
#define JC_TURN2 0b010100
#define CLEAR_LEFT 0b110111
#define CLEAR_FRONT 0b111011
#define SET_LEFT 0b001000
#define SET_FRONT 0b000100

#define SET_JB_DDR 0b000011
#define JC_IDLE 0b111111
#define DETECT_DIST 4
#define ROBOT_DIST 3
#define TURN_SHARPNESS_ADJUST 2

#define QUAD_ENC_1IN 45
#define QUAD_ENC_TURN_90 290

#define LDUTYADJUST 15
#define RDUTYADJUST 1
#define DUTYUPPERLIM 0xb
#define DUTYLOWERLIM 0x8

#define FIRST_SWITCH 0x0001
#define ITP (uint32_t *)
#define HEX_MAX 40
#define HEX_MIN -40
#define NEG_SIGN_ENABLE (1<<7) 
#define READ_JB1 0b000001
#define READ_JB2 0b000010




const volatile uint32_t TCSR_OFFEST = 0;
const volatile uint32_t TLR_OFFEST = 1;
const volatile uint32_t TCR_OFFSET = 2;


// Addresses for all the timers, ITP casts the int to uint32_t ptr
const uint32_t * TIMERS[] = { ITP 0x40009000, ITP 0x40009100, ITP 0x4000A000, ITP 0x4000A100,
                              ITP 0x4000B000, ITP 0x4000B100, ITP 0x4000C000, ITP 0x4000C100};

// Function declarations - implemented below
void init_program(); // One Time Initializations
_Bool delay_1s();
_Bool delay_half_sec();
_Bool stopwatch_1s(); 
void restart_timer0(); 
void set_trig_pin_front();
void set_trig_pin_left();
void clear_trig_pin_front();
void clear_trig_pin_left(); 
void show_sseg(uint8_t * sevenSegValue);
uint32_t read_L1_quad_enc(_Bool reset);
uint32_t read_R1_quad_enc(_Bool reset);
uint32_t * convert_timer_to_hex_address (uint8_t timer_number);
void configure_timers();
void start_stopwatch(uint8_t timer_number);
uint32_t read_stopwatch(uint8_t timer_number);
void driveForward();
void Idle();
_Bool driveDist(uint8_t inches);
_Bool turn_left();
_Bool turn_right();
void slight_left();
uint32_t Read_Front_Sensor(); 
uint32_t Read_Left_Sensor(); 
uint32_t get_timer0_value_us(); 
_Bool read_echo_pin_front(); 
_Bool read_echo_pin_left(); 
void timer_2us(unsigned t);

uint8_t sevenSegLUT[10] = {
            0xc0, //0
            0xf9, //1
            0xa4, //2
            0xb0, //3
            0x99, //4
            0x92, //5
            0x82, //6
            0xf8, //7
            0x80, //8
            0x90, //9
            }; 
    
uint8_t sevenSegLetterLUT[4] = {
        0xA1, //d
        0xc0, //0
        0xAB, //n
        0x86, //E
}; 

uint8_t sevenSegValue[1] = {0}; 
uint8_t sevenSegValueLet[4] = {0}; 


int main (void)
{
    init_program();
    uint32_t distance_forward, distance_left; 
    uint32_t l_quad,old_l_quad;
    uint32_t left_duty_cycle = 0xe0;
    uint32_t right_duty_cycle = 0xff;
    uint32_t old_dist_forward, old_dist_left;
    uint8_t obstacle = 0; 
    uint8_t counter = 0; 
    _Bool navigating = TRUE;
    bool enter_coords=true;
    uint8_t pwmCnt = 0;

    //set up PMOD DDRs
    JC_DDR &= 0;  
    JA |= 0x03; 
    JB_DDR = SET_JB_DDR; 
    LEDS = 0x0000; 
    ANODES = 0b1110; 
    SEVEN_SEG = sevenSegLUT[obstacle]; 

    //Timer Values 
    TIMER_0 = 0b010010010001;
    TIMER_02 = 0x00000000; 
    restart_timer0(); 

    //blocking while loop so robot only moves after switch is flipped
    while(enter_coords)
    {       
       if((SWITCHES&FIRST_SWITCH))
       {
           enter_coords = false;            
       }
    }

    //main driving while loop, uses bool so it can be exited when course is completed
    while (navigating)
    { 
        //read sensors and Sensors and store in variables 
        distance_forward = Read_Front_Sensor(); 
        distance_left = Read_Left_Sensor();

        //shows what sensors are triggered with LEDS
        if(distance_forward <= DETECT_DIST)
        {
            LEDS |= 0xFF00; 

            
            
        }
        else
        {
            LEDS &= 0x00FF; 
        }

        if(distance_left <= DETECT_DIST)
        {
            LEDS |= 0x00FF; 

        }
        else
        {
            LEDS &= 0xFF00; 
        }

        //Robots action when something is detected in front of it
        if ( distance_forward < DETECT_DIST)
        {

            //set duty cycles
            left_duty_cycle = 0xe0;
            right_duty_cycle = 0xff; 

            
            SEVEN_SEG = sevenSegLUT[++obstacle]; 

            //code to make the robot turn right
            while(turn_right())
            {
                if (pwmCnt <= left_duty_cycle)
                {
                    JC |= (1 << L_PWM_OFFSET);            
                }
                else
                {
                    JC &= ~(1 << L_PWM_OFFSET);            
                }
                if (pwmCnt <= right_duty_cycle)
                {
                    JC |= (1 << R_PWM_OFFSET);            
                }
                else
                {
                    JC &= ~(1 << R_PWM_OFFSET);            
                }
                if (pwmCnt++ == PWM_TOP)
                {
                    pwmCnt = 0;
                }
            }

            
            //exits while loop at the end of the course by setting bool to false
            if (distance_forward<DETECT_DIST && old_dist_forward<DETECT_DIST)
            {
                navigating=FALSE; 
            }
            //reset quad encoders
            read_L1_quad_enc(true); 
            read_R1_quad_enc( true);
            //increment object counter

        }

        //robot is just driving forward
        else if(distance_left <= DETECT_DIST)
        {
            //ensures robot stays near wall by manipulating duty cycles
            if (distance_left<ROBOT_DIST)
            {
                left_duty_cycle=0x00;
            }
            else if(distance_left>ROBOT_DIST)
            {
                left_duty_cycle=0xff;
            }
            else
            {
                right_duty_cycle=0xff;
                left_duty_cycle = 0xe0;

            }
            //have robot drive 1 unit of distance (unit is determined by a macro in funciton)
            while(driveDist(1))
            {
                l_quad=read_L1_quad_enc(false);

                //if robot gets stuck due to too low duty cycles, gives it more so it can move
                if (old_l_quad==l_quad)
                {
                    left_duty_cycle=0xa0;
                }

                if (pwmCnt <= left_duty_cycle)
                {
                    JC |= (1 << L_PWM_OFFSET);            
                }
                else
                {
                    JC &= ~(1 << L_PWM_OFFSET);            
                }
                if (pwmCnt <= right_duty_cycle)
                {
                    JC |= (1 << R_PWM_OFFSET);            
                }
                else
                {
                    JC &= ~(1 << R_PWM_OFFSET);            
                }
                if (pwmCnt++ == PWM_TOP)
                {
                    pwmCnt = 0;
                }
                old_l_quad=l_quad;
            }
            //reset quad encoders
            read_L1_quad_enc(true); 
            read_R1_quad_enc( true); 
        }

        //code to execute left turn if wall is lost or if an object has ended    
        else if (distance_left > DETECT_DIST && distance_forward > DETECT_DIST)
        {
            //set duty cycles
            left_duty_cycle = 0xe0;
            right_duty_cycle = 0xff;
            
            //if statment to control sharpness of left turn, and prevent crashing into wall
            if (counter < TURN_SHARPNESS_ADJUST && old_dist_left>2)
            {
                slight_left();
                ++counter; 
            }
            else
            {
                counter = 0; 
                while(driveDist(1))
                {
                    if (pwmCnt <= left_duty_cycle)
                    {
                        JC |= (1 << L_PWM_OFFSET);            
                    }
                    else
                    {
                        JC &= ~(1 << L_PWM_OFFSET);            
                    }
                    if (pwmCnt <= right_duty_cycle)
                    {
                        JC |= (1 << R_PWM_OFFSET);            
                    }
                    else
                    {
                        JC &= ~(1 << R_PWM_OFFSET);            
                    }
                    if (pwmCnt++ == PWM_TOP)
                    {
                        pwmCnt = 0;
                    }
                } 
            }
            if (pwmCnt <= left_duty_cycle)
            {
                JC |= (1 << L_PWM_OFFSET);            
            }
            else
            {
                JC &= ~(1 << L_PWM_OFFSET);            
            }

            if (pwmCnt <= right_duty_cycle)
            {
                JC |= (1 << R_PWM_OFFSET);            
            }
            else
            {
                JC &= ~(1 << R_PWM_OFFSET);            
            }
            if (pwmCnt++ == PWM_TOP)
            {
                pwmCnt = 0;
            }
            
            //reset quad encoders
            read_L1_quad_enc(true); 
            read_R1_quad_enc( true);
        }
        
        //store current sensor data as old sensor data before next cycle
        old_dist_forward=distance_forward;
        old_dist_left=distance_left;
    }
    
    //while loop for "victory dance"
    while(1)
        {
            //set duty cycles
            left_duty_cycle = 0xe0;
            right_duty_cycle = 0xff;

            //display a value on the SSG to show we detected the end of the course, and our robot isn't just bugging out
            sevenSegValueLet[0] = sevenSegLetterLUT[3]; 
            sevenSegValueLet[1] = sevenSegLetterLUT[2]; 
            sevenSegValueLet[2] = sevenSegLetterLUT[1]; 
            sevenSegValueLet[3] = sevenSegLetterLUT[0]; 
            show_sseg(sevenSegValueLet); 
            
            //robot twirls like a princess
            slight_left();

            if (pwmCnt <= left_duty_cycle)
            {
                JC |= (1 << L_PWM_OFFSET);            
            }
            else
            {
                JC &= ~(1 << L_PWM_OFFSET);            
            }
            if (pwmCnt <= right_duty_cycle)
            {
                JC |= (1 << R_PWM_OFFSET);            
            }
            else
            {
                JC &= ~(1 << R_PWM_OFFSET);            
            }
            //Will reset if too high
            if (pwmCnt++ == PWM_TOP)
            {
                pwmCnt = 0;
            } 
        } 

    return 0; 
}


//Function definitions
uint32_t Read_Front_Sensor()
{
    uint32_t time = 0, 
    count = 0, 
    distance = 0;  

    enum phase_list {send_trig, wait_for_echo, 
    count_echo_duration, echo_falling_edge, cooldown}; 
    enum phase_list current_state = send_trig; 
    enum phase_list next_state = current_state; 

    while(1)
    {
        switch(current_state)
        {
        case send_trig: 
            set_trig_pin_front(); 
            timer_2us(5); 
            clear_trig_pin_front(); 
            count = 0; 
            next_state = wait_for_echo; 
            break; 
        case wait_for_echo: 
            if(read_echo_pin_front())
            {
                time = 0; 
                next_state = count_echo_duration; 
            }
            else if(!(read_echo_pin_front()))
            {
                count += 1; 
                if(count == TIMEOUT)
                {
                    next_state = cooldown; 
                }
            }
            break; 
        case count_echo_duration:
            restart_timer0(); 
            while (read_echo_pin_front()); 
            next_state = echo_falling_edge; 
            break; 
        case echo_falling_edge: 
            time = get_timer0_value_us();  
            distance = (time / 148); 
            if (distance != 0) 
            {
             
                return distance;             
            }
            next_state = cooldown; 
            break; 
        case cooldown:
            if (delay_half_sec())
            {
                next_state = send_trig; 
            }
            break; 
        default: 
            next_state = send_trig; 
        }
    current_state = next_state; 
    }
    
}

uint32_t Read_Left_Sensor()
{
    uint32_t time = 0, 
    count = 0, 
    distance = 0; 

    enum phase_list {send_trig, wait_for_echo, 
    count_echo_duration, echo_falling_edge, cooldown}; 
    enum phase_list current_state = send_trig; 
    enum phase_list next_state = current_state; 
  
    while(1)
    {
        switch(current_state)
        {
        case send_trig: 
            set_trig_pin_left(); 
            timer_2us(5);         
            clear_trig_pin_left(); 
            count = 0; 
            next_state = wait_for_echo; 
            break; 
        case wait_for_echo: 
            if(read_echo_pin_left())
            {
                time = 0; 
                next_state = count_echo_duration; 
            }
            else if(!(read_echo_pin_left()))
            {
                count += 1; 
                if(count == TIMEOUT)
                {
                    next_state = cooldown; 
                }
            }
            break; 
        case count_echo_duration:
            restart_timer0(); 
            while (read_echo_pin_left()); 
            next_state = echo_falling_edge; 
            break; 
        case echo_falling_edge: 
            time = get_timer0_value_us();  
            distance = (time / 148); 
            if (distance != 0) 
            {
                return distance;             
            }
            next_state = cooldown; 
            break; 
        case cooldown:
            if (delay_half_sec())
            {
                next_state = send_trig; 
            }
            break; 
        default: 
            next_state = send_trig; 
        }
        current_state = next_state; 
    }

}


void driveForward()
{
    JC &=JC_FORWARD_1;
    JC |=JC_FORWARD_2;
}


void Idle()
{
    read_L1_quad_enc(true);
    read_R1_quad_enc(true);
    JC |=JC_IDLE;
}

_Bool driveDist(uint8_t inches)
{
    if((read_L1_quad_enc(false)<(QUAD_ENC_1IN*inches)&&read_R1_quad_enc(false)<(QUAD_ENC_1IN*inches)))
        {
            driveForward();
            return true;
        }
    else
        {
            return false;
        }        

}

_Bool turn_left()
{
    if((read_L1_quad_enc(false)<QUAD_ENC_TURN_90)&&(read_R1_quad_enc(false)<QUAD_ENC_TURN_90))
        {
            JC &= ~((1<<LEFT2_OFFSET)|(1<<RIGHT2_OFFSET));
            JC |= ((1<<LEFT1_OFFSET)|(1<<RIGHT1_OFFSET));
            return true;
        }
    else
        {

            Idle();
            return false;
        }        
}

void slight_left()
{
    JC &= ~((1<<LEFT2_OFFSET)|(1<<RIGHT2_OFFSET));
    JC |= ((1<<LEFT1_OFFSET)|(1<<RIGHT1_OFFSET));
}

_Bool turn_right()
{
    if((read_L1_quad_enc(false)<(QUAD_ENC_TURN_90))&&(read_R1_quad_enc(false)<(QUAD_ENC_TURN_90)))
        {
            JC |= ((1<<LEFT2_OFFSET)|(1<<RIGHT2_OFFSET)); //0b010001;
            JC &= ~((1<<LEFT1_OFFSET)|(1<<RIGHT1_OFFSET));//0b110101; 
            return true;
        }
    else
        {

            Idle();
            return false;
        }        
}

// Function Implementation - Initialization
void init_program()
{
    configure_timers();
}

// Function Implementation - Ultrasonic Sensors
void set_trig_pin_front()
{
    JB |= SET_FRONT;
}

void set_trig_pin_left()
{
    JB |= SET_LEFT; 
}

void clear_trig_pin_front()
{
    JB &=CLEAR_FRONT;
}

void clear_trig_pin_left()
{
    JB &=CLEAR_LEFT; 
}

_Bool read_echo_pin_front()
{
    return JB & READ_JB1; 
}

_Bool read_echo_pin_left()
{
    return JB & READ_JB2; 
}

// Function implementation - Hardware Timers
uint32_t * convert_timer_to_hex_address (uint8_t timer_number) {
    if (timer_number > 7)
        return ITP (TIMERS[0]);
    else
        return ITP (TIMERS[timer_number]);
}

void restart_timer0()
{
    TIMER_0 &= ~(1<<7);
    TIMER_0 |= 1<<5;
    TIMER_0 &= ~(1<<5);
    TIMER_0 |= (1<<7);
}

uint32_t get_timer0_value_us()
{
    uint32_t count = TIMER_02;
    TIMER_0 |= 1<<8;
    uint32_t time_us = count/100;
    return time_us;
}

void configure_timers (){
    for (int i =  0; i < 8; i++) {
        uint32_t * timer_base_address = convert_timer_to_hex_address(i);
        uint32_t * tcr = timer_base_address + TCR_OFFSET;
        uint32_t * tcsr = timer_base_address + TCSR_OFFEST;
        *(tcr) = 0x00000000;
        *(tcsr) = 0b010010010001;
    }
}

void start_stopwatch(uint8_t timer_number){
    if (timer_number > 7){
        return;
}
    uint32_t * timer_base_address = convert_timer_to_hex_address(timer_number);
    volatile uint32_t * tcsr = timer_base_address + TCSR_OFFEST;

    *tcsr &= ~(1<<7);
    *tcsr |= 1<<5;
    *tcsr &= ~(1<<5);
    *tcsr |= 1<<7;
}

uint32_t read_stopwatch(uint8_t timer_number){
    if (timer_number > 7){
        return 0;
}
    uint32_t * timer_base_address = convert_timer_to_hex_address(timer_number);
    volatile uint32_t * tcr = timer_base_address + TCR_OFFSET;
    return (*tcr) / 100;
}

_Bool stopwatch_1s()
{
    const uint32_t TOP = 2870000;
    static uint32_t count = 0;
    if (count++ == TOP)
    {
        count = 0;
        return true;
    }
    return false;
}

// Function Implementation - Software Delays
_Bool delay_1s(){
    const uint32_t TOP = 2870000;
    static uint32_t count = 0;
    if (count == TOP){
        count = 0;
        return true;
    }
    count++;
    return false;
}

_Bool delay_half_sec()
{
    const uint32_t TOP = 1350000;
    static uint32_t count = 0;
    if (count == TOP){
        count = 0;
        return true;
    }
    count++;
    return false;
}

void timer_2us(unsigned t)
{
    volatile unsigned cntr1;
    while(t--)
        for(cntr1=0; cntr1 < 8; cntr1++);
}

uint32_t read_L1_quad_enc(_Bool reset){
    static uint32_t cnt = 0;
    static _Bool quad_enc_last_state = 0;

    if (reset){cnt = 0;}

    if((quad_enc_last_state == 0) && (JA & (1<<L1_QUAD_ENC_OFFSET))){
        cnt++;
    }
    quad_enc_last_state = JA & (1<<L1_QUAD_ENC_OFFSET);
    return cnt;
}


uint32_t read_R1_quad_enc(_Bool reset){
    static uint32_t cnt = 0;
    static _Bool quad_enc_last_state = 0;

    if (reset){cnt = 0;}
    if((quad_enc_last_state == 0) && (JA & (1<<R1_QUAD_ENC_OFFSET))){
        cnt++;
    }
    quad_enc_last_state = JA & (1<<R1_QUAD_ENC_OFFSET);
    return cnt;
}

// Function implementation - SSeg
void show_sseg(uint8_t * sevenSegValue){
    static uint8_t anodeCnt = 0;
    if(read_stopwatch(1)>1000){
        anodeCnt++;
        ANODES = ~(1 << (anodeCnt % 4));
        SEVEN_SEG = sevenSegValue[anodeCnt % 4];
        start_stopwatch(1);
    }
}
