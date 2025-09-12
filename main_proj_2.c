#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


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
#define TIMEOUT = 10000000 // How long to wait until assuming trig was lost
#define ITP (uint32_t *)

#define JC_FORWARD_1 0b000000
#define JC_FORWARD_2 0b011011
#define JC_TURN1 0b101011
#define JC_TURN2 0b010100

#define JC_IDLE 0b111111

#define QUAD_ENC_1IN 45
#define QUAD_ENC_TURN_90 320

#define LDUTYADJUST 15
#define RDUTYADJUST 1
#define DUTYUPPERLIM 0xb
#define DUTYLOWERLIM 0x8

#define FIRST_SWITCH 0x0001
#define ITP (uint32_t *)
#define HEX_MAX 40
#define HEX_MIN -40
#define NEG_SIGN_ENABLE (1 << 7)

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
void timer_2us(unsigned t);
void set_trig_pin();
void clear_trig_pin();
_Bool read_echo_pin();
void show_sseg(uint8_t * sevenSegValue);
_Bool UpButton_pressed();
_Bool DownButton_pressed();
_Bool LeftButton_pressed();
_Bool RightButton_pressed();
uint32_t read_L1_quad_enc(_Bool reset);
uint32_t read_R1_quad_enc(_Bool reset);
uint32_t * convert_timer_to_hex_address (uint8_t timer_number);
void configure_timers();
void start_stopwatch(uint8_t timer_number);
uint32_t read_stopwatch(uint8_t timer_number);
void driveForward();
void increment_values(uint8_t* value1,uint8_t* value2);
void Idle();
_Bool driveDist(uint8_t inches);
_Bool turn90();
_Bool turn180();
void Update_SSEG_Vals();

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
    
int8_t ypos=0;
int8_t xpos=0;
uint8_t ypos_abs;
uint8_t xpos_abs;
uint8_t sevenSegValue[4] = {0};

int main (void)
{
    uint32_t left_duty_cycle = 0xc3;
    uint32_t right_duty_cycle = 0xdc;
    //set up PMOD DDRs
    JC_DDR &= 0;  
    JA |= 0x03; 


    init_program();

    show_sseg(&sevenSegValue[0]); 

    uint8_t pwmCnt = 0;
    uint8_t l_quad_counter;
    uint8_t r_quad_counter;    

    bool enter_coords=true;

    read_L1_quad_enc(true);
    read_R1_quad_enc(true);
     
            
    l_quad_counter=read_L1_quad_enc(false);
    r_quad_counter=read_R1_quad_enc(false);
        
            
    // Update display with new duty cycle values
    
    while (enter_coords)
    {
        Update_SSEG_Vals();
        if ((SWITCHES&FIRST_SWITCH))
        {
            enter_coords=false;
        }
    }
            
    if(ypos<0)
    {
        while(turn180())
        {   
            if (pwmCnt <= left_duty_cycle)
            {
                JC |= (1 << L_PWM_OFFSET);            
            }
            else
            {
                JC &= ~(1 << L_PWM_OFFSET);            
            }

            // Enter code to modify the PWM signal for the right motor
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
            if ((l_quad_counter<r_quad_counter)&&(left_duty_cycle<DUTYUPPERLIM))
            {
                left_duty_cycle+=LDUTYADJUST;   
            }
            if ((l_quad_counter<r_quad_counter))
            {
                right_duty_cycle-=RDUTYADJUST;
            }   
            if ((r_quad_counter<l_quad_counter)&&(left_duty_cycle>DUTYLOWERLIM))
            {
                left_duty_cycle--;
            }              
        }
        ypos_abs=abs(ypos);
    }
    else 
    {
        ypos_abs = ypos; 
    }
    read_L1_quad_enc(true);
    read_R1_quad_enc( true);
            
   
    while(driveDist(ypos_abs))
    {
        show_sseg( &sevenSegValue[0]);
        l_quad_counter=read_L1_quad_enc(false);
        l_quad_counter=read_R1_quad_enc(false);
        if (pwmCnt <= left_duty_cycle)
        {
            JC |= (1 << L_PWM_OFFSET);            
        }
        else
        {
            JC &= ~(1 << L_PWM_OFFSET);            
        }

        // Enter code to modify the PWM signal for the right motor
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
        if ((l_quad_counter<r_quad_counter)&&(left_duty_cycle<DUTYUPPERLIM))
        {
            left_duty_cycle+=LDUTYADJUST;       
        }
        if ((l_quad_counter<r_quad_counter))
        {
            right_duty_cycle-=RDUTYADJUST;
        }   
        if ((r_quad_counter<l_quad_counter)&&(left_duty_cycle>DUTYLOWERLIM))
        {
            left_duty_cycle--;
        }

    }

    read_L1_quad_enc(true);
    read_R1_quad_enc( true);

    while(turn90())
    {           
        l_quad_counter=read_L1_quad_enc(false);
        r_quad_counter=read_R1_quad_enc(false);
        if (pwmCnt <= left_duty_cycle)
        {
            JC |= (1 << L_PWM_OFFSET);            
        }
        else
        {
            JC &= ~(1 << L_PWM_OFFSET);            
        }
        // Enter code to modify the PWM signal for the right motor
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
        if ((l_quad_counter<r_quad_counter)&&(left_duty_cycle<DUTYUPPERLIM))
        {
            left_duty_cycle+=LDUTYADJUST;
        }
        if ((l_quad_counter<r_quad_counter))
        {
            right_duty_cycle-=RDUTYADJUST;
        }   
        if ((r_quad_counter<l_quad_counter)&&(left_duty_cycle>DUTYLOWERLIM))
        {
            left_duty_cycle--;
        }
    }

    read_L1_quad_enc(true);
    read_R1_quad_enc( true);
    if(xpos>0)
    {
        xpos_abs = xpos; 
        while(turn180())
        {
            show_sseg( &sevenSegValue[0]);
            if (pwmCnt <= left_duty_cycle)
            {
                JC |= (1 << L_PWM_OFFSET);            
            }
            else
            {
                JC &= ~(1 << L_PWM_OFFSET);            
            }

            // Enter code to modify the PWM signal for the right motor
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
            if ((l_quad_counter<r_quad_counter)&&(left_duty_cycle<DUTYUPPERLIM))
            {
                left_duty_cycle+=LDUTYADJUST;
            }
            if ((l_quad_counter<r_quad_counter))
            {
                right_duty_cycle-=RDUTYADJUST;
            }   
            if ((r_quad_counter<l_quad_counter)&&(left_duty_cycle>DUTYLOWERLIM))
            {
                left_duty_cycle--;
            }
        }
            
    }
    else
    {
        xpos_abs = abs(xpos); 
    }
    read_L1_quad_enc(true);
    read_R1_quad_enc( true);
    while(driveDist(xpos_abs))
    {
        show_sseg( &sevenSegValue[0]);
        l_quad_counter=read_L1_quad_enc(false);
        r_quad_counter=read_R1_quad_enc(false);
        if (pwmCnt <= left_duty_cycle)
        {
            JC |= (1 << L_PWM_OFFSET);            
        }
        else
        {
            JC &= ~(1 << L_PWM_OFFSET);            
        }

        // Enter code to modify the PWM signal for the right motor
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
        if ((l_quad_counter<r_quad_counter)&&(left_duty_cycle<DUTYUPPERLIM))
        {
            left_duty_cycle+=LDUTYADJUST;
               
        }
        if ((l_quad_counter<r_quad_counter))
        {
            right_duty_cycle-=RDUTYADJUST;
        }   
        if ((r_quad_counter<l_quad_counter)&&(left_duty_cycle>DUTYLOWERLIM))
        {
            left_duty_cycle--;
        }
    }

    Idle();   

    
    return 0; 
}

void Update_SSEG_Vals()
{
    _Bool btnU = UpButton_pressed(); 
    _Bool btnD = DownButton_pressed(); 
    _Bool btnL = LeftButton_pressed(); 
    _Bool btnR = RightButton_pressed();  


    int8_t ypos1, ypos10, xpos1, xpos10; 
   

    if (btnU && (ypos + INCREMENT <= HEX_MAX))
    {
        ypos = ypos + INCREMENT; 
    }
    if (btnD && (ypos - INCREMENT >= HEX_MIN))
    {
        ypos = ypos - INCREMENT; 
    }
    if (btnL && (xpos + INCREMENT <= HEX_MAX))
    {
        xpos = xpos + INCREMENT; 
    }
    if (btnR && (xpos - INCREMENT >= HEX_MIN))
    {
        xpos = xpos - INCREMENT; 
    }

   
    ypos1 = abs(ypos) % 10; 
    ypos10 = abs(ypos) / 10;
    xpos1 = abs(xpos) % 10; 
    xpos10 = abs(xpos) / 10;     
    sevenSegValue[0] = sevenSegLUT[ypos1]; 
    sevenSegValue[1] =  sevenSegLUT[ypos10]; 
    sevenSegValue[2] =  sevenSegLUT[xpos1]; 
    sevenSegValue[3] =  sevenSegLUT[xpos10]; 

    if(ypos < 0)
    {
        sevenSegValue[0] &= ~NEG_SIGN_ENABLE;
        sevenSegValue[1] &= ~NEG_SIGN_ENABLE; 
    }
    if(xpos < 0)
    {
        sevenSegValue[2] &= ~NEG_SIGN_ENABLE;
        sevenSegValue[3] &= ~NEG_SIGN_ENABLE; 
    } 
   

    show_sseg(&sevenSegValue[0]); 
}
 


void increment_values(uint8_t* value1,uint8_t* value2)
{
    if (UpButton_pressed() && (*value1 + INCREMENT <= 0x28))
        {
            *value1 = *value1 + INCREMENT;
        }
        if (DownButton_pressed() && (*value1 + INCREMENT <= 0x28))
        {
            *value1 = *value1 - INCREMENT;
        }
        if (LeftButton_pressed() && (*value2 + INCREMENT <= 0x28))
        {
            *value2 = *value2 + INCREMENT;
        }
        if (RightButton_pressed() && (*value2 + INCREMENT <= 0x28))
        {
            *value2 = *value2 - INCREMENT;
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
            read_L1_quad_enc(true);
            read_R1_quad_enc(true);
            Idle();
            return false;
        }        

}

_Bool turn90()
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
_Bool turn180()
{
    if((read_L1_quad_enc(false)<(2*QUAD_ENC_TURN_90))&&(read_R1_quad_enc(false)<(2*QUAD_ENC_TURN_90)))
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

// Function Implementation - Initialization
void init_program()
{
    configure_timers();
}

// Function Implementation - Ultrasonic Sensors
void set_trig_pin()
{
    JB |=1;
}

void clear_trig_pin()
{
    JB &=0;
}

_Bool read_echo_pin(){
    return JB & (1<<1);
}

// Function implementation - Hardware Timers
uint32_t * convert_timer_to_hex_address (uint8_t timer_number) {
    if (timer_number > 7)
        return ITP (TIMERS[0]);
    else
        return ITP (TIMERS[timer_number]);
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

_Bool delay_half_sec(){
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
        for( cntr1=0; cntr1 < 8; cntr1++);
}

// Function implementation - Button debounces
_Bool UpButton_pressed()
{
    _Bool last_btnU = 0;
    static _Bool btnU = 0;
    last_btnU = btnU;
    btnU = BUTTONS & (1 << BTNU_OFFSET);
    return (btnU & !last_btnU);
}

_Bool DownButton_pressed()
{
    _Bool last_btnD = 0;
    static _Bool btnD = 0;
    last_btnD = btnD;
    btnD = BUTTONS & (1 << BTND_OFFSET);
    return (btnD & !last_btnD);
}

_Bool LeftButton_pressed()
{
    _Bool last_btnL = 0;
    static _Bool btnL = 0;
    last_btnL = btnL;
    btnL = BUTTONS & (1 << BTNL_OFFSET);
    return (btnL & !last_btnL);
}

_Bool RightButton_pressed()
{
    _Bool last_btnR = 0;
    static _Bool btnR = 0;
    last_btnR = btnR;
    btnR = BUTTONS & (1 << BTNR_OFFSET);
    return (btnR & !last_btnR);
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
