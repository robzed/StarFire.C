
/*******************************************************************************
 2 wheeled WF Mouse using 33FJ128MC804 processor with 2 x QEI
and Pololu Mini Motors
File Name: main.c  By Ken Hewitt 5/5/2019

Description: 
2.0.1 Rob's wall following code
2.0.2 My left turn code
2.0.3 code for int on change to select run speed
2.0.4 Various rob's changes
 * 
3 pulse encoder, 30:1 gearing, 32mm wheel, 90ppr x2 mode=180ppr = 0.5585mm = 0.56mm approx
16 bits gives max distance of 2293mm

This version is using only 4 sensors 2 x 30 and 2 x forward
 * 
 * Distance between wheel centers = 77mm approx.
 * Circle = Pi * D = 3.14159 * 77 = 241.9 mm (242mm)
 * Half turn = 121 mm
 * Quarter turn = 60.5 mm ... should be 433, actually 390.
*******************************************************************************/
//The Internal Fast RC (FRC) Oscillator provides a nominal 7.37 MHz clock 
// Default PLL = 6.25
// So FOSC = Approx. 46 MHz
// FOSC is divided by 2 to generate the device instruction clock (FCY) and the peripheral clock time base (FP). 
#pragma config FNOSC = FRCPLL // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config FWDTEN = OFF   // Watchdog Timer Enable (Watchdog timer
                              // enabled/disabled by user software)
#pragma config OSCIOFNC = ON  // disable OSC2 pin from oscillating

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <string.h>

#define DIGOLE_OLED 0
#define SERIAL_TERMINAL 1

#if DIGOLE_OLED

#define cursor_off() printf("CS0")
#define clear_screen() printf("CL")
#define DISPLAY_TEXT "TT"
#define TEXT_RETURN "TRT"

#elif SERIAL_TERMINAL

#define cursor_off() // do nothing   // "\033[0D"    // back 0 spaces, non-empty to avoid warnings
#define clear_screen()              //"\033[2J" // \033[;H"
#define DISPLAY_TEXT ""
#define TEXT_RETURN "\r\n"

#else
#error "Serial output format not defined!"
#endif

#define QUARTER_TURN 80
#define HALF_TURN 160
#define FULL_TURN 390

/* These constants are the ones that need tweaking */
int left_wall_level = 5;
int left_turn_level = 20;
int front_wall_level = 80; // 230;
int speed1 = 60;
int speed2 = 70;
int speed3 = 80;
int Kp =2;
int Kd = 10;
int low_volts = 0x0262;     // 0x0262 6v cut off
int go_level = 200;

/* Text Strings used for OLED display */
const char ver_str[]        = DISPLAY_TEXT "    Ver 2.0.4\n";
char lr_str[]         = DISPLAY_TEXT "Left_______Front\n";
char speed_str[]      = DISPLAY_TEXT "   Speed M/S\n";
char sensor_wf_str[]  = DISPLAY_TEXT " 4 x Sensor WF\n";
char pololu_str[]     = DISPLAY_TEXT "  Pololu Motors\n";
char max_speed_str[]  = DISPLAY_TEXT "Max_Speed     mS\n";
char press_r_str[]    = DISPLAY_TEXT "   Press R_BUT  \n";
char ready_str[]      = DISPLAY_TEXT "   Ready to Go\n";
char sensor_seen_str[]= DISPLAY_TEXT "   Sensor Seen\n";
char running_str[]    = DISPLAY_TEXT "     Running\n";

/* Defines for later use */
#define pwr_led	LATBbits.LATB4
#define right_led LATAbits.LATA4
#define left_led LATCbits.LATC3
#define IR_1    LATAbits.LATA10
#define IR_2    LATAbits.LATA7
#define IR_3    LATBbits.LATB14
#define IR_4    LATBbits.LATB15
#define IR_5    LATAbits.LATA8
#define IR_6    LATAbits.LATA9
#define R_BUT   PORTCbits.RC4
#define L_BUT   PORTCbits.RC5
#define LED1_TRIS _TRISB4
#define LED2_TRIS _TRISA4
#define LED3_TRIS _TRISC3
#define R_BUT_TRIS _TRISC4
#define L_BUT_TRIS _TRISC5
#define IR_1_TRIS _TRISA10
#define IR_2_TRIS _TRISA7
#define IR_3_TRIS _TRISB14
#define IR_4_TRIS _TRISB15
#define IR_5_TRIS _TRISA8
#define IR_6_TRIS _TRISA9
#define L_PWM PDC2
#define R_PWM PDC3
#define SRAM_CS LATAbits.LATA3
#define SRAM_CS_TRIS _TRISA3

//#define SPIOUT_PIN LATBbits.LATB9
//#define SPIOUT_TRIS _TRISB9
//#define SPIIN_PIN LATBbits.LATB7
//#define SPIIN_TRIS _TRISB7


/* Varibables used in code  */
/* unsigned 16 bit integers */
int stop = 0x100;		//
volatile unsigned int tick1;
volatile unsigned int tick2;
volatile unsigned int tick0_1ms;
unsigned int V_bat;     // raw ADC reading for battery. 
// 1024@3.3v 0-3.3 input. divider = 4k7/10k. 
// 1v at battery input = 0.3197v at ADC input
// 1 level = 3300mV/1024 = 3.223mV per level
// 319.7mV / 3.223mV =  99.19 = 1v. 
// 7v = 694.33
// 1/99.19 = 1% increase.. e.g. 694.33 + 1% = 701
// = Hardly worth it.
unsigned int bat_count = 0x0000;
int dark;
int l_dia;
int r_dia;
int l_front;
int r_front;
int old_wall_level;
//int r_count;        //
//int l_count;        //POS1CNT
//int distance;
int wf_dis;
int set_speed;
int max_speed;
volatile int run_speed = 1;
int blink;
int left_pwm;
int right_pwm;
int Rspin;
int Lspin;
int Lturn;
int Rturn;
//int Left_turn_start_distance;
int prop;
/* signed 16 bit integers*/
int  Sprop;
int  last_prop;
int  Deriv;
int  PD_error;

int wall_tracking;      // a counter for how long we've been tracking the wall

/* Floating point variables*/
//float V_batf;

//
// function definitions
//
void delay_milliseconds(unsigned int milliseconds);
void delay_seconds_milliseconds(unsigned int seconds, unsigned int milliseconds);
//void short_delay(unsigned int microseconds);
void Init_SPI(void);

void wait_for_go(void);
void read_RF_sensor(void);
void read_L_sensor(void);
void read_Bat_volts(void);
void sensor_display(void);
void speed_display(int distance);
void low_battery(void);
void init_hardware(void);
#if SERIAL_TERMINAL
void do_commands(void);
#endif
void lturn(int straight_distance, int my_distance, int my_speed);

// borland c like functions
#define kbhit() U1STAbits.URXDA
int getch(void)
{ 
    while (! kbhit()) { 
        // wait
        read_Bat_volts();
    }
    //if ((ustatus->URXDA) == 0) break;
    return U1RXREG;
    
}
int getche(void) { int cbuf = getch(); putchar(cbuf); return cbuf; }
#define clear_overflow() if(U1STAbits.OERR == 1) { U1STAbits.OERR = 0; }


/******************************************************************************/
/*Start of Main Code */
int main()
{
    init_hardware();        //Setup all of the hardware we are going to use
    pwr_led = 1;            //Power LED ON ASAP
    left_led = 0;
    right_led = 0;

#if SERIAL_TERMINAL
    if (!L_BUT && !R_BUT)
    {
        do_commands();
    }
#endif
    
    if (!L_BUT){            //if L_BUT held at power on go to sensor display
        sensor_display();}
    if (!R_BUT){            //if R_BUT held at power on go to display speed
        speed_display(0);}
    
    // If But_A Not pressed then wait for it to be pressed and released
    while (R_BUT)          // wait for button to be pressed
    {
    blink = run_speed;
        while (blink > 0)
        {
        tick1 = 0;
        left_led = 1;
        //right_led = 1;
        while (tick1 < 2000);
        left_led = 0;
        //right_led = 0;
        while (tick1 < 5000);
        blink--;
        }
        if (kbhit())
        {
            getch();
            do_commands();
        }

    tick2 = 0;
    while (tick2 < 10000){if(!R_BUT) tick2 = 10001;}    
    }
/******************************************************************************/    
    right_led = 1;
    while (!R_BUT);         // wait for it to be released
    left_led = 0;
    right_led = 0;
/******************************************************************************/    
    if (run_speed == 1){max_speed = speed1;}
    if (run_speed == 2){max_speed = speed2;}    
    if (run_speed == 3){max_speed = speed3;}
/******************************************************************************/    
    wait_for_go();          //button A has been pressed now wait for finger seen
/******************************************************************************/
/*Clear LCD, Enable PWM, Start Timer for Ticks, get required wall distance and set speed*/   
    PTCONbits.PTEN = 1;     // Start PWM
    left_led = 0;
    right_led = 0;
    
    delay_seconds_milliseconds(2,0);    // Delay 2 sec before starting off
    
//set the distance to stay away from the left wall  
    read_L_sensor();
    wf_dis = l_dia;  //set the distance away from wall
    if(wf_dis == 0){
        wf_dis = 1;
        }
//calculate motor speed settings
    set_speed = stop + max_speed;
    Lspin = stop+max_speed; // Speed for Left motor for clockwise spin
    Rspin = stop-max_speed; // Speed for Right motor for clockwise spin
    
    // these might need adjusting for different speeds...
    Lturn = stop+max_speed * 0.5; //0.5 Speed for Left motor for Left turn
    Rturn = stop+max_speed * 1.3; //1.3 Speed for Right motor for Left turn
    //Left_turn_start_distance = 20;

    L_PWM = max_speed;      //set forward speed for L motor
    R_PWM = max_speed;      //set forward speed for R motor
    /******************************************************************************/
    //This is the main program loop for the wall following function*/
    //Follow_Left_wall2:
    while (1)
    {
        tick1 = 0; //reset the tick1 timer for this main loop

        read_RF_sensor();
        if (r_front > front_wall_level)
        {
            L_PWM = Lspin; //Spin clockwise
            R_PWM = Rspin;
            POS1CNT = 0; //zero the Left wheel counter
            while (POS1CNT < 100); // Left wheel counter
            //tick2 = 0;          //reset tick2 timer for Right spin time
            //L_PWM = Rspin;      //Spin anticlockwise to stop
            //R_PWM = Lspin;      // over shoot
            //while(tick2 < 20){} //2mS
            wall_tracking = 0;
            // added for wall tracking purposes
            POS1CNT = 0; //zero the Left wheel counter
            POS2CNT = 0; //zero the Left wheel counter
        }

        read_L_sensor();

        if ((wf_dis - l_dia) > 0)
        {
            //to far right
            PD_error = (wf_dis - l_dia) / 2;
        }
        else
        {
            //to far left
            PD_error = (wf_dis - l_dia) / 20; //8    
        }
        /******************************************************************/
        if (PD_error > left_turn_level)
        {
            left_led = 0;
            // carry on going a short way
            //POS2CNT = 0; //zero the Left wheel counter
            //while (-(POS2CNT) < Left_turn_start_distance); // right wheel counter

#define TRACKED_DISTANCE 100     // 100 = 100/2 * 0.6 = 3cm,     50 = 50/2 * 0.6mm = 1.5cm
//#define SELECT_ALTERATIVE_LTURN
#define SELECT_ALTERATIVE_LTURN2
//#define SELECT_ALTERATIVE_LTURN3
//#define SELECT_ALTERATIVE_LTURN4
#ifdef SELECT_ALTERATIVE_LTURN
            
            while(1)
            {
                // 30mm forward, then 
                lturn(70, 300, max_speed);
                read_L_sensor();

                if ((wf_dis - l_dia) > 0)
                {
                    //to far right
                    PD_error = (wf_dis - l_dia) / 2;
                }
                else
                {
                    //to far left
                    PD_error = (wf_dis - l_dia) / 20; //8    
                }
                /******************************************************************/
                if (PD_error < left_turn_level)
                {
                    break;
                }
            }
#elif defined(SELECT_ALTERATIVE_LTURN2)
            if(wall_tracking > TRACKED_DISTANCE)
            {
                // 30mm forward, then turn
                lturn(70, 300, max_speed);
            }
            else
            {
                // we are not tracking a wall, no need to clear it
                lturn(0, 50, max_speed);
            }
#elif defined(SELECT_ALTERATIVE_LTURN3)
            // we could try to go forward for 30mm after wall tracking only
            // then use the partial turn code below, rather than a full quarter turn
            // then continue around loop
            if(wall_tracking > TRACKED_DISTANCE)
            {
                L_PWM = stop+my_speed;// Speed for Left motor for clockwise spin
                R_PWM = stop+my_speed; // Speed for Right motor for clockwise spin

                PTCONbits.PTEN = 1;     // Start PWM

                POS1CNT = 0; //zero the Left wheel counter
                while (POS1CNT < 70); // Left wheel counter
            }
            
            // now do the turn
            L_PWM = Lturn; //Smooth turn to the left
            R_PWM = Rturn; //Smooth turn to the left
            POS2CNT = 0; //zero the Left wheel counter
            while (-(POS2CNT) < 50); // right wheel counter
            
#elif defined(SELECT_ALTERNAIVE_LTURN4)
            // we could try to go forward for 30mm after wall tracking only
            // then use the partial turn code below, rather than a full quarter turn
            // and continue turning until we see a left hand wall...
#else
            // now do the turn
            L_PWM = Lturn; //Smooth turn to the left
            R_PWM = Rturn; //Smooth turn to the left
            POS2CNT = 0; //zero the Left wheel counter
            while (-(POS2CNT) < 50); // right wheel counter
#endif
            PD_error = 0;
            wall_tracking = 0;
        }
        else
        {
            if(wall_tracking < 10000)
            {
                wall_tracking += (POS1CNT-POS2CNT);  // 0.6mm = 2 counts
            }
            if(wall_tracking > TRACKED_DISTANCE)
            {
                left_led = 1;
            }
       }
        // added for wall tracking purposes
        POS1CNT = 0; //zero the Left wheel counter
        POS2CNT = 0; //zero the Left wheel counter

/******************************************************************/
//PD_control();

// set_motors:
    left_pwm = set_speed - PD_error;	//apply error to left wheel
    if(left_pwm < 0){left_pwm = 0;}
    if(left_pwm > 0x200){left_pwm = 0x200;}
    L_PWM = left_pwm;
    
    right_pwm = set_speed + PD_error;	//apply error to right wheel
    if(right_pwm < 0){right_pwm = 0;}
    if(right_pwm > 0x200){right_pwm = 0x200;}
    R_PWM = right_pwm;

    read_Bat_volts();
    
    while(tick1 < 20){} //5mS loop time
    } 
}
/******************************************************************************/
void PD_control(void)
{
   prop = wf_dis - l_dia;
   Deriv = prop - last_prop;
   PD_error = prop/Kp + Deriv * Kd;//3 and 20
   last_prop = prop;   
}
/******************************************************************************/
/* Timer interupt routine called every 0.1mS by T1 timer */
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt(void)
    {
    IFS0bits.T1IF = 0; 		/* clear interrupt flag */
    tick1++;
    tick2++;
    tick0_1ms++;
}
/******************************************************************************/
/* Change Notification interrupt routine */
void __attribute__((__interrupt__,__auto_psv__)) _CNInterrupt(void)
    {
    // ISR Code goes here
    if(!L_BUT){run_speed++;}
    if(run_speed > 3){run_speed = 1;}
    // Remember to do this next time !!!!
    IFS1bits.CNIF = 0;      //clear CN interrupt flag   
    }
/******************************************************************************/
//wait for Left wall sensor to see a hand
void wait_for_go(void){
    clear_screen();
    printf (TEXT_RETURN);
    printf (TEXT_RETURN);
    puts(ready_str);
    printf (TEXT_RETURN);
    printf (DISPLAY_TEXT "  -------------\n");
 
    read_L_sensor();
    //go_level = l_dia + 10;
    old_wall_level = left_wall_level;
    left_wall_level = go_level;         //set wall sensor detection level for go 
    while(l_dia < go_level)      //level/while sensor level is below go
    {                                   //keep reading the L sensor
        read_L_sensor();
    }
    left_led = 1;
    clear_screen();
    printf (TEXT_RETURN);
    printf (TEXT_RETURN);
    puts(sensor_seen_str);
    printf (TEXT_RETURN);
    printf (DISPLAY_TEXT "  -------------\n");
     
    while(l_dia > go_level-10)   //set wall sensor detection lower than
    {                                   //it was to give some hysteresis
        read_L_sensor();                //keep reading the L sensor
    }
    left_led = 0;                       //make sure the L Led is not left ON
    left_wall_level = old_wall_level;   //set wall sensor detection level back
    left_led = 0;
    clear_screen();
    printf (TEXT_RETURN);
    printf (TEXT_RETURN);
    puts(running_str);
    printf (TEXT_RETURN);
    printf (DISPLAY_TEXT "  -------------\n");
}                                       
/******************************************************************************/
void read_RF_sensor(void){
    /* Read Sensor 1, r_front, AN0, IR_1*/
    AD1CHS0 = 0;            //0 = AN0
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #225");      //10us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    while (!AD1CON1bits.DONE);
    dark = ADC1BUF0;
    IR_1 = 1;
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #1125");      //50us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    IR_1 = 0;
    while (!AD1CON1bits.DONE);
    r_front = ADC1BUF0;
    r_front = r_front-dark;
    if(r_front < 0){r_front = 0;}
    }
void read_R_sensor(void){
    /* Read Sensor 2, r_dia, AN1, IR_2 */
    AD1CHS0 = 01;            //01 = AN1
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #225");      //10us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    while (!AD1CON1bits.DONE);
    dark = ADC1BUF0;
    IR_2 = 1;
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #1125");      //50us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    IR_2 = 0;
    while (!AD1CON1bits.DONE);
    r_dia = ADC1BUF0;
    r_dia = r_dia-dark;
    if(r_dia < 0){r_dia = 0;}       
} 
void read_L_sensor(void){
    /* Read Sensor 5, l_dia, AN6, IR_5 */
    AD1CHS0 = 06;            //06 = AN6
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #225");      //10us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    while (!AD1CON1bits.DONE);
    dark = ADC1BUF0;
    IR_5 = 1;
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #1125");      //50us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    IR_5 = 0;
    while (!AD1CON1bits.DONE);
    l_dia = ADC1BUF0;
    l_dia = l_dia-dark;
    if(l_dia < 0){l_dia = 0;}
}
void read_LF_sensor(void){
    /* Read Sensor 6, l_front, AN7, IR_6 */
    AD1CHS0 = 07;            //07 = AN7
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #225");      //10us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    while (!AD1CON1bits.DONE);
    dark = ADC1BUF0;
    IR_6 = 1;
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #1125");      //50us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    IR_6 = 0;
    while (!AD1CON1bits.DONE);
    l_front = ADC1BUF0;
    l_front = l_front-dark;
    if(l_front < 0){l_front = 0;}
    }
/******************************************************************************/
    /* Measure battery voltage 10k/4k7 divider = Bat volts * 0.47 */
    /* 6v cut off = 6v x 0.47 = 2.82v = 875 = 0x036B ADC value */
void read_Bat_volts(void){
    AD1CHS0 = 8;            //8 = AN8 used for VBat
    AD1CON1bits.SAMP = 1;    //Start sampling
    asm("repeat #225");      //10us delay
    asm("nop");
    AD1CON1bits.SAMP = 0;    //Stop sampling start converting
    while (!AD1CON1bits.DONE);
    V_bat = ADC1BUF0;
    if (V_bat < low_volts){
        bat_count++;}
    if  (V_bat > low_volts){
        bat_count = 0;}
    if (bat_count > 1000){
        low_battery();}
    }
/******************************************************************************/
void send_data(void){
    clear_screen();
    printf(TEXT_RETURN);
    printf (DISPLAY_TEXT " %4d\r\n",prop);
    printf (DISPLAY_TEXT " %4d\r\n",Sprop);
    printf (DISPLAY_TEXT " %4d\r\n",Deriv);
    printf(TEXT_RETURN);
    printf (DISPLAY_TEXT " %4d\r\n",PD_error);
    printf (DISPLAY_TEXT " %4d\r\n",L_PWM);
    printf (DISPLAY_TEXT " %4d\r\n",R_PWM);
}
/******************************************************************************/

void sensor_display(void){
    
    while(! kbhit())
    {
        read_L_sensor();
        read_LF_sensor();
        read_R_sensor();
        read_RF_sensor();

        printf ("    Sensor's\r\n");
        printf ("%6d Frnt %6d\r\n",l_front, r_front);
        printf ("\r\n");
        printf ("%6d Diag %6d\r\n",l_dia, r_dia); 

        delay_seconds_milliseconds(1,0);
    }
    getch();
}
/******************************************************************************/
void speed_display(int distance){
    int temp;
    if(distance <= 0)
    {
        distance = 1790;
    }
    cursor_off();         //Set Cursor OFF
    max_speed = 20;
    
    while(! kbhit())
    {
        clear_screen();          //Clear Screen
        puts(speed_str);
        printf (TEXT_RETURN);
        printf (TEXT_RETURN);
        puts(max_speed_str);
        printf (TEXT_RETURN);

        delay_seconds_milliseconds(2,0);
        L_PWM = stop + max_speed;      //set forward speed for L motor
        R_PWM = stop + max_speed;      //set forward speed for R motor
        //PR1 = 0x0166;           // set period1 register 0x0166 = 1mS    
        PTCONbits.PTEN = 1;     // Start PWM
        POS1CNT = 0;
        tick1 = 0;

        while(POS1CNT < 1790){} //1m of wheel rotations
        temp = tick1;
        L_PWM = stop;
        R_PWM = stop;

        printf (DISPLAY_TEXT "%d\r\n",max_speed);
        printf (DISPLAY_TEXT "         \n");
        printf (DISPLAY_TEXT "%d\r\n",temp/10);
        printf (TEXT_RETURN);

       // If But_A Not pressed then wait for it to be pressed and released
        while (R_BUT);          // wait for button to be pressed
        while (!R_BUT);         // wait for it to be released    
        //while(1){}        //at 4 MIPS 2400000 at 23 MIPS    
        max_speed = max_speed + 5;
    }
    getch();

}
/******************************************************************************/
void maze_display(void){
    //cursor_off();         //Set Cursor OFF
    clear_screen();          //Clear Screen
    printf ("DR");
    putchar(0); // top left X
    putchar(0); // top left Y
    putchar(63);// bottom right X
    putchar(63);// bottom right Y
    printf ("LN");
    putchar(3); // top left X
    putchar(63); // top left Y
    putchar(3);// bottom right X
    putchar(60);// bottom right Y    
disp_loop:
    goto disp_loop;
}
/******************************************************************************/
// Come here and loop forever when battery volts low
void low_battery(void){
    PTCONbits.PTEN = 0;     // Stop PWM
    //T1CONbits.TON = 0;      // Don't stop Timer ... we use it for timing still
    pwr_led = 0;
    left_led = 0;
    right_led = 0;
    L_PWM = stop;
    R_PWM = stop;
    
    printf("/r/nHALT:Low Bat %d mV/r/n", (int)(V_bat*10.1));
    
    //Blink PWR LED to show mouse shut down due to low bat
    while(1)
    {
        delay_seconds_milliseconds(1,430);
        pwr_led = 1;
        delay_seconds_milliseconds(0,130);
        pwr_led = 0;
    }
}
/******************************************************************************/
/* Here we Initialize all of the I/O pins and hardware setup                  */
/******************************************************************************/
void init_hardware(void)
    {
    //These symbols are defined at beginning of Code
    LED1_TRIS = 0;          //Pin 2 port A0 for LED1 grn_led output
    LED2_TRIS = 0;          //Pin 3 port A1 for LED2 red_led output
    LED3_TRIS = 0;          //Pin 6 port B3 for LED3 Yel_led output
    R_BUT_TRIS = 1;         //Pin 9 port A2 for R_BUT input
    L_BUT_TRIS = 1;         //Pin 9 port A2 for R_BUT input
    _TRISB10 = 0;           //set pin 8 for output for PWM1H3
    _TRISB11 = 0;           //set pin 9 for output for PWM1L3
    _TRISB12 = 0;           //set pin 10 for output for PWM1H2
    _TRISB13 = 0;           //set pin 11 for output for PWM1L2
    IR_1_TRIS = 0;
    IR_2_TRIS = 0;
    IR_3_TRIS = 0;
    IR_4_TRIS = 0;
    IR_5_TRIS = 0;
    IR_6_TRIS = 0;
    _TRISB5 = 0;           //set pin 41 for output for Uart TX
    SRAM_CS_TRIS = 0;       // set pin 31 for output to allow control of SRAM CS
    SRAM_CS = 1;            // CS is active low, so disable
    
/******************************************************************************/
    /* Setup Remappable Pins*/
    RPINR18 = 0XFF06;       //Uart 1 RX mapped to RP6 pin 42
    RPOR2 = 0X0300;         //Uart 1 TX mapped to RP5 pin 41
    RPINR14 =0x1716;        //Pins 2(RP22), 3(RP23) for QEI 1 inputs
    RPINR16 =0x1918;        //Pins 4(RP24), 5(RP25) for QEI 2 inputs
    
    Init_SPI();
    
    /* Configure the UART */
    U1MODE = 0x8400;        // 2 stop bits needed for crownhill display
    U1STA = 0;
    U1MODEbits.UARTEN = 1;
    U1BRG = 148;             // 37 =38400, 148 = 9600
    U1STAbits.UTXEN = 1;
/******************************************************************************/
    /* Configure OLED Display and send Ver No */ 
    //printf ("SB38400\r"); //Set Baud rate to 38400
    cursor_off();         //Set Cursor OFF
    clear_screen();          //Clear Screen
    printf (TEXT_RETURN);         //Start New Line
    //printf ("DC0");         //Turn OFF Config display
    //printf ("DSS0");        //Turn OFF Start up Screen
    puts(sensor_wf_str);             //Print what is in the Buffer5    
    //printf (TEXT_RETURN);
    //puts(pololu_str);           //Print what is in the Buffer6   
    //printf (TEXT_RETURN);
    printf (TEXT_RETURN);
    puts(ver_str);             //Print what is in the Buffer1 Ver x.x.x
    printf (TEXT_RETURN);
    printf (TEXT_RETURN);
    puts(press_r_str);             //Print what is in the Buffer8
/******************************************************************************/   
    /* Configure T1 Timer Interrupt */
    _T1IE = 0;          // T1 Interrupt OFF
    TMR1 = 0;           /* clear timer1 register */
    PR1 = 0x023;       // set period1 register 0x023=0.1mS
    T1CONbits.TCKPS = 2;// T1 prescaler 3=256 2=64 1=8 0=1
    T1CONbits.TCS = 0; 	/* set internal clock source */
    IPC0bits.T1IP = 4; 	/* set priority level */
    IFS0bits.T1IF = 0; 	/* clear interrupt flag */
    IEC0bits.T1IE = 1; 	/* enable interrupts */
    SRbits.IPL = 3;   	/* enable CPU priority levels 4-7 */
    T1CONbits.TON = 1;      // Start Timer
/******************************************************************************/
    /* Configure Pin Change Interrupt*/
    CNEN2bits.CN26IE = 1;   //set RC5 L_BUT for change interrupt
    IEC1bits.CNIE = 1;      //enable CN interrupt  
    IFS1bits.CNIF = 0;      //clear CN interrupt flag
/******************************************************************************/   
    /* Configure the QEI */
    QEI1CON = 0X0400;   /* QEI 1 x2 Mode, Enabled, Inputs NOT Swapped*/
    QEI2CON = 0X0400;   /* QEI 2 x2 Mode, Enables, Inputs NOT Swapped*/
/******************************************************************************/
    /* Configure the ADC */
    AD1CON1bits.ADON = 1;
    //AD1PCFGL = 0X007F;
/******************************************************************************/
    /* Configure PWM */
    PTPER = stop;         // Timebase for 89KHz PWM
    PWMCON1 = 0x0066;       // Us PWM1H2, PWM1L2,PWM1H3,PWM1L3 pins
    P1TCON = 0x0000;         // Fcy / x1 prescaler
    L_PWM = stop;          // Period should give 50:50 = Stop
    R_PWM = stop;
    }
/******************************************************************************/

// Subtraction of two n-bit values wraps modulo 2n - which C99 guarantees for subtraction of unsigned values.
// Assume unsigned to signed cast will work. Alternatively could use:
// #include <limits.h>
    // unsigned subtraction always works for wrap around, as long as it doesn't
    // wrap twice:
    //   * Obviously works if start < curent
    //   * If start very high number (say -half time, and end is +half time)
    //     then while tick is less than wrap then XXXXX
    //          while tick is more than wrap, but less than +half time then YYYYY
    //          while tick is more than +half time then ZZZZZ

unsigned int calculate_wait_end(unsigned int length_in_milliseconds)
{
    return tick0_1ms + (10 * length_in_milliseconds);
}

int wait_expired(unsigned int end)
{
    //return (tick0_1ms - end) < UINT_MAX/2;
    return ((signed int)(tick0_1ms - end)) >= 0;
}

int wait_not_expired(unsigned int end)
{
    //return (tick0_1ms - end) >= UINT_MAX/2;
    return ((signed int)(tick0_1ms - end)) < 0;
}


// Max at 16 bit = 3 seconds
void delay_milliseconds(unsigned int milliseconds)
{
    while(milliseconds > 3000)
    {
        delay_milliseconds(3000);
        milliseconds -= 3000;
    }
    
    int end = calculate_wait_end(milliseconds);
    
    while(wait_not_expired(end))
    {
        // wait until the time is up
    }
}


// Max at 16 bit = 3 seconds
void delay_seconds_milliseconds(unsigned int seconds, unsigned int milliseconds)
{
    // avoid overflow because of 16 bit
    while(seconds > 2)
    {
        delay_milliseconds(2000);
        seconds -= 2;
    }
    while(milliseconds > 2000)
    {
        delay_milliseconds(2000);
        milliseconds -= 2;
    }
    
    // slightly inefficient, but convenient
    delay_milliseconds(seconds*1000 + milliseconds);
}

/******************************************************************************/
#if SERIAL_TERMINAL
//
void read_line(char *buffer, int max_len)
{
    int c;
    int chars_left = max_len;
    // clear errors
    clear_overflow();
    
    // read a line
    while(chars_left > 0)
    {
        c = getch();
        if(c == 10 || c == 13)
        {
            break;
        }
        if(c == '\x7f')
        {
            if(chars_left != max_len)
            {
                //puts("\x1B[1D");
                printf("\b \b");
                buffer--;
                chars_left++;
            }
            continue;
        }
        *buffer = c;
        putchar(*buffer);
        buffer++;
        chars_left--;
    }
    // zero terminate it always
    if(chars_left==0)
    {
        buffer--;
    }
    *buffer = 0;
    
    printf(TEXT_RETURN);
}

int running = 1;

void exit_cmd(const char* args)
{
    running = 0;
}

void ticks_cmd(const char* args)
{
   printf("%d" TEXT_RETURN, tick0_1ms);
}
void ms_cmd(const char* args)
{
   printf("%d" TEXT_RETURN, tick0_1ms/10);
}

void delay_cmd(const char* args)
{
    unsigned int delay_in_seconds = 1;
    if(args != 0)
    {
        sscanf(args, "%u", &delay_in_seconds);
    }
    delay_seconds_milliseconds(delay_in_seconds, 0);
}

void bat_cmd()
{
    // 10 bit mode = 1024 levels
    // divider = 4k7 / 10k
    // ADC Ref = 3.3v
    // Therefore voltage in mV = (value / (ADC_ref * 1024)) * 14k7/4k7 * 1000 
    //                        = value * 10.079
    printf("%d mV" TEXT_RETURN, (int)(V_bat*10.1));
}

void led_cmd(const char* args)
{
    unsigned int led = 0;
    unsigned int led_on = 1;
    if(args != 0)
    {
        sscanf(args, "%u %u", &led, &led_on);
    }
    led_on = led_on?1:0;
    if(led == 0)
    {
        left_led = led_on;
    }
    else if(led == 1)
    {
        pwr_led = led_on;
    }
    else
    {
        right_led = led_on;
    }
}

void wait_for_go_cmd(const char* args)
{
    unsigned int runs = 1;
    if(args != 0)
    {
        sscanf(args, "%u", &runs);
    }
    while(runs)
    {
        printf("Run %u\n\r", runs);
        wait_for_go();
        runs --;
    }
}

void action_test_cmd(const char* args)
{
    //set the distance to stay away from the left wall  
    read_L_sensor();
    wf_dis = l_dia; //set the distance away from wall
    if (wf_dis == 0)
    {
        wf_dis = 1;
    }

    /******************************************************************************/
    //This is the main program loop for the wall following function*/
    //Follow_Left_wall2:
    while (1)
    {
        tick1 = 0; //reset the tick1 timer for this main loop

        read_RF_sensor();
        if (r_front > front_wall_level)
        {
            printf("Spin Right\r\n");
        }

        read_L_sensor();

        if ((wf_dis - l_dia) > 0)
        {
            //to far right
            PD_error = (wf_dis - l_dia) / 2;
            if (PD_error > left_turn_level)
            {
                printf("Turn Left\r\n");
            }
            else
            {
                printf("Drift Left\r\n");
            }
        }
        else
        {
            //to far left
            PD_error = (wf_dis - l_dia) / 20; //8    
            printf("Drift Right\r\n");
        }
        /******************************************************************/
 
        read_Bat_volts();

        while (tick1 < 5000) // 0.5s loop time
        { // abort if serial key pressed
            if (kbhit())
            {
                break;
            }
        }
        if (kbhit())
        {
            getch();
            break;
        }
    }
}

void set_fw_cmd(const char* args)
{
    int level = -1;
    if(args != 0)
    {
        sscanf(args, "%d", &level);
    }

    if(level > 0)
    {
        front_wall_level = level;
    }
}

void lturn_cmd (const char* args)
{
    int straight_distance = 20;
    int my_distance = 50;
    int my_speed = speed1;
    if(args != 0)
    {
        sscanf(args, "%d %d %d", &straight_distance, &my_distance, &my_speed);
    }
    printf("%i %i %i\r\n", straight_distance, my_distance, my_speed);
    lturn(straight_distance, my_distance, my_speed);
    
    L_PWM = stop;
    R_PWM = stop;  
}

void lturn(int straight_distance, int my_distance, int my_speed)
{
    Lturn = stop + my_speed * 0.5; //0.5 Speed for Left motor for Left turn
    Rturn = stop + my_speed * 1.3; //1.3 Speed for Right motor for Left turn

    L_PWM = stop+my_speed;// Speed for Left motor for clockwise spin
    R_PWM = stop+my_speed; // Speed for Right motor for clockwise spin

    PTCONbits.PTEN = 1;     // Start PWM

        // 100 counts = 56mm, quarter turn is 60.5mm
    POS1CNT = 0; //zero the Left wheel counter
    while (POS1CNT < straight_distance); // Left wheel counter

    L_PWM = Lturn; //Smooth turn to the left
    R_PWM = Rturn; //Smooth turn to the left
    
    POS2CNT = 0; //zero the Left wheel counter
    //printf("%i %i\r\n", POS1CNT, POS2CNT);
    while (-(POS2CNT) < my_distance); // right wheel counter
    //printf("%i %i\r\n", POS1CNT, POS2CNT);

}


void lturn_tweak(int straight_distance, int my_distance, int my_speed, int lturn_ratio, int rturn_ratio)
{
    float lturn_f = lturn_ratio / 10.0;
    float rturn_f = rturn_ratio / 10.0;
    Lturn = stop + my_speed * lturn_f; //0.5 Speed for Left motor for Left turn
    Rturn = stop + my_speed * rturn_f; //1.3 Speed for Right motor for Left turn

    L_PWM = stop+my_speed;// Speed for Left motor for clockwise spin
    R_PWM = stop+my_speed; // Speed for Right motor for clockwise spin

    PTCONbits.PTEN = 1;     // Start PWM

        // 100 counts = 56mm, quarter turn is 60.5mm
    POS1CNT = 0; //zero the Left wheel counter
    while (POS1CNT < straight_distance); // Left wheel counter

    L_PWM = Lturn; //Smooth turn to the left
    R_PWM = Rturn; //Smooth turn to the left
    
    POS2CNT = 0; //zero the Left wheel counter
    //printf("%i %i\r\n", POS1CNT, POS2CNT);
    while (-(POS2CNT) < my_distance); // right wheel counter
    //printf("%i %i\r\n", POS1CNT, POS2CNT);

}

void lturn_tweak_cmd (const char* args)
{
    int straight_distance = 20;
    int my_distance = 50;
    int my_speed = speed1;
    int lturn = 0.5;
    int rturn = 1.3;
    
    if(args != 0)
    {
        sscanf(args, "%d %d %d %d %d", &straight_distance, &my_distance, &my_speed, &lturn, &rturn);
    }
    printf("%i %i %i %i %i\r\n", straight_distance, my_distance, my_speed, lturn, rturn);
    lturn_tweak(straight_distance, my_distance, my_speed, lturn, rturn);
    
    L_PWM = stop;
    R_PWM = stop;  
}


void rspin_cmd (const char* args)
{
    int my_distance = 100;
    int my_speed = speed1;
    if(args != 0)
    {
        sscanf(args, "%d %d", &my_distance, &my_speed);
    }

    L_PWM = stop+my_speed;// Speed for Left motor for clockwise spin
    R_PWM = stop-my_speed; // Speed for Right motor for clockwise spin

    PTCONbits.PTEN = 1;     // Start PWM

    // 100 counts = 56mm, quarter turn is 60.5mm
    POS1CNT = 0; //zero the Left wheel counter
    while (POS1CNT < my_distance); // Left wheel counter

    L_PWM = stop;
    R_PWM = stop;  
}

void fwd_cmd(const char* args)
{
    int my_distance = 100;
    int my_speed = speed1;
    if(args != 0)
    {
        sscanf(args, "%d %d", &my_distance, &my_speed);
    }
    
    L_PWM = stop+my_speed;
    R_PWM = stop+my_speed;

    PTCONbits.PTEN = 1;     // Start PWM

    // 100 counts = 56mm, quarter turn is 60.5mm
    POS1CNT = 0; //zero the Left wheel counter
    while (POS1CNT < my_distance); // Left wheel counter

    L_PWM = stop;
    R_PWM = stop;  
}

void speed_display_cmd(const char* args)
{
    int my_distance = 0;
    if(args != 0)
    {
        sscanf(args, "%d", &my_distance);
    }
    speed_display(my_distance);
}

void sensor_display_cmd(const char* args)
{
    sensor_display();
}

void Init_SPI(void)
{
    // spi pin setup
    RPINR20 = 0xFF07;// SPI1 Data Input, SPI IN, RP7, pin 43   
    RPOR4 = 0x0708; // function 8 = SPI1 Clock Output, SPI CLK RP8 (lower byte), pin 44)
                    // function 7 = SPI1 Data Output, SPI OUT, RP9 (upper byte), pin 1

    // setup the SPI peripheral
    SPI1STAT = 0x0;  // disable the SPI module (just in case)
    //
    // SFR Name Addr Bit 15 Bit 14 Bit 13 Bit 12 Bit 11 Bit 10 Bit 9 Bit 8 Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0
  // SPI1CON1   0242   —      —       —   DISSCK DISSDO MODE16  SMP    CKE  SSEN CKP   MSTEN ----SPRE<2:0>----  PPRE<1:0>  Reset Value = 0000
//  SPI1CON1 = 0x0161;	// FRAMEN = 0, SPIFSD = 0, DISSDO = 0, MODE16 = 0; SMP = 0; CKP = 1; CKE = 1; SSEN = 0; MSTEN = 1; SPRE = 0b000, PPRE = 0b01
    SPI1CON1 = 0x057B;	//             SPIFSD = 0, DISSDO = 0, MODE16 = 1; SMP = 0; CKP = 1; CKE = 1; SSEN = 0; MSTEN = 1; SPRE = 0b110, PPRE = 0b11    
//    SPI1CON1 = 0x0573;	//             SPIFSD = 0, DISSDO = 0, MODE16 = 1; SMP = 0; CKP = 1; CKE = 1; SSEN = 0; MSTEN = 1; SPRE = 0b110, PPRE = 0b11
    // 
    // main clock is 23 MHz (/2 of Fosc)
    // SPI Peripheral clock is /2 = 12MHz.
    //
    // 111 = Secondary prescale 1:1
    // 110 = Secondary prescale 2:1     <<<-- will give 12 MHz (with primary set at 1:1). However logic analyser is 20 Msps
    // 101 = 3:1
    // 100 = 4:1
    // 011 = 5:1
    // 010 = 6:1
    // 001 = 7:1
    // 000 = Secondary prescale 8:1
    //
    // 11 = Primary prescale 1:1
    // 10 = Primary prescale 4:1
    // 01 = Primary prescale 16:1
    // 00 = Primary prescale 64:1
    // SMP: SPIx Data Input Sample Phase bit Master mode:
    //          0 = Input data sampled at middle of data output time 
    // CKE: SPIx Clock Edge Select bit(1) 
    //          1 = Serial output data changes on transition from active clock state to Idle clock state (see bit 6) 
    // CKP: Clock Polarity Select bit 
    //          0 = Idle state for clock is a low level; active state is a high level 
    SPI1CON1bits.CKE = 0x01;
    SPI1CON1bits.CKP = 0x00;
    SPI1STAT = 0x8000; // enable the SPI module 
}

unsigned short _SPI_wait_read_buf(void)
{
    while (!SPI1STATbits.SPIRBF)	// wait for the data to be read
        ;
    return SPI1BUF;
}

void start_SPI_write(unsigned short address)
{
    short temp;
    SRAM_CS = 0;		// lower the slave select line
    // pause required here? 25ns required, instruction time = 43ns. Should be ok :-)
    
    temp = SPI1BUF;		// dummy read of the SPI1BUF register to clear the SPIRBF flag
    temp = (address&0x8000) ? 0x0201 : 0x0200;        // instruction 0x02 is write
    SPI1BUF = temp;		// write the data out to the SPI peripheral
    
    _SPI_wait_read_buf();
    
    SPI1BUF = (unsigned short)(address << 1);   // emulate 16 bit address

    // the second part of the address is now being sent out, while the function returns
}

void SPI_write_u16(unsigned short data)
{
    _SPI_wait_read_buf();    
    SPI1BUF = data;
}
#define SPI_write_s16(data) SPI_write_u16((unsigned short)data)

void SPI_write_finish()
{    
    _SPI_wait_read_buf();

    // Need to wait over 1 bit after . At 12MHz this is 83ns.
    // At 6MHz this is 176ns (used for logic analyser)
    // Instructions are 43ns. 
    //asm("repeat #5");      //200ns delay
    //asm("nop");
    // actual delay is much longer because of call overhead
    
    SRAM_CS = 1;		// raise the slave select line    
}


void start_SPI_read(unsigned short address)
{
    short temp;	
    SRAM_CS = 0;		// lower the slave select line
    // pause required here? 25ns required, instruction time = 43ns. Should be ok :-)
    
    temp = SPI1BUF;		// dummy read of the SPI1BUF register to clear the SPIRBF flag
    temp = (address&0x8000) ? 0x0301 : 0x0300;        // instruction 0x03 is read
    SPI1BUF = temp;		// write the data out to the SPI peripheral
    _SPI_wait_read_buf();    
    
    SPI1BUF = (unsigned short)(address << 1);   // emulate 16 bit address
    _SPI_wait_read_buf();
}

unsigned short SPI_read_u16(void)
{
    SPI1BUF = 0; // dummy value
    return _SPI_wait_read_buf();
}

#define SPI_read_s16() ((signed short)SPI_read_u16())

void SPI_read_finish()
{
    // Need to wait over 1 bit after . At 12MHz this is 83ns.
    // At 6MHz this is 176ns (used for logic analyser)
    // Instructions are 43ns. 
    //asm("repeat #5");      //200ns delay
    //asm("nop");
    // actual delay is much longer because of call overhead

    SRAM_CS = 1;		// raise the slave select line    
}


/*
void write_SPI(short command)
{	
    short temp;	

    SRAM_CS = 0;		// lower the slave select line
    temp = SPI1BUF;		// dummy read of the SPI1BUF register to clear the SPIRBF flag
    SPI1BUF = command;		// write the data out to the SPI peripheral
    while (!SPI1STATbits.SPIRBF)	// wait for the data to be sent out
            ;
    SRAM_CS = 1;		// raise the slave select line
}
*/


void calexport_cmd(const char* args)
{
    // Put mouse at back wall, in the center.
    // Move forward slowly 34mm or 60 (34/0.6) steps
    // (Mouse is about 100mm long, inside is 180-12 = 168mm. Therefore movement is 168mm-100mm/2)
    int my_speed = 20;
    unsigned short address = 0;
    unsigned short count = 0;
    unsigned short data;
    
    // move_forward 60
    L_PWM = stop+my_speed;
    R_PWM = stop+my_speed;

    PTCONbits.PTEN = 1;     // Start PWM

    // 100 counts = 56mm, quarter turn is 60.5mm
    POS1CNT = 0; //zero the Left wheel counter
    while (POS1CNT < 60); // Left wheel counter

    L_PWM = stop;
    R_PWM = stop;  
    
    start_SPI_write(address);

    delay_seconds_milliseconds(2, 0);
    
    // 100 counts = 56mm, quarter turn is 60.5mm
    POS1CNT = 0; //zero the Left wheel counter
    POS2CNT = 0;
    
    // Rotate 360 degrees slowly
    // Record sensor readings (SPI RAM) and rotation angle
    L_PWM = stop+my_speed;// Speed for Left motor for clockwise spin
    R_PWM = stop-my_speed; // Speed for Right motor for clockwise spin

    while (POS1CNT < FULL_TURN || count == 9362) // Left wheel counter
    {
        // so we can calculate how long the sensor readings take
        SPI_write_u16(tick0_1ms);
        SPI_write_u16(POS1CNT);
        SPI_write_u16(POS2CNT);
        read_RF_sensor();
        SPI_write_u16(r_front);
        read_L_sensor();
        SPI_write_u16(l_dia);
        read_R_sensor();
        SPI_write_u16(r_dia);
        read_LF_sensor();
        SPI_write_u16(l_front);
        count++;
    }
    SPI_write_finish();

    L_PWM = stop;
    R_PWM = stop;  
        
    // Stream out to serial port to allow spreadsheet analysis
    // 960cps for 14 values, would be ~68 values in one second
    // better to RAM buffer?
    printf("%hu\r\n", count);
    start_SPI_read(address);
    for(address = 0; address < count; address++)
    {
        int i;
        for(i = 0; i < 7; i++)
        {
            data = SPI_read_u16();
            printf("%hu ", data);
        }
        printf("\r\n");
    }
    SPI_read_finish();
}


void por_cmd(const char* args)
{
    printf("%u\r\n", RCON);
}

void spiw_cmd(const char* args)
{
    unsigned short addr = 0;
    unsigned short data = 0;
    if(args != 0)
    {
        sscanf(args, "%hu %hu", &addr, &data);
    }
    printf("Write %hu to %hu\r\n", data, addr);
    start_SPI_write(addr);
    SPI_write_u16(data);
    SPI_write_finish();
}

void spiw2_cmd(const char* args)
{
    unsigned short addr = 0;
    unsigned short data = 0;
    unsigned short data2 = 0;
    if(args != 0)
    {
        sscanf(args, "%hu %hu %hu", &addr, &data, &data2);
    }
    printf("Write %hu then %hu to %hu\r\n", data, data2, addr);
    start_SPI_write(addr);
    SPI_write_u16(data);
    SPI_write_u16(data2);
    SPI_write_finish();
}

void spir_cmd(const char* args)
{
    unsigned short addr = 0;
    unsigned short data;
    if(args != 0)
    {
        sscanf(args, "%hu", &addr);
    }
    start_SPI_read(addr);
    data = SPI_read_u16();
    SPI_read_finish();
    printf("Read %hu from %hu\r\n", data, addr);
}

void spi_test_cmd(const char* args)
{
    unsigned short addr;
    unsigned short data;

    // one less than max.
    for(addr = 0; addr < 65535; addr++)
    {
        start_SPI_write(addr);
        SPI_write_u16(addr);
        SPI_write_finish();
    }

    for(addr = 0; addr < 65535; addr++)
    {
        start_SPI_read(addr);
        data = SPI_read_u16();
        SPI_read_finish();
        if(data != addr)
        {
            printf("Failed %hu read %hu\r\n", addr, data);
            return;
        }
    }
    printf("Pass\r\n");
}

/*
void spio_test_cmd(const char* args)
{
    int i;
    SPIOUT_TRIS = 0;
    for(i = 0; i < 100; i++)
    {
        SPIOUT_PIN = 1;
        delay_seconds_milliseconds(0,1);
        SPIOUT_PIN = 0;   
        delay_seconds_milliseconds(0,1);
    }
    SPIOUT_TRIS = 1;
}

void spii_test_cmd(const char* args)
{
    int i;
    SPIIN_TRIS = 0;
    for(i = 0; i < 100; i++)
    {
        SPIIN_PIN = 1;
        delay_seconds_milliseconds(0,1);
        SPIIN_PIN = 0;   
        delay_seconds_milliseconds(0,1);
    }
    SPIIN_TRIS = 1;
}
*/

void help_cmd(const char* args);

typedef struct { 
    const char* cmd;
    void (*func)(const char* args);
} command_type;

command_type commands[] = {
    { "ticks", ticks_cmd },
    { "ms", ms_cmd },
    { "delay", delay_cmd },
    { "exit", exit_cmd },
    { "?", help_cmd },
    { "help", help_cmd },
    { "bat", bat_cmd },
    { "led", led_cmd },
    { "wait_for_go", wait_for_go_cmd },
    { "action_test", action_test_cmd },
    { "lturn", lturn_cmd },
    { "rspin", rspin_cmd },
    { "set_fw", set_fw_cmd },
    { "fwd", fwd_cmd },
    { "speed_display", speed_display_cmd },
    { "sensor_display", sensor_display_cmd },
    { "calexport", calexport_cmd },
    { "por", por_cmd },
    { "spiw", spiw_cmd },
    { "spiw2", spiw2_cmd },
    { "spir", spir_cmd },
    { "spi_test", spi_test_cmd },
    //{ "spio_test", spio_test_cmd },
    //{ "spii_test", spii_test_cmd },

    { 0, 0}
};

void help_cmd(const char* args)
{
    command_type* cp = commands;
    const char* cmd;
    while( (cmd = cp++->cmd) )
    {
        printf("%s ", cmd);
    }
    printf(TEXT_RETURN);
}

#define CMD_LEN 80
char cmdbuf[CMD_LEN+1];

void run_command(char* str)
{
    if(str == 0) { return; }
    
    command_type* cp = commands;
    char* token = strtok(str, " \t"); 
    while(1)
    {

        if(cp->cmd == 0 || token == 0)
        {
            printf("?" TEXT_RETURN);
            break;
        }
        
        //printf("Checking %s", cp->cmd);
        if(strcmp(cp->cmd, token)==0)
        {
            cp->func(strtok(NULL, ""));
            break;
        }
        cp++;
    }
}


void do_commands(void)
{
    running = 1;
    while(running)
    {
        printf(TEXT_RETURN "> ");
        read_line(cmdbuf, CMD_LEN);
        run_command(cmdbuf);
    }
}

#endif