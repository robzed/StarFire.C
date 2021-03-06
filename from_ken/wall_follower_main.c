
/*******************************************************************************
 2 wheeled WF Mouse using 33FJ128MC804 processor with 2 x QEI
and Pololu Mini Motors
File Name: main.c  By Ken Hewitt 5/5/2019

Description: 
2.0.1 Rob's wall following code
2.0.2 My left turn code
2.0.3 code for int on change to select run speed

3 pulse encoder, 30:1 gearing, 32mm wheel, 90ppr x2 mode=180ppr = 0.56mm
16 bits gives max distance of 2293mm
 
This version is useing only 4 sensors 2 x 30 and 2 x forward
*******************************************************************************/
#pragma config FNOSC = FRCPLL // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config FWDTEN = OFF   // Watchdog Timer Enable (Watchdog timer
                              // enabled/disabled by user software)
#include <stdio.h>
#include <stdlib.h>
#include  <xc.h>

/* These constants are the ones that need tweaking */
int left_wall_level = 5;
int left_turn_level = 20;
int front_wall_level = 230;
int speed1 = 60;
int speed2 = 70;
int speed3 = 80;
int Kp =2;
int Kd = 10;
int low_volts = 0x0262;     // 0x0262 6v cut off
int go_level = 200;

/* Text Strings used for OLED display */
char buf1[] = "TT    Ver 2.0.3\n";
char buf2[] = "TT    Sensor's\n";
char buf3[] = "TTLeft_______Front\n";
char buf4[] = "TT   Speed M/S\n";
char buf5[] = "TT 4 x Sensor WF\n";
char buf6[] = "TT  Pololu Motors\n";
char buf7[] = "TTMax_Speed     mS\n";
char buf8[] = "TT   Press R_BUT  \n";
char buf9[] = "TT   Ready to Go\n";
char buf10[]= "TT   Sensor Seen\n";
char buf11[]= "TT     Running\n";

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

/* Varibables used in code  */
/* unsigned 16 bit integers */
int stop = 0x100;		//
int long delay;
int temp;
unsigned int tick1;
unsigned int tick2;
unsigned int V_bat;
unsigned int bat_count = 0x0000;
int dark;
int l_dia;
int r_dia;
int l_front;
int r_front;
int old_wall_level;
int r_count;        //
int l_count;        //POS1CNT
int distance;
int wf_dis;
int set_speed;
int max_speed;
int run_speed = 1;
int blink;
int left_pwm;
int right_pwm;
int Rspin;
int Lspin;
int Lturn;
int Rturn;
int prop;
/* signed 16 bit integers*/
int  Sprop;
int  last_prop;
int  Deriv;
int  PD_error;

/* Floating point variables*/
float V_batf;
/******************************************************************************/
/*Start of Main Code */
int main()
{
    init_hardware();        //Setup all of the hardware we are going to use
    pwr_led = 1;            //Power LED ON ASAP
    left_led = 0;
    right_led = 0;

    if (!L_BUT){            //if L_BUT held at power on go to sensor display
        sensor_display();}
    if (!R_BUT){            //if R_BUT held at power on go to display speed
        speed_display();}
    
    // If But_A Not pressed then wait for it to be pressed and released
    T1CONbits.TON = 1;      // Start Timer
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
    
    tick1 = 0;               // clear the 0.1mS tick timer
    while(tick1 < 20000){}   // Delay 2 sec before starting off
    
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
    Lturn = stop+max_speed * 0.6; //0.5 Speed for Left motor for Left turn
    Rturn = stop+max_speed * 1.2; //1.3 Speed for Right motor for Left turn
    L_PWM = max_speed;      //set forward speed for L motor
    R_PWM = max_speed;      //set forward speed for R motor
/******************************************************************************/
//This is the main program loop for the wall following function*/
Follow_Left_wall2:
while (1)
{
tick1 = 0;              //reset the tick1 timer for this main loop

    read_RF_sensor();
    if (r_front > front_wall_level)
    {
        L_PWM = Lspin;      //Spin clockwise
        R_PWM = Rspin;
        POS1CNT = 0;        //zero the Left wheel counter
        while(POS1CNT < 100); // Left wheel counter
        //tick2 = 0;          //reset tick2 timer for Right spin time
        //L_PWM = Rspin;      //Spin anticlockwise to stop
        //R_PWM = Lspin;      // over shoot
        //while(tick2 < 20){} //2mS
    }     
    
read_L_sensor();

if((wf_dis - l_dia) > 0)
{
    //to far right
    PD_error = (wf_dis - l_dia)/2;
}   
else
{
    //to far left
    PD_error = (wf_dis - l_dia)/20; //8    
}
/******************************************************************/ 
if (PD_error > left_turn_level)
{
    L_PWM = Lturn;      //Smooth turn to the left
    R_PWM = Rturn;      //Smooth turn to the left
    POS2CNT = 0;        //zero the Left wheel counter
    while(POS2CNT < 50); // right wheel counter
    PD_error = 0;
}
/******************************************************************/
//PD_control();

 set_motors:
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
    printf ("CL");
    printf ("TRT");
    printf ("TRT");
    puts(buf9);
    printf ("TRT");
    printf ("TT  -------------\n");
 
    read_L_sensor();
    //go_level = l_dia + 10;
    old_wall_level = left_wall_level;
    left_wall_level = go_level;         //set wall sensor detection level for go 
    while(l_dia < go_level)      //level/while sensor level is below go
    {                                   //keep reading the L sensor
        read_L_sensor();
    }
    printf ("CL");
    printf ("TRT");
    printf ("TRT");
    puts(buf10);
    printf ("TRT");
    printf ("TT  -------------\n");
     
    while(l_dia > go_level-10)   //set wall sensor detection lower than
    {                                   //it was to give some hysteresis
        read_L_sensor();                //keep reading the L sensor
    }
    left_led = 0;                       //make sure the L Led is not left ON
    left_wall_level = old_wall_level;   //set wall sensor detection level back
    printf ("CL");
    printf ("TRT");
    printf ("TRT");
    puts(buf11);
    printf ("TRT");
    printf ("TT  -------------\n");
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
        bat_count = bat_count++;}
    if  (V_bat > low_volts){
        bat_count = 0;}
    if (bat_count > 1000){
        low_battery();}
    }
/******************************************************************************/
void send_data(void){
    printf ("CL");
    printf("TRT");
    printf ("TT %4d\r\n",prop);
    printf ("TT %4d\r\n",Sprop);
    printf ("TT %4d\r\n",Deriv);
    printf("TRT");
    printf ("TT %4d\r\n",PD_error);
    printf ("TT %4d\r\n",L_PWM);
    printf ("TT %4d\r\n",R_PWM);
}
/******************************************************************************/
void sensor_display(void){
    printf ("CS0");         //Set Cursor OFF
sensor_disp_loop:
    read_L_sensor();
    read_LF_sensor();
    read_R_sensor();
    read_RF_sensor();
    printf ("CL");          //Clear Screen
    printf ("TT    Sensor's\n");
    printf ("TRT");         //Start New Line 
    //printf ("TT_Lf__________Rt_\n");
    puts(buf2);
    printf ("TRT");    
    printf ("TT%04x\r\n",l_front);
    printf ("TT  Frnt  \n");
    printf ("TT%04x\r\n",r_front);
    printf ("TRT");
    printf ("TT%04x\r\n",l_dia);
    printf ("TT  Diag  \n");
    printf ("TT%04x\r\n",r_dia); 
    printf ("TRT");
    //printf ("TT%04x\r\n",l_side);
    //printf ("TT  Side  \n");
    //printf ("TT%04x\r\n",r_side); 
    delay = 100000;        //This value 200000 gives about 2sec delay ?
    while(delay--){}        //at 4 MIPS 2400000 at 23 MIPS    
    goto sensor_disp_loop;
}
/******************************************************************************/
void speed_display(void){
    printf ("CS0");         //Set Cursor OFF
    max_speed = 20;
speed_disp_loop:    
    printf ("CL");          //Clear Screen
    puts(buf4);
    printf ("TRT");
    printf ("TRT");
    puts(buf7);
    printf ("TRT");
    
    delay = 2000000;        //This value 200000 gives about 2sec delay ?
    while(delay--){}        //at 4 MIPS 2400000 at 23 MIPS 
    L_PWM = stop + max_speed;      //set forward speed for L motor
    R_PWM = stop + max_speed;      //set forward speed for R motor
    PR1 = 0x0166;           // set period1 register 0x0166 = 1mS    
    PTCONbits.PTEN = 1;     // Start PWM
    T1CONbits.TON = 1;      // Start Timer
    POS1CNT = 0;
    tick1 = 0;
    
    while(POS1CNT < 1790){} //1m of wheel rotations
    temp = tick1;
    L_PWM = stop;
    R_PWM = stop;      

    printf ("TT%d\r\n",max_speed);
    printf ("TT         \n");
    printf ("TT%d\r\n",temp);
    printf ("TRT");
 
   // If But_A Not pressed then wait for it to be pressed and released
    while (R_BUT);          // wait for button to be pressed
    while (!R_BUT);         // wait for it to be released    
    //while(1){}        //at 4 MIPS 2400000 at 23 MIPS    
    max_speed = max_speed + 5;
    goto speed_disp_loop;
}
/******************************************************************************/
void maze_display(void){
    //printf ("CS0");         //Set Cursor OFF
    printf ("CL");          //Clear Screen
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
    T1CONbits.TON = 0;      // Stop Timer
    pwr_led = 0;
    left_led = 0;
    right_led = 0;
    L_PWM = stop;
    R_PWM = stop;
low_bat_loop:               //Blink PWR LED to show mouse shut down due to low bat
    delay = 2000000;        //This value 2000000 gives about 1.43sec delay
    while(delay--){}        
    pwr_led = 1;
    delay = 200000;         //This value 200000 gives about 130mS delay
    while(delay--){}        
    pwr_led = 0;
    goto low_bat_loop;
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
/******************************************************************************/
    /* Setup Remappable Pins*/
    RPINR18 = 0X0006;       //Uart 1 RX mapped to RP6 pin 42
    RPOR2 = 0X0300;         //Uart 1 TX mapped to RP5 pin 41
    RPINR14 =0x1716;        //Pins 2(RP22), 3(RP23) for QEI 1 inputs
    RPINR16 =0x1918;        //Pins 4(RP24), 5(RP25) for QEI 2 inputs

    /* Configure the UART */
    U1MODE = 0x8400;        // 2 stop bits needed for crownhill display
    U1STA = 0;
    U1MODEbits.UARTEN = 1;
    U1BRG = 148;             // 37 =38400, 148 = 9600
    U1STAbits.UTXEN = 1;
/******************************************************************************/
    /* Configure OLED Display and send Ver No */ 
    //printf ("SB38400\r"); //Set Baud rate to 38400
    printf ("CS0");         //Set Cursor OFF
    printf ("CL");          //Clear Screen
    printf ("TRT");         //Start New Line
    //printf ("DC0");         //Turn OFF Config display
    //printf ("DSS0");        //Turn OFF Start up Screen
    puts(buf5);             //Print what is in the Buffer5    
    //printf ("TRT");
    //puts(buf6);           //Print what is in the Buffer6   
    //printf ("TRT");
    printf ("TRT");
    puts(buf1);             //Print what is in the Buffer1 Ver x.x.x
    printf ("TRT");
    printf ("TRT");
    puts(buf8);             //Print what is in the Buffer8
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