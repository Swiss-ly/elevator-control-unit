/*
 * Author: Shea Hucock
 * Student Number: u1148525
 */



#include <xc.h>
#include "lcd.h"
#include <stdio.h>

#define _XTAL_FREQ 20000000
#pragma config WDT = OFF

#define A  PORTBbits.RB2 //Call Up Floor 1
#define B  PORTBbits.RB3 //Call Down Floor 2
#define C  PORTBbits.RB4 //Call Up Floor 2
#define D  PORTBbits.RB5 //Call Down Floor 3
#define E  PORTBbits.RB6 //Select Floor 1
#define F  PORTBbits.RB7 //Select Floor 2
#define G  PORTAbits.RA6 //Select Floor 3
#define H  PORTAbits.RA4//Door Ajar
#define S  PORTCbits.RC2 //Winch Speed (PWM)
#define U  PORTAbits.RA1 //Winch Direction (Up = 1, Down = 0)
#define P  PORTAbits.RA2 //Elevator Position Sensor
#define O  PORTAbits.RA3 //Elevator Door Relay
#define OPEN PORTBbits.RB0 //Door Open
#define CLOSE PORTBbits.RB1 //Door Close
#define BUZZER PORTCbits.RC1 //Buzzer
#define COUNTER PORTCbits.RC3 //To find the loop time
#define UP 1;
#define DOWN 0;

//Declaring global variables and function prototypes
unsigned int adc_value;
int scaled_adc;
unsigned int timer0_count = 0;
int moving = 0;
int test = 0;

int read_adc();
void adc_init();
void init(void);
void pwm_init();
void interrupt_init();
int level();
int floorCall();
int floorSelect();
void doorControl(int cmd);
void motorControl(int spd);
void LCDControl();
void LCDControl2();
int doorButton ();
void BuzzerControl(int cmd);
void __interrupt() ISR(void);
void delay(int time);
void elevatorStart(int destination);
void elevatorStop();
int doorAjar();
int elevatorMoving(int flr);
int direction(int destination);
void maintenance();

void main() {
    lcd_init();
    init();
    adc_init();
    pwm_init();
    interrupt_init();
    while(1) {
        LCDControl(); //Updates LCD
        //Tests the states of the floor call, floor select buttons and door ajar
        if (floorCall()) {
            if (!A) {
                direction(1);
                elevatorStart(1);
            }
            else if (!B) {
                direction(2);
                elevatorStart(2);
                direction(1); //sets direction to down
            }
            else if (!C) {
                direction(2);
                elevatorStart(2);
                direction(3); //sets direction to up
            }
            else if (!D) {
                direction(3);
                elevatorStart(3);
            }
            elevatorStop();
        }
        if (floorSelect()) {
            if (E) {
                elevatorStart(1);
            }
            else if (F) {
                elevatorStart(2);
            }
            else if (G) {
                elevatorStart(3);
            }
            elevatorStop();
        }
        if (doorButton()) {
            if ((level() == 1) && !OPEN && !CLOSE) { //WARNING: For some reason it only activates when the close button is pressed first
                maintenance();
                lcd_cmd(L_CLR);
            }
            else if (!OPEN) {
                if (moving == 1){
                }
                else {
                    doorControl(1);
                    delay(1600);
                    doorControl(2);
                }
            }
            else if (!CLOSE) {
                doorControl(2);
            }
        }
        LCDControl();
    }
}

void init(void) {
    TRISA = 0b01110100; //sets all but RA2, RA4 & RA6 to outputs
    TRISB = 0b11111111; //sets the B port to inputs
    TRISC = 0b00000000; //sets all other registers to known states
    TRISD = 0;
    TRISE = 0;
    LATA = 0;
    LATB = 1;
    LATC = 0;
    LATD = 0;
    LATE = 0;
}

void adc_init() {
    ADCON1 = 0b00001100; //sets the analog input channel to AN2(RA2)
    ADCON2 = 0b10010101; //sets the clock and acquisition time for the ADC
    ADCON0 = 0b00001001;//sets the channel to read to AN2 and turns on the ADC
}

void pwm_init() {
    CCP1CONbits.CCP1M = 0b1100; //Sets CCP1 module to PWM mode
    T2CON = 0b00000110; //Turns on Timer2 and sets prescaler to 16
    PR2 = 124; //Desired Frequency: 2500, Fosc = 20MHz, TMR2prescaler = 16
}

void interrupt_init() {
    T0CON = 0b110000100; // Turns on Timer0 with prescale value of 1:32 on a 8 bit timer
    TMR0H = 253; //Sets the timers initial value
    INTCON = 0b11100000; //Enables global and Timer0 interrupts
    INTCON3bits.INT1IP = 1; //High priority for int2
    INTCON3bits.INT1IE = 1; //Enables int2
    INTCON3bits.INT1IF = 0; 
    INTCON2bits.INTEDG1 = 1; //Enables flag on rising edge
}

int read_adc() {
    //sets the adc to read mode and returns the value
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO);
    return ADRES;
}

int level(){
    unsigned int floor;
    adc_value = read_adc();
    scaled_adc = (float)adc_value * 6/1023*1000-200; //converts the digital value from adc to mm
    if (scaled_adc > 5475 && scaled_adc < 5725) {
        floor = 3;
    }
    else if (scaled_adc > 2675 && scaled_adc < 2925) {
        floor = 2;
}
    else if (scaled_adc > -125 && scaled_adc < 125) {
        floor = 1;
    }
    else {
        floor = -1;
    }
    return floor;
}

int floorCall() {
    if (!A || !B || !C || !D) { //detects if any of these buttons are pressed
        return 1;
    }
    else {
        return 0;
    }
}

int floorSelect() {
    if (E || F || G) {//detects if any of these buttons are pressed
        return 1;
    }
    else {
        return 0;
    }
}

int doorAjar() {//detects if the sensor is inputting a LOW value
    if (!H) {
        return 1;
    }
    else {
        return 0;
    }
}

int doorButton (){
    if (!OPEN || !CLOSE) {
        return 1;
    }
    else {
        return 0;
    }
}

void doorControl(int cmd) {
    switch(cmd){
        case(1): //Opens the door
            O = 1;
            break;
        case(2): //Closes the door
            O = 0;
            break;
    }
}



void motorControl(int spd) {
    int PWMfreq = 2500;
    int ontime = (spd*(_XTAL_FREQ/PWMfreq))/160; //the intended range of spd is 0-10 to set the PWM output between 0% - 100% in increments of 10%
    unsigned int lbit = (ontime+2)/4; 
    unsigned int hbit = ontime - lbit;
    CCPR1L = lbit; 
    CCP1CONbits.DC1B = hbit;
    }

void LCDControl(){
    char line1[16];
    char line2[16];
    if (level() > 0) {
        sprintf(line1, "Floor:%2d", level());
        }
        else {
            sprintf(line1, "Floor: -");
        }
    lcd_cmd(L_L1);
    lcd_str(line1);
}

void LCDControl2() {
    char line2[16];
    sprintf(line2, "%4dmm", scaled_adc);
    lcd_cmd(L_L2);
    lcd_str(line2);
}

void BuzzerControl(int cmd) {
    switch(cmd){
        case(1): //Turns on Buzzer
            BUZZER = 1;
            break;
        case(2): //Turns off Buzzer
            BUZZER = 0;
            break;
    }
}

void __interrupt() ISR(void){
    if (INTCONbits.TMR0IF == 1){
        timer0_count++;
        COUNTER = ~COUNTER;
        TMR0H = 253; //Set to 253 so the timer overflows every 5ms
        INTCONbits.TMR0IF = 0; //Resets interrupt flag
    }
    if (INTCON3bits.INT1IF == 1) {
        doorControl(2); // When the Door Close button is pressed the microcontroller will override the door open signal to close the door
        INTCON3bits.INT1IF = 0;
    }
}


void delay(int time) { //Note: this delay works in increments of 5ms so a delay of 500ms = delay(100)
    int initial_time = timer0_count; //Notes the time on the clock the command was taken
    while (timer0_count - initial_time < time); //Checks when the time elapsed since the initial time was taken is less than the duration of the delay then it continues the code 
}

int elevatorMoving(int flr){
    moving = 1; //updates variable
    LCDControl(); //Updates the LCD
    if(test == 1) {
            LCDControl2();
        }
    if(level() == flr) {
        motorControl(0); //Turns off motor when it arrives at the desired floor
        moving = 0; //updates variable
        return 0; //Exits the while loop
    }
    else if ((scaled_adc > 5475 && scaled_adc < 5725) || (scaled_adc > 2475 && scaled_adc < 2725) || (scaled_adc > -125 && scaled_adc < 125)) {
        motorControl(2); //Slows down motor when within 0.125m of other floors
        return 1;
    }
    else {
        motorControl(5); //Normal motor speed
        return 1;
    }
}

void elevatorStart(int destination) {
    if (level() == 1 || level() == 3) {
        direction(destination);
    }
    if((U == 1 && level() == 2 && destination < level()) || (U == 0 && level() == 2 && destination > level())){ //logic to prevent a elevator going down to go up unless its on floor 1 or 3
        return;
    }
    doorControl(2);
    while(!doorAjar()) { //waits till the doors are closed
    }
    while(elevatorMoving(destination)) {//runs function til it reaches the desired floor
    }
}

void elevatorStop(){
    BuzzerControl(1);
    doorControl(1); //Opens door
    LCDControl();
    delay(50);
    BuzzerControl(2); //Turn off buzzer after 250ms
    delay(1550);
    doorControl(2);//Close door after 8 seconds
}

int direction(int destination) { 
    if(destination > level()){ //If destination is greater than current level go up
        U = UP;
    }
    else if(destination < level()){//If destination is lesser than current level go down
        U = DOWN;
    }
}

void maintenance() {
    test = 1; //Prints sensor reading on LCD
    PORTCbits.RC4 = 1; //Turns on Maintenance mode light
    direction(3); 
    elevatorStart(3);
    direction(1);
    elevatorStart(1); //Goes up to floor 3 and returns to floor 1
    test = 0;
    PORTCbits.RC4 = 0;
}