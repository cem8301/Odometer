/****** ASEN 4/5519 Lab 6 ******************************************************
 * Author: Carolyn Mason
 * Date  : 12/1/15
 *
 ******************************************************************************/

 
#include <p18cxxx.h>
#include "LCDroutines.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <delays.h>

#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
// LCD 
const rom far char LCDRow1[11] = {0x80,'C','U','R','S','P','E','E','D',0x00};
const rom far char LCDRow2[11] = {0xC0,'x','x','.','x',' ','m','p','h',0x00};
char LCD1_curspeed[11] = {0x80,'C','U','R','R','E','N','T',' ',0x00};
char LCD1_avgspeed[11] = {0x80,'A','V','E','R','A','G','E',' ',0x00};
char LCD1_maxspeed[11] = {0x80,'M','A','X','I','M','U','M',' ',0x00};
char LCD1_time[11] = {0x80,'R','I','D','E','T','I','M','E',0x00};
char LCD1_dist[11] = {0x80,'D','I','S','T','A','N','C','E',0x00};
char LCD1_pulse[11] = {0x80,'P','U','L','S','E',' ',' ',' ',0x00};
char LCD2_speed[11] = {0xC0,'x','x','.','x',' ','m','p','h',0x00};
char LCD2_time[11] = {0xC0,'x','x',':','x','x',':','x','x',0x00};
char LCD2_pulse[11] = {0xC0,'x','x','x',' ',' ','b','p','m',0x00};
char LCD2_dist[11] = {0xC0,'x','x','x','.','x',' ','m','l',0x00};

// Timer
static const unsigned char tenth_sec = 25; // Instruction cycles/10000 for 0.1 second delay

// Pulse Sensor Variables
int i = 0;
unsigned int thresh = 3000;
unsigned int N = 0;
unsigned int Nkeep[2] = {0};
unsigned int rate[10] = {0};
unsigned int BPM = 0;
unsigned int keep2[2] = {0};
char buffer_pulse[4] = {0};
unsigned int signal[2] = {0};
unsigned int beat = 0;

// Hall Effect Sensor Variables
char buffer_curspeed[4] = {0};
char buffer_avgspeed[4] = {0};
char buffer_maxspeed[4] = {0};
char buffer_hr[3] = {0};
char buffer_min[3] = {0};
char buffer_sec[3] = {0};
char buffer_dist[5] = {0};
unsigned int hr = 0;
unsigned int min = 0;
unsigned int sec = 0;
unsigned int currentSpeed = 0;
unsigned int avgSpeed = 0;
unsigned int maxSpeed = 0;
unsigned int rideTime = 0;
unsigned int totalDistance = 0;
unsigned int keep[2] = {0x0FFF,0x0FFF};
unsigned int N2 = 150;
unsigned int Nkeep2[2] = {0};
unsigned int count = 0;
unsigned int count2 = 0;

// Switch
int countSw = 1;

char nn[6] = {0};

#pragma idata gpr3

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void PulseSensor(void); 
void HallEffectSensor(void);
void HiPriISR(void);
void LoPriISR(void);
void TMR0handler(void);     // Interrupt handler for TMR1

#pragma code highVector=0x08
void atHighVector(void)
{
 _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18
void atLowVector(void)
{
 _asm GOTO LoPriISR _endasm
}
#pragma code

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                          // Initialize everything
     while(1) {    
        Delay10KTCYx(tenth_sec);   // 0.1second
            
        // Check Switch and increment for display
        if(PORTDbits.RD3 == 0){
            if(countSw == 6){
                countSw = 1;
            }
            else{
                countSw += 1;
            }
        }    
        
        // Current Speed
        if(countSw == 1){
            LCD2_speed[1] = buffer_curspeed[0];
            LCD2_speed[2] = buffer_curspeed[1];
            LCD2_speed[4] = buffer_curspeed[2];
            // Display to LCD
            DisplayV(LCD1_curspeed);
            DisplayV(LCD2_speed);
        }
        // Average Speed
        if(countSw == 2){
            LCD2_speed[1] = buffer_avgspeed[0];
            LCD2_speed[2] = buffer_avgspeed[1];
            LCD2_speed[4] = buffer_avgspeed[2];
            // Display to LCD
            DisplayV(LCD1_avgspeed);
            DisplayV(LCD2_speed);
        }
        // Maximum Speed
        if(countSw == 3){
            LCD2_speed[1] = buffer_maxspeed[0];
            LCD2_speed[2] = buffer_maxspeed[1];
            LCD2_speed[4] = buffer_maxspeed[2];
            // Display to LCD
            DisplayV(LCD1_maxspeed);
            DisplayV(LCD2_speed);
        }
        // Total Distance
        if(countSw == 4){
            LCD2_dist[1] = buffer_dist[0];
            LCD2_dist[2] = buffer_dist[1];
            LCD2_dist[3] = buffer_dist[2];
            LCD2_dist[5] = buffer_dist[3];
            // Display to LCD
            DisplayV(LCD1_dist);
            DisplayV(LCD2_dist);
        }
        // Ride Time
        if(countSw == 5){
            LCD2_time[1] = buffer_hr[0];
            LCD2_time[2] = buffer_hr[1];
            LCD2_time[4] = buffer_min[0];
            LCD2_time[5] = buffer_min[1];
            LCD2_time[7] = buffer_sec[0];
            LCD2_time[8] = buffer_sec[1];
            // Display to LCD
            DisplayV(LCD1_time);
            DisplayV(LCD2_time);
        }
        // Pulse
        if(countSw == 6){
            LCD2_pulse[1] = buffer_pulse[0];
            LCD2_pulse[2] = buffer_pulse[1];
            LCD2_pulse[3] = buffer_pulse[2];
            // Display to LCD
            DisplayV(LCD1_pulse);
            DisplayV(LCD2_pulse);
        }                                               
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 ******************************************************************************/
void Initial() {  
    // Configure the IO ports
    TRISB = 0b00000000;              // LED's (output)
    LATB = 0b00010000;
    TRISA = 0b11111111;              // Pulse senor on AN10 (J5 pin 11), hall effect AN11 (pin 10) (inputs)
    TRISC = 0b11010000;              // LCD
    LATC = 0b10000000;   
    TRISD = 0b11111111;              // Switch (input)
    
    // Configure the LCD pins for output
    LCD_RS_TRIS   = 0;
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b00001111;     // Note the LCD is only on the upper nibble, The lower nibble is all inputs
    
    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);
    DisplayC(LCDRow2);

    // Initializing TMR0
    T0CON = 0b00001000;             // B'00001000'
    TMR0H = 0x00;
    TMR0L = 0x00;                   // Set count to 2^16-5000

    // Configuring Interrupts for blinking LED 6
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    
    // Set up ADC for pulse sensor 
    ADCON0 = 0b00101001;            // Select AN10 and turn on A/d, Not set to go yet
//    ADCON0 = 0b00101101;            // AN11 for hall effect sensor
    ADCON1 = 0b00000000;            // Vdd and Vss as ref voltage 
    ADCON2 = 0b10101001;            // right justify, TAD = 12, fosc/8
       
    // Set up the ANCON chanel for 10 and 11
    ANCON1 = 0b11111111;      
    
    // Set up the USART
    TRISC = 0b10000000;             // Set bit2 input (RX2), bit1 output (TX2)
    PORTC = 0b00000000;
    TXSTA1 = 0b00100100;            // asynchronus<7>, 8bits<6>,enable transmission<5>, BRGH<2>, no parity, 1 stop bit,
    RCSTA1 = 0b10010000;            // enabled<7> , 8bit<6>, cont received <4>, asynchronus
    BAUDCON1 = 0b00001000;          // 16 bit- use SPBRGHx and SPBRGx
    
    SPBRGH1 = 0;                    // Set baud rate to 19200  
    SPBRG1 = 129;     
    
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
#pragma interrupt HiPriISR
void HiPriISR() {
    
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR1IF and CCP1IF are clear.
 ******************************************************************************/
#pragma interruptlow LoPriISR nosave=TBLPTR, TBLPTRU, TABLAT, PCLATH, PCLATU, PROD, section("MATH_DATA")//, section(".tmpdata")
void LoPriISR() {
    while(1) {
        if( INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }
        
        break;
    }
}

/******************************************************************************
 * Timer handler interrupt service routine
 *
 ******************************************************************************/
void TMR0handler() {
    
    if(ADCON0bits.CHS0 == 1){
        // Check Hall effect sensor
        HallEffectSensor();
        ADCON0 = 0b00101001;
    }
    else{
        // Check Pulse Sensor
        PulseSensor();
        ADCON0 = 0b00101101;
    }
    
    // Reset timer so it interupts every 17ms
    TMR0H = 0xF0;
    TMR0L = 0x00;               // Set count 
    INTCONbits.TMR0IF = 0;      // Clear flag and return to polling routine
    
    // Set the go bit to ADC
    ADCON0bits.GO = 1;  
    
}

/******************************************************************************
 * PulseSensor interrupt service routine
 *
 ******************************************************************************/
void PulseSensor() {          
    // Count
    signal[0] = signal[1];
    signal[1] = ADRES; 
    
    // Check if there is a beat
    if(signal[1] > 4000){
        if(signal[0] < 4000){
            LATBbits.LATB4 = ~LATBbits.LATB4;
            
            Nkeep[0] = Nkeep[1];
            Nkeep[1] = N;
         
            // Calculate bpm
            BPM = 3529/(Nkeep[1] - Nkeep[0]);  // Counts of 17ms between beats (60s/(delN*17ms))
        }
    }
    
    // Increment N
    N += 1;
    
    // Reset N if too large. Will be slight upset in bpm.
    if(N > 65500){
        N = 0; 
    }
    
    // Convert to string to write to LCD later
    sprintf(buffer_pulse,(const rom far char*)"%3u",BPM);
    
    sprintf(nn,(const rom far char*)"%4u",ADRES);
    // debug hall effect sensor
    for(i=0; i<5 ;i++){
        while(PIR1bits.TX1IF == 0) {}
        TXREG1 = nn[i];   
    }
}

/******************************************************************************
 * HallEffectSensor interrupt service routine
 *
 ******************************************************************************/
void HallEffectSensor() {
    // Save latest value 
    keep[0] = keep[1]; // Store old value for comparison
    keep[1] = ADRES;   // Store new value so you don't miss a change
    
    // Check if there is a beat
    if(keep[1] > 3000){
        if(keep[0] < 3000){           
            Nkeep2[0] = Nkeep2[1];
            Nkeep2[1] = N2;
         
            // Calculate mph
            currentSpeed = 5985/2/(Nkeep2[1] - Nkeep2[0]);  // Counts of 17ms between beats (5700=10*60min*60sec*0.0013386995*2*delN/17ms))
            
            // Calculate the total distance (2*10*N*distance/rotation)
            totalDistance = count*0.013386995;

            // Keep track of total ride time, but only if the rider is moving
            if(currentSpeed > 0){
                // Increment E (17ms seconds)
                count += 1;

                // Keep overall average speed in [0]
                avgSpeed = (currentSpeed + count*avgSpeed)/(count+1);
                
                // Turn on speed status LED's
                if(currentSpeed > avgSpeed){
                    LATBbits.LATB5 = 1;
                    LATBbits.LATB6 = 0;
                    LATBbits.LATB7 = 0;
                }
                if(currentSpeed == avgSpeed){
                    LATBbits.LATB5 = 0;
                    LATBbits.LATB6 = 1;
                    LATBbits.LATB7 = 0;
                }
                if(currentSpeed < avgSpeed){
                    LATBbits.LATB5 = 0;
                    LATBbits.LATB6 = 0;
                    LATBbits.LATB7 = 1;
                }          
                
                // Update the max speed
                if(currentSpeed > maxSpeed){
                    maxSpeed = currentSpeed;
                }
            } 
        }
    }
    
    // Increment N
    N2 += 1;
    
    // Set speed to 0 if no change in 2.5 seconds
    if((N2-Nkeep2[1]) > 150){
        currentSpeed = 0;
    }
    else{
        count2 += 1;
        
        // Ride Time
        rideTime = count2*0.0175;

        // Convert to hours and minutes
        hr = rideTime/60/60;
        min = rideTime/60;
        sec = (rideTime % 60);
    }
    // Reset N if too large. Will be slight upset in current speed
    if(N2 > 65500){
        N2 = 150; 
    }
    
    // Convert current speed to string to display later on LCD screen
    sprintf(buffer_curspeed,(const rom far char*)"%3u",currentSpeed);  
    sprintf(buffer_avgspeed,(const rom far char*)"%3u",avgSpeed); 
    sprintf(buffer_maxspeed,(const rom far char*)"%3u",maxSpeed); 
    sprintf(buffer_hr,(const rom far char*)"%2u",hr);
    sprintf(buffer_min,(const rom far char*)"%2u",min);
    sprintf(buffer_sec,(const rom far char*)"%2u",sec);
    sprintf(buffer_dist,(const rom far char*)"%4u",totalDistance);
  
//    // debug hall effect sensor
//    sprintf(nn,(const rom far char*)"%5u",count);
//    for(i=0; i<6 ;i++){
//        while(PIR1bits.TX1IF == 0) {}
//        TXREG1 = nn[i];   
//    }
}
