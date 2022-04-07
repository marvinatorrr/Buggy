/*
 * File:   buggyv2.c
 * Author: M
 *
 * Created on March 12, 2020, 11:17 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "buggyv2.h"
/*
 * 
 */


int count = 0;
int count_off;
int count_on;
int forward_flag = 0;

void init_ADC(void);
void init_PWM(void);
void init_interrupttimer(void);
void init_ports(void);
void setDCmotorspeed(int abs_error);
void reverse(void);
void forwardenable(void);
void TaylorSpeedControl(int error);


int current_position;
int target_position = 0;
int piderror;
int integral;
int derivative;
int last_error;
int pid;

float kp = 1.4;
float ki = 0.05;
float kd = 2;


int pidCalc(int error)
{
    integral = integral + error;
    derivative = error - last_error;
    // Integral Wind Up Reset 
    // Resets integral to 0 if error crosses 0
    if (last_error * error < 0) 
    {
        integral = 0;
    }
    pid = (kp * error) + (kd * derivative) + (ki * integral);
    //kp - faster response but overshoot
    //ki - reduce steady state error
    //kd - minimize overshoot
    last_error = error;

    return(constrain(pid, -1023 ,1023)); //constrain from -1023 to 1023
}


void interrupt ISR()
{
    if(INTCONbits.TMR0IF)//check for flag timer0
    {
        TMR0L -= (100 - 2 - 1);//preload 24us 
        //i tick = 0.5us, overflow = (0.05ms)/8us = 100
        //2 ticks for 2 instruction cycle increment when tmr0 is written
        //1 tick for 1 instruction cycle of writing to tmr0 
        INTCONbits.TMR0IF=0;//clear flag  
        count++;//every increment takes 0.05ms
    }
    
    //left 5%, center 7.5%, right 10%
    
    if(count == 400 - count_on) //off time, desired time/0.05ms
    {
        LATBbits.LATB0=1;
    }
    if(count == 400) //on time, desired time/0.05ms
    {
        LATBbits.LATB0=0;
        count=0;
    }
}

uint16_t ADC_Result[2];
uint16_t error = 0;
uint16_t ADC_Read(uint8_t channel)
{
    switch(channel)
    {
        case 0:
            ADCON0bits.CHS = 0b000;
            break;
        case 1:
            ADCON0bits.CHS = 0b001;
            break;
        default:;
    }
    
    ADCON0bits.GO_nDONE = 1; //start conversion
    
    while (ADCON0bits.GO_nDONE);  //wait for ADC to complete
    return(ADRESH << 2 | ADRESL >> 6); 
}

void main(void) {

    init_ADC();
    init_interrupttimer();       
    init_ports();
    init_PWM();
    
    T2CONbits.TMR2ON = 1; //start timer
    LATBbits.LATB1 = 1; //on g_en
            
    while(1)
    {
        ADC_Result[0] = ADC_Read(0);
        ADC_Result[1] = ADC_Read(1); //0-1023
        
        error = ADC_Result[1] - ADC_Result[0]; // +ve Right | -ve Left  
        count_on = pidCalc(error) * 0.0333 + 30;   
        setDCmotorspeed(abs(error));
        //TaylorSpeedControl(error);
        
//        while(ADC_Result[0] <= 390 && ADC_Result[1] <= 390)
//        {
//            ADC_Result[0] = ADC_Read(0);
//            ADC_Result[1] = ADC_Read(1); //0-1023
//            reverse();
//        }
//        
//        if(forward_flag == 0)
//        {
//            forwardenable();
//        }
            
        PIR1bits.TMR2IF = 0;
        while(PIR1bits.TMR2IF == 0);
    }
    return;
}

//////////////////////////////////////////
///////////////INITIALISATION/////////////
//////////////////////////////////////////

void init_ADC(void)
{
    OSCCON = 0b01111110; //run mode, 8MHz, internal osc block
    
    ADCON0bits.VCFG = 0b00; //Vref+ = AVDD, Vref- = AVSS
    ADCON0bits.GO_nDONE = 0; //ADC status bit
    ADCON0bits.CHS = 0; //channel select
    ADCON0bits.ADON = 1; //enable ADC module
   
    ADCON1 = 0b01111100; // AN0 and AN1 analog, all others digital

    ADCON2bits.ADFM = 0; //left justified
    ADCON2bits.ACQT = 0b100; //8 Tad
    ADCON2bits.ADCS = 0b101; //Fosc/16
}

void init_interrupttimer(void)
{
    TMR0 = 0; //clear preload
    
    INTCONbits.TMR0IE = 1; //enable timer0 overflow interrupt
    INTCONbits.GIE = 1; //enable all global interrupts
    INTCONbits.PEIE = 1; //enable peripheral interrupts
    INTCONbits.TMR0IF = 0; //clear timer0 overflow flag
    
    T0CONbits.T0SE = 0; //edge select low to high
    T0CONbits.PSA = 1; //prescaler not assigned
    T0CONbits.T08BIT = 1; //8 bit timer
    T0CONbits.T0CS = 0; //internal instruction cycle clock
}

void setDCmotorspeed(int abs_error)
{           
            if(abs_error > 270)
            {
                CCPR1L = 31;
            }
            else if((abs_error > 230) && (abs_error <= 270))
            {
                CCPR1L = 35;
            }
            else if((abs_error > 190) && (abs_error <= 230))
            {
                CCPR1L = 36;
            }
            else if((abs_error > 150) && (abs_error <= 190))
            {
                CCPR1L = 40;
            }
            else if(abs_error <= 150)
            {
                CCPR1L = 49;
            }
}

void init_ports(void)
{
    TRISBbits.RB3 = 0; //p1a control sig
    TRISBbits.RB6 = 0; //p1c control sig
    TRISBbits.RB2 = 0; //output p1b pwm
    TRISBbits.RB7 = 0; //output p1d pwm
    TRISBbits.RB0 = 0; //set as output (servo)
    TRISAbits.RA0 = 1; //analog left
    TRISAbits.RA1 = 1; //analog right
    TRISBbits.RB1 = 0; //output global enable
    
    
    TRISAbits.RA2 = 0; //test remove
}

void init_PWM(void)
{
    CCP1CON = 0b01001100; //full bridge forward p1d pwm, p1a control signal
                          //all active high
    
    T2CON = 0b00000000; //prescale 1, postscale 1:1
    PR2 = 199; //calculation PWM Period   =  [(PR2) + 1] ? 4 ? TOSC ?(TMR2 Prescale Value)
}

void reverse(void)
{
    if(forward_flag == 1)
    {
    CCPR1L = 0; //turn off mosfets
    CCP1CONbits.P1M = 0b00;
    Delay10KTCYx(10); 
    }
    
    forward_flag = 0; //disable flag
    CCP1CONbits.P1M = 0b11; //reverse mode p1b pwm, p1c control signal
    CCPR1L = 33; //feed pwm
}

void forwardenable()
{
    CCPR1L = 0; //turn off mosfets
    CCP1CONbits.P1M = 0b00;
    Delay10KTCYx(10); 
    CCP1CONbits.P1M = 0b01; //forward mode p1d pwm, p1a control signal
    forward_flag = 1; //enable flag
}

void TaylorSpeedControl(int error)
{
    float speed;
    float errorRad;
    errorRad = 0.005 * error;
    errorRad = float_constrain(errorRad, -1.54, 1.54);
    speed = 0.245 * (1 - (0.025 * (errorRad * errorRad)));
    CCPR1L = speed * 200;
}
