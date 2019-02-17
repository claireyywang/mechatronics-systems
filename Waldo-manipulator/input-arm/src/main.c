/* Name: main.c
 * Author: Yuanyuan Wang,Victor Janniaud   
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "teensy_general.h"  // includes the resources included in the teensy_general.h file
#include "t_usb.h"

void initADC(){
    // 1. set the voltage reference: Vcc
    clear(ADMUX,REFS1);
    set(ADMUX,REFS0);

    // 2. set the ADC clock prescaler
    // system at 16MHz, /128
    set(ADCSRA,ADPS2);
    set(ADCSRA,ADPS1);
    set(ADCSRA,ADPS0);

    // set free-running mode
    set(ADCSRA,ADATE); 
}

void setupADC(int channel){
    clear(ADCSRA,ADSC);
    ADMUX = 1;
    // select channel
    if (channel<=7){
        // 3. disable digital inputs 
        DIDR0 |= 1 << channel; // set channel
        clear(ADCSRB,MUX5); // set lower 
        ADMUX = 0b1000000+channel; // make sure ref is not cleared
    }
    if (channel>7){
        DIDR2 |= 1 << (channel-8); // set channel
        set(ADCSRB,MUX5); // set upper 
        ADMUX = 0b1000000+channel-8;// make sure ref is not cleared
    }

    // 5. begin new conversion immediately after finishing a previous one 
    set(ADCSRA,ADEN); // enable ADC
    set(ADCSRA,ADSC); // begin conversion 
}

int readADC(){
    // wait for conversion to finish
    while(bit_is_clear(ADCSRA,ADIF));
    set(ADCSRA,ADIF);
    return ADC;
}

void printADC(int channel, int adc){
    usb_tx_string("\nChannel: ");
    usb_tx_decimal(channel);
    usb_tx_char(' ');
    usb_tx_decimal(adc);
}

void printAngle(int channel, float angle){
    usb_tx_string("\nJoint: ");
    usb_tx_decimal(channel);
    usb_tx_char(' ');
    usb_tx_decimal(angle);
    
}

float readAngle(int adc){
    // reading range 0-1023
    float angle;
    // need to convert to fixed point arithmetic 
    // full angle: 300 
    angle = adc/1023.0*300.0; 
    return angle;
}

int main(void){
    // connect via USB
    m_usb_init();
    while(!m_usb_isconnected());
    teensy_clockdivide(0); //clock speed 16MHz

    initADC();
    int c0 = 0; // select sensing channel
    int c1 = 1; // select sensing channel
    for(;;){
        setupADC(c0); // connect to 1st ADC port
        int adc0 = readADC(); // read 1st ADC value and store in variable
        float angle0 = readAngle(adc0);
        printAngle(c0, angle0);
        // printADC(c0,adc0);
        teensy_wait(100);
        setupADC(c1); // connect to 2nd ADC port
        int adc1 = readADC(); // read 2nd ADC value and store in variable
        float angle1 = readAngle(adc1);
        printAngle(c1, angle1);
        // printADC(c1,adc1);
        teensy_wait(100);
    }
    return 0; /* never reached */
}